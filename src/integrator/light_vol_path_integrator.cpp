#include "bulbit/bsdf.h"
#include "bulbit/bxdfs.h"
#include "bulbit/camera.h"
#include "bulbit/film.h"
#include "bulbit/integrators.h"
#include "bulbit/lights.h"
#include "bulbit/material.h"
#include "bulbit/sampler.h"
#include "bulbit/visibility.h"

namespace bulbit
{

LightVolPathIntegrator::LightVolPathIntegrator(
    const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler, int32 max_bounces, int32 rr_min_bounces
)
    : BiDirectionalRayIntegrator(accel, std::move(lights), sampler, std::make_unique<PowerLightSampler>())
    , max_bounces{ max_bounces }
    , rr_min_bounces{ rr_min_bounces }
{
}

SpectrumSample LightVolPathIntegrator::L(
    const Ray& primary_ray,
    const Medium* primary_medium,
    WavelengthSample& lambda,
    const Camera* camera,
    Film& film,
    Sampler& sampler
) const
{
    BulbitNotUsed(primary_ray);
    BulbitNotUsed(primary_medium);

    Float eta_scale = 1;
    Intersection isect;

    // Sample light to start light tracing
    SampledLight sampled_light;
    if (!light_sampler->Sample(&sampled_light, isect, sampler.Next1D()))
    {
        return SpectrumSample(0);
    }

    // Sample point and direction from sampled light
    LightSampleLe light_sample;
    if (!sampled_light.light->Sample_Le(&light_sample, sampler.Next2D(), sampler.Next2D(), lambda))
    {
        return SpectrumSample(0);
    }

    const Medium* medium = light_sample.medium;
    isect.point = light_sample.ray.o;

    SampledCameraSampleWi camera_sample;
    if (camera->SampleWi(&camera_sample, isect, sampler.Next2D(), lambda))
    {
        // Add bounce 0 light to film
        LightSampleLi li_sample;
        if (sampled_light.light->Sample_Li(&li_sample, isect, sampler.Next2D(), lambda))
        {
            if (SpectrumSample V = Tr(this, light_sample.ray.o, camera_sample.p_aperture, medium, lambda); !V.IsBlack())
            {
                SpectrumSample L = V * li_sample.Li * AbsDot(light_sample.normal, camera_sample.wi) *
                                   AbsDot(camera_sample.normal, camera_sample.wi) * camera_sample.Wi /
                                   (sampled_light.pmf * camera_sample.pdf * light_sample.pdf_p);

                film.AddSplat(camera_sample.p_raster, L, lambda);
            }
        }
    }

    int32 bounce = 0;
    Ray ray = light_sample.ray;

    SpectrumSample beta = light_sample.Le / (sampled_light.pmf * light_sample.pdf_p * light_sample.pdf_w);
    if (light_sample.normal != Vec3::zero)
    {
        beta *= AbsDot(light_sample.normal, ray.d);
    }

    SpectrumSample r_u(1);

    // Trace light path
    while (true)
    {
        Vec3 wo = Normalize(-ray.d);
        bool found_intersection = Intersect(&isect, ray, Ray::epsilon, infinity);

        if (medium)
        {
            constexpr int32 hero = WavelengthSample::hero_lane;
            bool scattered = false;
            bool terminated = false;

            Float t_max = found_intersection ? isect.t : infinity;
            Float u = sampler.Next1D();

            uint64 hash0 = Hash(sampler.Next1D());
            uint64 hash1 = Hash(sampler.Next1D());
            RNG rng(hash0, hash1);

            SpectrumSample T_maj = Sample_MajorantTransmittance(
                medium, lambda, ray, t_max, u, rng,
                [&](Point3 p, MediumSample ms, SpectrumSample sigma_maj, SpectrumSample T_maj) -> bool {
                    if (beta.IsBlack())
                    {
                        terminated = true;
                        return false;
                    }

                    SpectrumSample sigma_a = ms.sigma_a;
                    SpectrumSample sigma_s = ms.sigma_s;
                    SpectrumSample Le = ms.Le;
                    Float p_absorb = sigma_a[hero] / sigma_maj[hero];
                    Float p_scatter = sigma_s[hero] / sigma_maj[hero];
                    Float p_null = std::max<Float>(0, 1 - p_absorb - p_scatter);
                    Float events[3] = { p_absorb, p_scatter, p_null };

                    int32 event = SampleDiscrete(events, rng.NextFloat());
                    switch (event)
                    {
                    case 0:
                    {
                        // Sampled absorption event

                        Intersection medium_isect{ .point = p };
                        if (camera->SampleWi(&camera_sample, medium_isect, { rng.NextFloat(), rng.NextFloat() }, lambda))
                        {
                            if (SpectrumSample V = Tr(this, p, camera_sample.p_aperture, medium, lambda); !V.IsBlack())
                            {
                                Float pdf = T_maj[hero] * sigma_a[hero];
                                beta *= T_maj * sigma_a / pdf;
                                r_u *= T_maj * sigma_a / pdf;

                                SpectrumSample L = camera_sample.Wi * V * Le * beta / r_u.Average();
                                film.AddSplat(camera_sample.p_raster, L / camera_sample.pdf, lambda);
                            }
                        }

                        terminated = true;
                        return false;
                    }

                    case 1:
                    {
                        // Sampled real scattering event
                        if (bounce++ >= max_bounces)
                        {
                            terminated = true;
                            return false;
                        }

                        Float pdf = T_maj[hero] * sigma_s[hero];
                        beta *= T_maj * sigma_s / pdf;
                        r_u *= T_maj * sigma_s / pdf;

                        // Add light contribution to film
                        Intersection medium_isect{ .point = p };
                        if (camera->SampleWi(&camera_sample, medium_isect, { rng.NextFloat(), rng.NextFloat() }, lambda))
                        {
                            Vec3 wi = camera_sample.wi;
                            if (SpectrumSample V = Tr(this, p, camera_sample.p_aperture, medium, lambda); !V.IsBlack())
                            {
                                SpectrumSample L = camera_sample.Wi * V * ms.phase->p(wo, wi) * beta / r_u.Average();
                                film.AddSplat(camera_sample.p_raster, L / camera_sample.pdf, lambda);
                            }
                        }

                        // Sample phase function to find next path direction
                        PhaseFunctionSample phase_sample;
                        if (!ms.phase->Sample_p(&phase_sample, wo, sampler.Next2D()))
                        {
                            terminated = true;
                        }

                        beta *= phase_sample.p / phase_sample.pdf;

                        ray.o = p;
                        ray.d = phase_sample.wi;

                        scattered = true;

                        return false;
                    }

                    case 2:
                    {
                        // Sampled null scattering event, continue sampling
                        SpectrumSample sigma_n = Max<Float>(sigma_maj - sigma_a - sigma_s, 0);
                        Float pdf = T_maj[hero] * sigma_n[hero];
                        if (pdf == 0)
                        {
                            beta = SpectrumSample(0);
                        }
                        else
                        {
                            beta *= T_maj * sigma_n / pdf;
                        }

                        r_u *= T_maj * sigma_n / pdf;

                        return !beta.IsBlack();
                    }

                    default:
                        BulbitAssert(false);
                        return false;
                    }
                }
            );

            if (terminated || beta.IsBlack() || r_u.IsBlack())
            {
                break;
            }

            if (scattered)
            {
                // Continue medium sampling
                continue;
            }

            // It past the medium extent
            beta *= T_maj / T_maj[hero];
            r_u *= T_maj / T_maj[hero];
        }

        if (!found_intersection)
        {
            break;
        }

        if (bounce++ >= max_bounces)
        {
            break;
        }

        int8 mem[max_bxdf_size];
        BufferResource res(mem, sizeof(mem));
        Allocator alloc(&res);
        BSDF bsdf;
        if (!isect.GetBSDF(&bsdf, wo, lambda, alloc))
        {
            medium = isect.GetMedium(ray.d);
            ray.o = isect.point;
            --bounce;
            continue;
        }

        if (camera->SampleWi(&camera_sample, isect, sampler.Next2D(), lambda))
        {
            Vec3 wi = camera_sample.wi;

            if (SpectrumSample V = Tr(this, isect.point, camera_sample.p_aperture, isect.GetMedium(wi), lambda); !V.IsBlack())
            {
                SpectrumSample L = camera_sample.Wi * V * bsdf.f(wo, wi, TransportDirection::ToCamera) *
                                   AbsDot(isect.shading.normal, wi) * beta / r_u.Average();

                film.AddSplat(camera_sample.p_raster, L / camera_sample.pdf, lambda);
            }
        }

        // Sample bsdf to find next vertex
        BSDFSample bsdf_sample;
        if (!bsdf.Sample_f(&bsdf_sample, wo, sampler.Next1D(), sampler.Next2D(), TransportDirection::ToCamera))
        {
            break;
        }

        if (bsdf_sample.IsTransmission())
        {
            eta_scale *= Sqr(bsdf_sample.eta);
        }

        // Update path state
        beta *= bsdf_sample.f * AbsDot(bsdf_sample.wi, isect.shading.normal) / bsdf_sample.pdf;
        ray = Ray(isect.point, bsdf_sample.wi);
        medium = isect.GetMedium(bsdf_sample.wi);

        // Terminate path with russian roulette
        if (bounce > rr_min_bounces)
        {
            SpectrumSample rr = beta * eta_scale / r_u.Average();
            if (Float p = rr.MaxComponent(); p < 1)
            {
                if (sampler.Next1D() > p)
                {
                    break;
                }
                else
                {
                    beta /= p;
                }
            }
        }
    }

    return SpectrumSample(0);
}

} // namespace bulbit
