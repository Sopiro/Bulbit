#include "bulbit/bxdfs.h"
#include "bulbit/integrators.h"
#include "bulbit/lights.h"
#include "bulbit/material.h"

namespace bulbit
{

LightVolPathIntegrator::LightVolPathIntegrator(
    const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler, int32 max_bounces
)
    : BiDirectionalRayIntegrator(accel, std::move(lights), sampler)
    , light_sampler{ all_lights }
    , max_bounces{ max_bounces }
{
}

Spectrum LightVolPathIntegrator::L(
    const Ray& primary_ray, const Medium* primary_medium, const Camera* camera, Film& film, Sampler& sampler
) const
{
    BulbitNotUsed(primary_ray);
    BulbitNotUsed(primary_medium);

    int32 wavelength = std::min<int32>(int32(sampler.Next1D() * 3), 2);
    Float eta_scale = 1;
    Intersection isect;

    // Sample light to start light tracing
    SampledLight sampled_light;
    if (!light_sampler.Sample(&sampled_light, isect, sampler.Next1D()))
    {
        return Spectrum::black;
    }

    // Sample point and direction from sampled light
    LightSampleLe light_sample;
    if (!sampled_light.light->Sample_Le(&light_sample, sampler.Next2D(), sampler.Next2D()))
    {
        return Spectrum::black;
    }

    const Medium* medium = light_sample.medium;
    isect.point = light_sample.ray.o;

    CameraSampleWi camera_sample;
    if (camera->SampleWi(&camera_sample, isect, sampler.Next2D()))
    {
        // Add bounce 0 light to film while ignoring delta light contribution
        if (Dot(light_sample.ray.d, camera_sample.wi) > 0 && light_sample.pdf_p != 1 && light_sample.pdf_w != 1)
        {
            if (Spectrum V = Tr(light_sample.ray.o, camera_sample.p_aperture, medium, wavelength); !V.IsBlack())
            {
                Spectrum L = V * light_sample.Le * AbsDot(light_sample.normal, camera_sample.wi) *
                             AbsDot(camera_sample.normal, camera_sample.wi) * camera_sample.Wi /
                             (sampled_light.pmf * camera_sample.pdf * light_sample.pdf_p);

                film.AddSplat(camera_sample.p_raster, L);
            }
        }
    }

    int32 bounce = 0;
    Ray ray = light_sample.ray;

    Spectrum beta =
        light_sample.Le * AbsDot(light_sample.normal, ray.d) / (sampled_light.pmf * light_sample.pdf_p * light_sample.pdf_w);
    Spectrum r_u(1);

    // Trace light path
    while (true)
    {
        Vec3 wo = Normalize(-ray.d);
        bool found_intersection = Intersect(&isect, ray, Ray::epsilon, infinity);

        if (medium)
        {
            bool scattered = false;
            bool terminated = false;

            Float t_max = found_intersection ? isect.t : infinity;
            Float u = sampler.Next1D();

            uint64 hash0 = Hash(sampler.Next1D());
            uint64 hash1 = Hash(sampler.Next1D());
            RNG rng(hash0, hash1);

            Spectrum T_maj = Sample_MajorantTransmittance(
                medium, wavelength, ray, t_max, u, rng,
                [&](Point3 p, MediumSample ms, Spectrum sigma_maj, Spectrum T_maj) -> bool {
                    if (beta.IsBlack())
                    {
                        terminated = true;
                        return false;
                    }

                    Float p_absorb = ms.sigma_a[wavelength] / sigma_maj[wavelength];
                    Float p_scatter = ms.sigma_s[wavelength] / sigma_maj[wavelength];
                    Float p_null = std::max<Float>(0, 1 - p_absorb - p_scatter);
                    Float events[3] = { p_absorb, p_scatter, p_null };

                    int32 event = SampleDiscrete(events, rng.NextFloat());
                    switch (event)
                    {
                    case 0:
                    {
                        // Sampled absorption event

                        Intersection medium_isect{ .point = p };
                        if (camera->SampleWi(&camera_sample, medium_isect, { rng.NextFloat(), rng.NextFloat() }))
                        {
                            if (Spectrum V = Tr(p, camera_sample.p_aperture, medium, wavelength); !V.IsBlack())
                            {
                                Float pdf = T_maj[wavelength] * ms.sigma_a[wavelength];
                                beta *= T_maj * ms.sigma_a / pdf;
                                r_u *= T_maj * ms.sigma_a / pdf;

                                Spectrum L = camera_sample.Wi * V * ms.Le * beta / r_u.Average();
                                film.AddSplat(camera_sample.p_raster, L / camera_sample.pdf);
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

                        Float pdf = T_maj[wavelength] * ms.sigma_s[wavelength];
                        beta *= T_maj * ms.sigma_s / pdf;
                        r_u *= T_maj * ms.sigma_s / pdf;

                        // Add light contribution to film
                        Intersection medium_isect{ .point = p };
                        if (camera->SampleWi(&camera_sample, medium_isect, { rng.NextFloat(), rng.NextFloat() }))
                        {
                            Vec3 wi = camera_sample.wi;
                            if (Spectrum V = Tr(p, camera_sample.p_aperture, medium, wavelength); !V.IsBlack())
                            {
                                Spectrum L = camera_sample.Wi * V * ms.phase->p(wo, wi) * beta / r_u.Average();
                                film.AddSplat(camera_sample.p_raster, L / camera_sample.pdf);
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
                        Spectrum sigma_n = Max<Float>(sigma_maj - ms.sigma_a - ms.sigma_s, 0);
                        Float pdf = T_maj[wavelength] * sigma_n[wavelength];
                        if (pdf == 0)
                        {
                            beta = Spectrum::black;
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
            beta *= T_maj / T_maj[wavelength];
            r_u *= T_maj / T_maj[wavelength];
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
        if (!isect.GetBSDF(&bsdf, wo, alloc))
        {
            medium = isect.GetMedium(ray.d);
            ray.o = isect.point;
            --bounce;
            continue;
        }

        if (camera->SampleWi(&camera_sample, isect, sampler.Next2D()))
        {
            Vec3 wi = camera_sample.wi;

            if (Spectrum V = Tr(isect.point, camera_sample.p_aperture, isect.GetMedium(wi), wavelength); !V.IsBlack())
            {
                Spectrum L = camera_sample.Wi * V * bsdf.f(wo, wi, TransportDirection::ToCamera) *
                             AbsDot(isect.shading.normal, wi) * beta / r_u.Average();

                film.AddSplat(camera_sample.p_raster, L / camera_sample.pdf);
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
        constexpr int32 min_bounces = 2;
        if (bounce > min_bounces)
        {
            Spectrum rr = beta * eta_scale / r_u.Average();
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

    return Spectrum::black;
}

} // namespace bulbit
