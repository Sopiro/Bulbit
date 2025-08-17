#include "bulbit/async_job.h"
#include "bulbit/bsdf.h"
#include "bulbit/bxdfs.h"
#include "bulbit/film.h"
#include "bulbit/integrators.h"
#include "bulbit/microfacet.h"
#include "bulbit/parallel_for.h"
#include "bulbit/progress.h"
#include "bulbit/sampler.h"
#include "bulbit/visibility.h"

namespace bulbit
{

PhotonMappingIntegrator::PhotonMappingIntegrator(
    const Intersectable* accel,
    std::vector<Light*> lights,
    const Sampler* sampler,
    int32 max_bounces,
    int32 n_photons,
    Float gather_radius
)
    : Integrator(accel, std::move(lights))
    , sampler_prototype{ sampler }
    , max_bounces{ max_bounces }
    , n_photons{ n_photons }
    , gather_radius{ gather_radius }
{
}

void PhotonMappingIntegrator::EmitPhotons()
{
    RNG rng(Hash(n_photons), Hash(gather_radius));

    for (int32 i = 0; i < n_photons; ++i)
    {
        SampledLight sampled_light;
        if (!light_sampler.Sample(&sampled_light, Intersection{}, rng.NextFloat()))
        {
            continue;
        }

        Point2 u0(rng.NextFloat(), rng.NextFloat());
        Point2 u1(rng.NextFloat(), rng.NextFloat());

        LightSampleLe light_sample;
        if (!sampled_light.light->Sample_Le(&light_sample, u0, u1))
        {
            continue;
        }

        Ray photon_ray = light_sample.ray;
        Spectrum beta = light_sample.Le / (sampled_light.pmf * light_sample.pdf_p * light_sample.pdf_w);

        int32 bounce = 0;
        while (true)
        {
            Vec3 wo = Normalize(-photon_ray.d);

            Intersection isect;
            if (!Intersect(&isect, photon_ray, Ray::epsilon, infinity))
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
                photon_ray.o = isect.point;
                --bounce;
                continue;
            }

            // Store photon to non specular surface
            if (bounce > 1 && IsNonSpecular(bsdf.Flags()))
            {
                Photon p;
                p.position = isect.point;
                p.wi = wo;
                p.beta = beta;
                photon_map.Store(p);
            }

            BSDFSample bsdf_sample;
            if (!bsdf.Sample_f(
                    &bsdf_sample, wo, rng.NextFloat(), { rng.NextFloat(), rng.NextFloat() }, TransportDirection::ToCamera
                ))
            {
                break;
            }

            photon_ray = Ray(isect.point, bsdf_sample.wi);
            beta *= bsdf_sample.f * AbsDot(isect.shading.normal, bsdf_sample.wi) / bsdf_sample.pdf;

            constexpr int32 min_bounces = 2;
            if (bounce > min_bounces)
            {
                if (Float p = beta.MaxComponent(); p < 1)
                {
                    if (rng.NextFloat() > p)
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
    }

    photon_map.Build(gather_radius);
}

Spectrum PhotonMappingIntegrator::SampleDirectLight(
    const Vec3& wo, const Intersection& isect, const BSDF* bsdf, Sampler& sampler, const Spectrum& beta
) const
{
    SampledLight sampled_light;
    if (!light_sampler.Sample(&sampled_light, isect, sampler.Next1D()))
    {
        return Spectrum::black;
    }

    LightSampleLi light_sample;
    if (!sampled_light.light->Sample_Li(&light_sample, isect, sampler.Next2D()))
    {
        return Spectrum::black;
    }

    if (light_sample.Li.IsBlack())
    {
        return Spectrum::black;
    }

    if (!V(this, isect.point, light_sample.point))
    {
        return Spectrum::black;
    }

    Float pdf = light_sample.pdf * sampled_light.pmf;
    return beta * light_sample.Li * bsdf->f(wo, light_sample.wi) * AbsDot(isect.shading.normal, light_sample.wi) / pdf;
}

Spectrum PhotonMappingIntegrator::Li(const Ray& primary_ray, const Medium* primary_medium, Sampler& sampler) const
{
    BulbitNotUsed(primary_medium);

    int32 bounce = 0;

    Spectrum L(0);
    Spectrum beta(1);

    Ray ray = primary_ray;
    bool was_specular_bounce = false;

    while (true)
    {
        Intersection isect;
        if (!Intersect(&isect, ray, Ray::epsilon, infinity))
        {
            if (bounce == 0 || was_specular_bounce)
            {
                for (Light* light : infinite_lights)
                {
                    L += beta * light->Le(ray);
                }
            }

            break;
        }

        Vec3 wo = Normalize(-ray.d);

        if (Spectrum Le = isect.Le(wo); !Le.IsBlack())
        {
            if (bounce == 0 || was_specular_bounce)
            {
                L += beta * isect.Le(-ray.d);
            }
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
            ray.o = isect.point;
            --bounce;
            continue;
        }

        if (IsNonSpecular(bsdf.Flags()))
        {
            // Estimate direct light
            L += SampleDirectLight(wo, isect, &bsdf, sampler, beta);

            // Estimate indirect light by gathering nearby photons
            Spectrum L_p(0);

            photon_map.Query(isect.point, gather_radius, [&](const Photon& p) {
                L_p += bsdf.f(-ray.d, p.wi) * AbsDot(isect.shading.normal, p.wi) * p.beta;
            });

            L_p *= 1 / (pi * Sqr(gather_radius) * n_photons);
            L += beta * L_p;

            // Done!
            break;
        }

        BSDFSample bsdf_sample;
        if (!bsdf.Sample_f(&bsdf_sample, -ray.d, sampler.Next1D(), sampler.Next2D()))
        {
            break;
        }

        was_specular_bounce = true;
        ray = Ray(isect.point, bsdf_sample.wi);
        beta *= bsdf_sample.f * AbsDot(isect.shading.normal, bsdf_sample.wi) / bsdf_sample.pdf;
    }

    return L;
}

std::unique_ptr<Rendering> PhotonMappingIntegrator::Render(const Camera* camera)
{
    ComoputeReflectanceTextures();

    Point2i resolution = camera->GetScreenResolution();
    const int32 spp = sampler_prototype->samples_per_pixel;
    const int32 tile_size = 16;
    std::unique_ptr<Rendering> progress = std::make_unique<Rendering>(camera, tile_size);

    EmitPhotons();

    progress->job = RunAsync([=, this, &progress]() {
        ParallelFor2D(
            resolution,
            [&](AABB2i tile) {
                std::unique_ptr<Sampler> sampler = sampler_prototype->Clone();

                for (Point2i pixel : tile)
                {
                    for (int32 sample = 0; sample < spp; ++sample)
                    {
                        sampler->StartPixelSample(pixel, sample);

                        PrimaryRay primary_ray;
                        camera->SampleRay(&primary_ray, pixel, sampler->Next2D(), sampler->Next2D());

                        Spectrum L = Li(primary_ray.ray, camera->GetMedium(), *sampler);
                        if (!L.IsNullish())
                        {
                            progress->film.AddSample(pixel, primary_ray.weight * L);
                        }
                    }
                }

                progress->tile_done++;
            },
            tile_size
        );

        progress->done = true;
        return true;
    });

    return progress;
}

} // namespace bulbit
