#include "bulbit/bxdfs.h"
#include "bulbit/integrators.h"
#include "bulbit/lights.h"
#include "bulbit/material.h"

namespace bulbit
{

LightPathIntegrator::LightPathIntegrator(
    const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler, int32 max_bounces
)
    : BiDirectionalRayIntegrator(accel, std::move(lights), sampler)
    , light_sampler{ all_lights }
    , max_bounces{ max_bounces }
{
}

Spectrum LightPathIntegrator::L(
    Film& film, const Camera& camera, const Ray& primary_ray, const Medium* primary_medium, Sampler& sampler
) const
{
    BulbitNotUsed(primary_ray);
    BulbitNotUsed(primary_medium);

    Intersection isect;

    // Sample light to start light tracing
    SampledLight sampled_light;
    if (!light_sampler.Sample(&sampled_light, isect, sampler.Next1D()))
    {
        return Spectrum::black;
    }

    // Sample point and direction from sampled light
    LightSampleLe light_sample = sampled_light.light->Sample_Le(sampler.Next2D(), sampler.Next2D());

    isect.point = light_sample.ray.o;

    CameraSampleWi camera_sample;
    if (!camera.SampleWi(&camera_sample, isect, sampler.Next2D()))
    {
        return Spectrum::black;
    }

    // Add bounce 0 light contribution to film
    if (V(light_sample.ray.o, camera_sample.p_aperture))
    {
        Float pdf = camera_sample.pdf * light_sample.pdf_p;
        if (pdf != 0)
        {
            Spectrum L = sampled_light.weight * light_sample.Le * AbsDot(light_sample.normal, camera_sample.wi) *
                         AbsDot(camera_sample.normal, camera_sample.wi) * camera_sample.Wi / pdf;

            film.AddSplat(camera.GetFilter(), camera_sample.p_raster, L);
        }
    }

    int32 bounce = 0;
    Ray ray = light_sample.ray;

    Float pdf_light = light_sample.pdf_p * light_sample.pdf_w / sampled_light.weight;
    if (pdf_light == 0)
    {
        return Spectrum::black;
    }

    Spectrum beta = light_sample.Le * AbsDot(light_sample.normal, ray.d) / pdf_light;

    // Trace light path
    while (true)
    {
        bool found_intersection = Intersect(&isect, ray, Ray::epsilon, infinity);
        if (!found_intersection)
        {
            break;
        }

        Vec3 wo = Normalize(-ray.d);

        int8 mem[max_bxdf_size];
        Resource res(mem, sizeof(mem));
        Allocator alloc(&res);
        BSDF bsdf;
        if (!isect.GetBSDF(&bsdf, wo, alloc))
        {
            ray = Ray(isect.point, -wo);
            --bounce;
            continue;
        }

        if (bounce++ >= max_bounces)
        {
            break;
        }

        if (camera.SampleWi(&camera_sample, isect, sampler.Next2D()))
        {
            Vec3 wi = camera_sample.wi;

            if (V(isect.point, camera_sample.p_aperture))
            {
                Spectrum L = beta * camera_sample.Wi * bsdf.f(wo, wi, TransportDirection::ToCamera) *
                             AbsDot(isect.shading.normal, wi) / camera_sample.pdf;

                film.AddSplat(camera.GetFilter(), camera_sample.p_raster, L);
            }
        }

        // Sample bsdf to find next vertex
        BSDFSample bsdf_sample;
        if (!bsdf.Sample_f(&bsdf_sample, wo, sampler.Next1D(), sampler.Next2D(), TransportDirection::ToCamera))
        {
            break;
        }

        // Update path state
        beta *= bsdf_sample.f * AbsDot(bsdf_sample.wi, isect.shading.normal) / bsdf_sample.pdf;
        ray = Ray(isect.point, bsdf_sample.wi);

        // Terminate path with russian roulette
        constexpr int32 min_bounces = 2;
        if (bounce > min_bounces)
        {
            if (Float p = beta.MaxComponent(); p < 1)
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
