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

Spectrum LightVolPathIntegrator::Tr(const Point3 p1, const Point3 p2, const Medium* medium, int32 wavelength) const
{
    Vec3 w = p2 - p1;
    Float visibility = w.Normalize();

    Ray ray(p1, w);

    Spectrum Tr(1);
    Spectrum r_pdf(1);

    RNG rng(Hash(p1), Hash(p2));

    while (visibility > 0)
    {
        Intersection isect;
        bool found_intersection = Intersect(&isect, ray, Ray::epsilon, visibility);

        // Intersects opaque boundary
        if (found_intersection && isect.primitive->GetMaterial())
        {
            return Spectrum::black;
        }

        if (medium)
        {
            Float t_max = found_intersection ? isect.t : visibility;

            // Estimate transmittance with ratio tracking
            Spectrum T_maj = Sample_MajorantTransmittance(
                medium, wavelength, ray, t_max, rng.NextFloat(), rng,
                [&](Point3 p, MediumSample ms, Spectrum sigma_maj, Spectrum T_maj) {
                    BulbitNotUsed(p);

                    Spectrum sigma_n = sigma_maj - ms.sigma_a - ms.sigma_s;
                    Float pdf = sigma_maj[wavelength] * T_maj[wavelength];

                    Tr *= sigma_n * T_maj / pdf;
                    r_pdf *= sigma_maj * T_maj / pdf;

                    return !Tr.IsBlack() && !r_pdf.IsBlack();
                }
            );

            Float pdf = T_maj[wavelength];
            Tr *= T_maj / pdf;
            r_pdf *= T_maj / pdf;
        }

        if (Tr.IsBlack())
        {
            return Spectrum::black;
        }

        if (!found_intersection)
        {
            break;
        }

        ray.o = isect.point;
        visibility -= isect.t;
        medium = isect.GetMedium(ray.d);
    }

    return Tr / r_pdf.Average();
}

Spectrum LightVolPathIntegrator::L(
    const Ray& primary_ray, const Medium* primary_medium, const Camera* camera, Film& film, Sampler& sampler
) const
{
    BulbitNotUsed(primary_ray);
    BulbitNotUsed(primary_medium);

    int32 wavelength = std::min<int32>(int32(sampler.Next1D() * 3), 2);
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
    if (!camera->SampleWi(&camera_sample, isect, sampler.Next2D()))
    {
        return Spectrum::black;
    }

    // Add bounce 0 light contribution to film
    if (Spectrum V = Tr(light_sample.ray.o, camera_sample.p_aperture, medium, wavelength); !V.IsBlack())
    {
        Spectrum L = sampled_light.weight * light_sample.Le * AbsDot(light_sample.normal, camera_sample.wi) *
                     AbsDot(camera_sample.normal, camera_sample.wi) * camera_sample.Wi / (camera_sample.pdf * light_sample.pdf_p);

        film.AddSplat(camera_sample.p_raster, L);
    }

    int32 bounce = 0;
    Ray ray = light_sample.ray;

    Spectrum beta =
        light_sample.Le * AbsDot(light_sample.normal, ray.d) * sampled_light.weight / (light_sample.pdf_p * light_sample.pdf_w);

    // Trace light path
    while (true)
    {
        if (!Intersect(&isect, ray, Ray::epsilon, infinity))
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
            medium = isect.GetMedium(ray.d);
            ray.o = isect.point;
            --bounce;
            continue;
        }

        if (bounce++ >= max_bounces)
        {
            break;
        }

        if (camera->SampleWi(&camera_sample, isect, sampler.Next2D()))
        {
            Vec3 wi = camera_sample.wi;

            if (Spectrum V = Tr(isect.point, camera_sample.p_aperture, medium, wavelength); !V.IsBlack())
            {
                Spectrum L = beta * camera_sample.Wi * bsdf.f(wo, wi, TransportDirection::ToCamera) *
                             AbsDot(isect.shading.normal, wi) / camera_sample.pdf;

                film.AddSplat(camera_sample.p_raster, L);
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
        medium = isect.GetMedium(bsdf_sample.wi);

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
