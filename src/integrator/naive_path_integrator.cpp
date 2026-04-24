#include "bulbit/bsdf.h"
#include "bulbit/bxdfs.h"
#include "bulbit/integrators.h"
#include "bulbit/material.h"
#include "bulbit/sampler.h"

namespace bulbit
{

NaivePathIntegrator::NaivePathIntegrator(
    const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler, int32 max_bounces, int32 rr_min_bounces
)
    : UniDirectionalRayIntegrator(accel, std::move(lights), sampler, nullptr)
    , max_bounces{ max_bounces }
    , rr_min_bounces{ rr_min_bounces }
{
}

SpectrumSample NaivePathIntegrator::Li(
    const Ray& primary_ray, const Medium* primary_medium, WavelengthSample& lambda, Sampler& sampler
) const
{
    BulbitNotUsed(primary_medium);

    int32 bounce = 0;
    SpectrumSample L(0), beta(1);
    Float eta_scale = 1;
    Ray ray = primary_ray;

    while (true)
    {
        Intersection isect;
        if (!Intersect(&isect, ray, Ray::epsilon, infinity))
        {
            for (Light* light : infinite_lights)
            {
                L += beta * light->Le(ray, lambda);
            }

            break;
        }

        Vec3 wo = Normalize(-ray.d);

        if (const Light* area_light = GetAreaLight(isect); area_light)
        {
            L += beta * area_light->Le(isect, wo, lambda);
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
            ray = Ray(isect.point, -wo);
            --bounce;
            continue;
        }

        BSDFSample bsdf_sample;
        if (!bsdf.Sample_f(&bsdf_sample, wo, sampler.Next1D(), sampler.Next2D()))
        {
            break;
        }

        if (bsdf_sample.IsTransmission())
        {
            eta_scale *= Sqr(bsdf_sample.eta);
        }

        beta *= bsdf_sample.f * AbsDot(isect.shading.normal, bsdf_sample.wi) / bsdf_sample.pdf;
        ray = Ray(isect.point, bsdf_sample.wi);

        if (bounce > rr_min_bounces)
        {
            if (Float p = beta.MaxComponent() * eta_scale; p < 1)
            {
                if (sampler.Next1D() > p)
                {
                    break;
                }
                beta /= p;
            }
        }
    }

    return L;
}

SpectrumSample NaivePathIntegrator::Li(const Ray& ray, WavelengthSample& lambda, Sampler& sampler, int32 depth) const
{
    SpectrumSample L(0);

    if (depth > max_bounces)
    {
        return L;
    }

    Intersection isect;
    if (!Intersect(&isect, ray, Ray::epsilon, infinity))
    {
        for (auto& light : infinite_lights)
        {
            L += light->Le(ray, lambda);
        }

        return L;
    }

    Vec3 wo = Normalize(-ray.d);

    if (const Light* area_light = GetAreaLight(isect); area_light)
    {
        L += area_light->Le(isect, wo, lambda);
    }

    int8 mem[max_bxdf_size];
    BufferResource res(mem, sizeof(mem));
    Allocator alloc(&res);
    BSDF bsdf;
    if (!isect.GetBSDF(&bsdf, wo, lambda, alloc))
    {
        return L;
    }

    BSDFSample bsdf_sample;
    if (!bsdf.Sample_f(&bsdf_sample, wo, sampler.Next1D(), sampler.Next2D()))
    {
        return L;
    }

    SpectrumSample f_cos = bsdf_sample.f * Dot(isect.normal, bsdf_sample.wi);
    return L + Li(Ray(isect.point, bsdf_sample.wi), lambda, sampler, depth + 1) * f_cos / bsdf_sample.pdf;
}

} // namespace bulbit
