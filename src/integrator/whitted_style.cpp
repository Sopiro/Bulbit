#include "bulbit/bsdf.h"
#include "bulbit/bxdfs.h"
#include "bulbit/integrators.h"
#include "bulbit/material.h"
#include "bulbit/sampler.h"

namespace bulbit
{

WhittedStyle::WhittedStyle(const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler, int32 max_depth)
    : UniDirectionalRayIntegrator(accel, std::move(lights), sampler, nullptr)
    , max_depth{ max_depth }
{
}

Spectrum WhittedStyle::Li(const Ray& ray, Sampler& sampler, int32 depth) const
{
    Spectrum L(0);

    if (depth > max_depth)
    {
        return L;
    }

    Intersection isect;
    if (!Intersect(&isect, ray, Ray::epsilon, infinity))
    {
        for (auto& light : infinite_lights)
        {
            L += light->Le(ray);
        }

        return L;
    }

    Vec3 wo = Normalize(-ray.d);

    // Evaluate emitted light
    L += isect.Le(wo);

    int8 mem[max_bxdf_size];
    BufferResource res(mem, sizeof(mem));
    Allocator alloc(&res);
    BSDF bsdf;
    if (isect.GetBSDF(&bsdf, wo, alloc) == false)
    {
        return L;
    }

    // Evaluate direct light
    for (const Light* light : all_lights)
    {
        LightSampleLi light_sample;
        if (!light->Sample_Li(&light_sample, isect, sampler.Next2D()))
        {
            continue;
        }

        if (light_sample.Li.IsBlack() == false && light_sample.pdf > 0)
        {
            Ray shadow_ray{ isect.point, light_sample.wi };
            if (IntersectAny(shadow_ray, Ray::epsilon, light_sample.visibility) == false)
            {
                Spectrum f_cos = bsdf.f(wo, light_sample.wi);
                L += light_sample.Li * f_cos / light_sample.pdf;
            }
        }
    }

    // Evaluate indirect light only for the perfect reflection direction
    Vec3 wi = Reflect(wo, isect.normal);

    if (Dot(isect.normal, wi) > 0)
    {
        Spectrum f_cos = bsdf.f(wo, wi) * Dot(isect.shading.normal, wi);
        return L + Li(Ray(isect.point, wi), sampler, depth + 1) * f_cos;
    }
    else
    {
        return L;
    }
}

} // namespace bulbit
