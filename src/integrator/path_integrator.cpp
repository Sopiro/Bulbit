#include "bulbit/bsdf.h"
#include "bulbit/bxdfs.h"
#include "bulbit/integrators.h"
#include "bulbit/lights.h"
#include "bulbit/material.h"
#include "bulbit/sampler.h"

namespace bulbit
{

PathIntegrator::PathIntegrator(
    const Intersectable* accel,
    std::vector<Light*> lights,
    const Sampler* sampler,
    int32 max_bounces,
    int32 rr_min_bounces,
    bool regularize_bsdf
)
    : UniDirectionalRayIntegrator(accel, std::move(lights), sampler, std::make_unique<PowerLightSampler>())
    , max_bounces{ max_bounces }
    , rr_min_bounces{ rr_min_bounces }
    , regularize_bsdf{ regularize_bsdf }
{
}

SpectrumSample PathIntegrator::Li(
    const Ray& primary_ray, const Medium* primary_medium, WavelengthSample& lambda, Sampler& sampler
) const
{
    BulbitNotUsed(primary_medium);

    int32 bounce = 0;
    SpectrumSample L(0), beta(1);
    bool specular_bounce = false;
    bool any_non_specular_bounces = false;
    Float eta_scale = 1;
    Ray ray = primary_ray;
    Float prev_bsdf_pdf = 0;

    while (true)
    {
        Intersection isect;
        if (!Intersect(&isect, ray, Ray::epsilon, infinity))
        {
            if (bounce == 0 || specular_bounce)
            {
                for (Light* light : infinite_lights)
                {
                    L += beta * light->Le(ray, lambda);
                }
            }
            else
            {
                for (Light* light : infinite_lights)
                {
                    Float light_pdf = light->EvaluatePDF_Li(ray) * light_sampler->EvaluatePMF(light);
                    Float mis_weight = PowerHeuristic(1, prev_bsdf_pdf, 1, light_pdf);

                    L += beta * mis_weight * light->Le(ray, lambda);
                }
            }

            break;
        }

        Vec3 wo = Normalize(-ray.d);

        if (const Light* area_light = GetAreaLight(isect); area_light)
        {
            SpectrumSample Le = area_light->Le(isect, wo, lambda);
            if (!Le.IsBlack())
            {
                if (bounce == 0 || specular_bounce)
                {
                    L += beta * Le;
                }
                else
                {
                    Float light_pdf = isect.primitive->GetShape()->PDF(isect, ray) * light_sampler->EvaluatePMF(area_light);
                    Float mis_weight = PowerHeuristic(1, prev_bsdf_pdf, 1, light_pdf);

                    L += beta * mis_weight * Le;
                }
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
        if (!isect.GetBSDF(&bsdf, wo, lambda, alloc))
        {
            ray = Ray(isect.point, -wo);
            --bounce;
            continue;
        }

        if (regularize_bsdf && any_non_specular_bounces)
        {
            bsdf.Regularize();
        }

        if (IsNonSpecular(bsdf.Flags()))
        {
            L += SampleDirectLight(wo, isect, &bsdf, lambda, sampler, beta);
        }

        BSDFSample bsdf_sample;
        if (!bsdf.Sample_f(&bsdf_sample, wo, sampler.Next1D(), sampler.Next2D()))
        {
            break;
        }

        specular_bounce = bsdf_sample.IsSpecular();
        any_non_specular_bounces |= !bsdf_sample.IsSpecular();
        if (bsdf_sample.IsTransmission())
        {
            eta_scale *= Sqr(bsdf_sample.eta);
        }

        prev_bsdf_pdf = bsdf_sample.is_stochastic ? bsdf.PDF(wo, bsdf_sample.wi) : bsdf_sample.pdf;
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
                else
                {
                    beta /= p;
                }
            }
        }
    }

    return L;
}

SpectrumSample PathIntegrator::SampleDirectLight(
    const Vec3& wo,
    const Intersection& isect,
    BSDF* bsdf,
    const WavelengthSample& lambda,
    Sampler& sampler,
    const SpectrumSample& beta
) const
{
    Float u0 = sampler.Next1D();
    Point2 u12 = sampler.Next2D();
    SampledLight sampled_light;
    if (!light_sampler->Sample(&sampled_light, isect, u0))
    {
        return SpectrumSample(0);
    }

    LightSampleLi light_sample;
    if (!sampled_light.light->Sample_Li(&light_sample, isect, u12, lambda))
    {
        return SpectrumSample(0);
    }

    Float bsdf_pdf = bsdf->PDF(wo, light_sample.wi);
    if (light_sample.Li.IsBlack() || bsdf_pdf == 0)
    {
        return SpectrumSample(0);
    }

    Ray shadow_ray(isect.point, light_sample.wi);
    if (IntersectAny(shadow_ray, Ray::epsilon, light_sample.visibility))
    {
        return SpectrumSample(0);
    }

    Float light_pdf = sampled_light.pmf * light_sample.pdf;
    SpectrumSample f_cos = bsdf->f(wo, light_sample.wi) * AbsDot(isect.shading.normal, light_sample.wi);

    if (sampled_light.light->IsDeltaLight())
    {
        return beta * light_sample.Li * f_cos / light_pdf;
    }
    else
    {
        Float mis_weight = PowerHeuristic(1, light_pdf, 1, bsdf_pdf);
        return beta * mis_weight * light_sample.Li * f_cos / light_pdf;
    }
}

} // namespace bulbit
