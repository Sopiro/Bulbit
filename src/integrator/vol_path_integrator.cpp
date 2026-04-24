#include "bulbit/bssrdfs.h"
#include "bulbit/bxdfs.h"
#include "bulbit/integrators.h"
#include "bulbit/material.h"
#include "bulbit/media.h"
#include "bulbit/random.h"
#include "bulbit/sampler.h"

namespace bulbit
{

VolPathIntegrator::VolPathIntegrator(
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

SpectrumSample VolPathIntegrator::Li(
    const Ray& primary_ray, const Medium* primary_medium, WavelengthSample& lambda, Sampler& sampler
) const
{
    constexpr int32 hero = WavelengthSample::hero_lane;
    int32 bounce = 0;
    SpectrumSample L(0), beta(1);

    SpectrumSample r_u(1);
    SpectrumSample r_l(1);

    bool specular_bounce = false;
    bool any_non_specular_bounces = false;

    Float eta_scale = 1;
    Ray ray = primary_ray;

    const Medium* medium = primary_medium;

    while (true)
    {
        Vec3 wo = Normalize(-ray.d);
        Intersection isect;
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

                    if (bounce < max_bounces && !Le.IsBlack())
                    {
                        Float pdf = sigma_maj[hero] * T_maj[hero];
                        SpectrumSample beta_e = beta * T_maj / pdf;
                        SpectrumSample r_e = r_u * sigma_maj * T_maj / pdf;

                        if (!r_e.IsBlack())
                        {
                            L += beta_e * sigma_a * Le / r_e.Average();
                        }
                    }

                    Float p_absorb = sigma_a[hero] / sigma_maj[hero];
                    Float p_scatter = sigma_s[hero] / sigma_maj[hero];
                    Float p_null = std::max<Float>(0, 1 - p_absorb - p_scatter);
                    Float events[3] = { p_absorb, p_scatter, p_null };

                    int32 event = SampleDiscrete(events, rng.NextFloat());
                    switch (event)
                    {
                    case 0:
                        terminated = true;
                        return false;

                    case 1:
                    {
                        if (bounce++ >= max_bounces)
                        {
                            terminated = true;
                            return false;
                        }

                        Float pdf = T_maj[hero] * sigma_s[hero];
                        beta *= T_maj * sigma_s / pdf;
                        r_u *= T_maj * sigma_s / pdf;

                        Intersection medium_isect{ .point = p };
                        L += SampleDirectLight(wo, medium_isect, medium, nullptr, ms.phase, lambda, sampler, beta, r_u);

                        PhaseFunctionSample phase_sample;
                        if (!ms.phase->Sample_p(&phase_sample, wo, sampler.Next2D()))
                        {
                            terminated = true;
                            return false;
                        }

                        beta *= phase_sample.p / phase_sample.pdf;
                        r_l = r_u / phase_sample.pdf;

                        ray.o = p;
                        ray.d = phase_sample.wi;

                        specular_bounce = false;
                        any_non_specular_bounces = true;

                        scattered = true;
                        return false;
                    }

                    case 2:
                    {
                        SpectrumSample sigma_n = Max(sigma_maj - sigma_a - sigma_s, 0);
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
                        r_l *= T_maj * sigma_maj / pdf;

                        return !beta.IsBlack() && !r_u.IsBlack();
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
                continue;
            }

            beta *= T_maj / T_maj[hero];
            r_u *= T_maj / T_maj[hero];
            r_l *= T_maj / T_maj[hero];
        }

        if (!found_intersection)
        {
            if (bounce == 0 || specular_bounce)
            {
                for (Light* light : infinite_lights)
                {
                    L += beta * light->Le(ray, lambda) / r_u.Average();
                }
            }
            else
            {
                for (Light* light : infinite_lights)
                {
                    Float light_pdf = light->EvaluatePDF_Li(ray) * light_sampler->EvaluatePMF(light);
                    L += beta * light->Le(ray, lambda) / (r_u + r_l * light_pdf).Average();
                }
            }

            break;
        }

        if (const Light* area_light = GetAreaLight(isect); area_light)
        {
            SpectrumSample Le = area_light->Le(isect, wo, lambda);
            if (!Le.IsBlack())
            {
                if (bounce == 0 || specular_bounce)
                {
                    L += beta * Le / r_u.Average();
                }
                else
                {
                    Float light_pdf = isect.primitive->GetShape()->PDF(isect, ray) * light_sampler->EvaluatePMF(area_light);
                    L += beta * Le / (r_u + r_l * light_pdf).Average();
                }
            }
        }

        if (bounce++ >= max_bounces)
        {
            break;
        }

        int8 mem[std::max(max_bxdf_size, max_bssrdf_size)];
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

        if (regularize_bsdf && any_non_specular_bounces)
        {
            bsdf.Regularize();
        }

        if (IsNonSpecular(bsdf.Flags()))
        {
            L += SampleDirectLight(wo, isect, nullptr, &bsdf, nullptr, lambda, sampler, beta, r_u);
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

        beta *= bsdf_sample.f * AbsDot(isect.shading.normal, bsdf_sample.wi) / bsdf_sample.pdf;
        if (bsdf_sample.is_stochastic)
        {
            r_l = r_u / bsdf.PDF(wo, bsdf_sample.wi);
        }
        else
        {
            r_l = r_u / bsdf_sample.pdf;
        }

        ray = Ray(isect.point, bsdf_sample.wi);
        medium = isect.GetMedium(bsdf_sample.wi);

        BSSRDF* bssrdf;
        if (isect.GetBSSRDF(&bssrdf, wo, lambda, alloc) && bsdf_sample.IsTransmission())
        {
            Float u0 = sampler.Next1D();
            Point2 u12 = sampler.Next2D();

            BSSRDFSample bssrdf_sample;
            if (!bssrdf->Sample_S(&bssrdf_sample, bsdf_sample, accel, lambda, u0, u12))
            {
                break;
            }

            SpectrumSample bssrdf_pdf = bssrdf_sample.pdf;
            Float pdf = bssrdf_pdf[hero] * bssrdf_sample.p;
            beta *= bssrdf_sample.Sp / pdf;
            r_u *= bssrdf_pdf / bssrdf_pdf[hero];

            any_non_specular_bounces = true;
            BSDF& Sw = bssrdf_sample.Sw;
            if (regularize_bsdf)
            {
                Sw.Regularize();
            }

            L += SampleDirectLight(bssrdf_sample.wo, bssrdf_sample.pi, nullptr, &Sw, nullptr, lambda, sampler, beta, r_u);

            if (!Sw.Sample_f(&bsdf_sample, bssrdf_sample.wo, sampler.Next1D(), sampler.Next2D()))
            {
                break;
            }

            beta *= bsdf_sample.f * AbsDot(bsdf_sample.wi, bssrdf_sample.pi.shading.normal) / bsdf_sample.pdf;
            r_l = r_u / bsdf_sample.pdf;

            specular_bounce = bsdf_sample.IsSpecular();

            ray = Ray(bssrdf_sample.pi.point, bsdf_sample.wi);
            medium = bssrdf_sample.pi.GetMedium(bsdf_sample.wi);
        }

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

    return L;
}

SpectrumSample VolPathIntegrator::SampleDirectLight(
    const Vec3& wo,
    const Intersection& isect,
    const Medium* medium,
    const BSDF* bsdf,
    const PhaseFunction* phase,
    const WavelengthSample& lambda,
    Sampler& sampler,
    SpectrumSample beta,
    SpectrumSample r_p
) const
{
    constexpr int32 hero = WavelengthSample::hero_lane;

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

    if (light_sample.Li.IsBlack() || light_sample.pdf == 0)
    {
        return SpectrumSample(0);
    }

    Float light_pdf = sampled_light.pmf * light_sample.pdf;

    SpectrumSample scattering_f;
    Float scattering_pdf;
    Vec3 wi = light_sample.wi;

    if (bsdf)
    {
        medium = isect.GetMedium(wi);
        scattering_f = bsdf->f(wo, wi) * AbsDot(isect.shading.normal, wi);
        scattering_pdf = bsdf->PDF(wo, wi);
    }
    else
    {
        BulbitAssert(medium != nullptr);
        BulbitAssert(phase != nullptr);
        scattering_f = SpectrumSample(phase->p(wo, wi));
        scattering_pdf = phase->PDF(wo, wi);
    }

    if (scattering_f.IsBlack())
    {
        return SpectrumSample(0);
    }

    Ray light_ray(isect.point, wi);
    Float visibility = light_sample.visibility;

    SpectrumSample T_ray(1);
    SpectrumSample r_u(1);
    SpectrumSample r_l(1);

    RNG rng(Hash(light_ray.o), Hash(light_ray.d));

    while (visibility > 0)
    {
        Intersection light_isect;
        bool found_intersection = Intersect(&light_isect, light_ray, Ray::epsilon, visibility);

        if (found_intersection && light_isect.primitive->GetMaterial())
        {
            return SpectrumSample(0);
        }

        if (medium)
        {
            Float t_max = found_intersection ? light_isect.t : visibility;
            Float u = rng.NextFloat();

            SpectrumSample T_maj = Sample_MajorantTransmittance(
                medium, lambda, light_ray, t_max, u, rng,
                [&](Point3 p, MediumSample ms, SpectrumSample sigma_maj, SpectrumSample T_maj) -> bool {
                    BulbitNotUsed(p);

                    SpectrumSample sigma_a = ms.sigma_a;
                    SpectrumSample sigma_s = ms.sigma_s;
                    SpectrumSample sigma_n = Max(sigma_maj - sigma_a - sigma_s, 0);
                    Float pdf = T_maj[hero] * sigma_maj[hero];
                    T_ray *= T_maj * sigma_n / pdf;
                    r_l *= T_maj * sigma_maj / pdf;
                    r_u *= T_maj * sigma_n / pdf;

                    SpectrumSample Tr = T_ray / (r_u + r_l).Average();
                    if (Tr.MaxComponent() < 0.05f)
                    {
                        constexpr Float rr = 0.75f;
                        if (rng.NextFloat() < rr)
                        {
                            T_ray = SpectrumSample(0);
                        }
                        else
                        {
                            T_ray /= 1 - rr;
                        }
                    }

                    return !T_ray.IsBlack();
                }
            );

            T_ray *= T_maj / T_maj[hero];
            r_l *= T_maj / T_maj[hero];
            r_u *= T_maj / T_maj[hero];
        }

        if (T_ray.IsBlack())
        {
            return SpectrumSample(0);
        }

        if (!found_intersection)
        {
            break;
        }

        light_ray.o = light_isect.point;
        visibility -= light_isect.t;
        medium = light_isect.GetMedium(light_ray.d);
    }

    r_l *= r_p * light_pdf;
    r_u *= r_p * scattering_pdf;

    if (sampled_light.light->IsDeltaLight())
    {
        return beta * scattering_f * T_ray * light_sample.Li / r_l.Average();
    }
    else
    {
        return beta * scattering_f * T_ray * light_sample.Li / (r_u + r_l).Average();
    }
}

} // namespace bulbit
