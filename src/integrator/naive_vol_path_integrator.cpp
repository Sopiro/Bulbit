#include "bulbit/bssrdfs.h"
#include "bulbit/bxdfs.h"
#include "bulbit/integrators.h"
#include "bulbit/material.h"
#include "bulbit/media.h"
#include "bulbit/random.h"
#include "bulbit/sampler.h"

namespace bulbit
{

NaiveVolPathIntegrator::NaiveVolPathIntegrator(
    const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler, int32 max_bounces, int32 rr_min_bounces
)
    : UniDirectionalRayIntegrator(accel, std::move(lights), sampler, nullptr)
    , max_bounces{ max_bounces }
    , rr_min_bounces{ rr_min_bounces }
{
}

SpectrumSample NaiveVolPathIntegrator::Li(
    const Ray& primary_ray, const Medium* primary_medium, WavelengthSample& lambda, Sampler& sampler
) const
{
    int32 bounce = 0;
    const int32 hero = WavelengthSample::hero_lane;

    SpectrumSample L(0), beta(1);
    SpectrumSample r_u(1);
    Float eta_scale = 1;

    Ray ray = primary_ray;
    const Medium* medium = primary_medium;

    while (true)
    {
        bool scattered = false;
        bool terminated = false;

        Vec3 wo = Normalize(-ray.d);
        Intersection isect;
        bool found_intersection = Intersect(&isect, ray, Ray::epsilon, infinity);

        if (medium)
        {
            uint64 hash0 = Hash(sampler.Next1D());
            uint64 hash1 = Hash(sampler.Next1D());
            RNG rng(hash0, hash1);

            Float t_max = found_intersection ? isect.t : infinity;
            Float u = sampler.Next1D();
            Float u_event = sampler.Next1D();

            SpectrumSample T_maj = Sample_MajorantTransmittance(
                medium, lambda, ray, t_max, u, rng,
                [&](Point3 p, MediumSample ms, SpectrumSample sigma_maj, SpectrumSample T_maj) -> bool {
                    SpectrumSample sigma_a = ms.sigma_a;
                    SpectrumSample sigma_s = ms.sigma_s;
                    SpectrumSample Le = ms.Le;

                    Float p_absorb = sigma_a[hero] / sigma_maj[hero];
                    Float p_scatter = sigma_s[hero] / sigma_maj[hero];
                    Float p_null = std::max<Float>(0, 1 - p_absorb - p_scatter);
                    Float events[3] = { p_absorb, p_scatter, p_null };

                    int32 event = SampleDiscrete(events, u_event);
                    switch (event)
                    {
                    case 0:
                    {
                        Float pdf = T_maj[hero] * sigma_a[hero];
                        beta *= T_maj / pdf;
                        r_u *= T_maj * sigma_a / pdf;

                        L += beta * sigma_a * Le / r_u.Average();
                        terminated = true;
                        return false;
                    }

                    case 1:
                    {
                        if (bounce++ >= max_bounces)
                        {
                            terminated = true;
                            return false;
                        }

                        Point2 u{ rng.NextFloat(), rng.NextFloat() };
                        PhaseFunctionSample ps;
                        if (!ms.phase->Sample_p(&ps, wo, u))
                        {
                            terminated = true;
                            return false;
                        }

                        Float pdf = T_maj[hero] * sigma_s[hero];
                        beta *= T_maj * sigma_s / pdf;
                        r_u *= T_maj * sigma_s / pdf;

                        beta *= ps.p / ps.pdf;
                        ray.o = p;
                        ray.d = ps.wi;
                        scattered = true;
                        return false;
                    }

                    case 2:
                    {
                        u_event = rng.NextFloat();
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
                        return !beta.IsBlack() && !r_u.IsBlack();
                    }

                    default:
                        BulbitAssert(false);
                        return false;
                    }
                }
            );

            beta *= T_maj / T_maj[hero];
            r_u *= T_maj / T_maj[hero];

            if (terminated)
            {
                break;
            }
            if (scattered)
            {
                continue;
            }
        }

        if (!found_intersection)
        {
            for (Light* light : infinite_lights)
            {
                L += beta * light->Le(ray, lambda) / r_u.Average();
            }

            break;
        }

        if (const Light* area_light = GetAreaLight(isect); area_light)
        {
            SpectrumSample Le = area_light->Le(isect, wo, lambda);
            if (!Le.IsBlack())
            {
                L += beta * Le / r_u.Average();
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

            BSDF& Sw = bssrdf_sample.Sw;
            if (!Sw.Sample_f(&bsdf_sample, bssrdf_sample.wo, sampler.Next1D(), sampler.Next2D()))
            {
                break;
            }

            beta *= bsdf_sample.f * AbsDot(bsdf_sample.wi, bssrdf_sample.pi.shading.normal) / bsdf_sample.pdf;
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
                beta /= p;
            }
        }
    }

    return L;
}

} // namespace bulbit
