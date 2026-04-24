#include "bulbit/async_job.h"
#include "bulbit/bsdf.h"
#include "bulbit/bxdfs.h"
#include "bulbit/film.h"
#include "bulbit/integrators.h"
#include "bulbit/media.h"
#include "bulbit/microfacet.h"
#include "bulbit/parallel_for.h"
#include "bulbit/progresses.h"
#include "bulbit/sampler.h"
#include "bulbit/visibility.h"

namespace bulbit
{

namespace
{

WavelengthSample IterationLambda(int32 iteration, int32 total_iterations)
{
    Float u = Float(iteration + 0.5f) / Float(std::max(1, total_iterations));
    return WavelengthSample::Sample(std::fmod(u, 1.0f));
}

} // namespace

VolPhotonMappingIntegrator::VolPhotonMappingIntegrator(
    const Intersectable* accel,
    std::vector<Light*> lights,
    const Sampler* sampler,
    int32 max_bounces,
    int32 n_photons,
    Float gather_radius_surface,
    Float gather_radius_volume,
    bool sample_direct_light
)
    : Integrator(accel, std::move(lights), std::make_unique<PowerLightSampler>())
    , sampler_prototype{ sampler }
    , max_bounces{ max_bounces }
    , n_photons{ n_photons }
    , radius{ gather_radius_surface }
    , vol_radius{ gather_radius_volume }
    , sample_direct_light{ sample_direct_light }
{
    AABB world_bounds = accel->GetAABB();
    Point3 world_center;
    Float world_radius;
    world_bounds.ComputeBoundingSphere(&world_center, &world_radius);

    if (radius <= 0)
    {
        radius = 2 * world_radius * 5e-4f;
    }
    if (vol_radius <= 0)
    {
        vol_radius = 2 * world_radius * 1e-3f;
    }
}

void VolPhotonMappingIntegrator::EmitPhotons(MultiPhaseRendering* progress, WavelengthSample lambda, int32 phase_index)
{
    constexpr int32 hero = WavelengthSample::hero_lane;
    const int32 min_bounces = 2;

    ThreadLocal<std::vector<Photon>> tl_photons;
    ThreadLocal<std::vector<Photon>> tl_vol_photons;

    ParallelFor(0, n_photons, [&](int32 i) {
        RNG rng(Hash(n_photons, radius, vol_radius), Hash(i));

        auto Next1D = [&]() { return rng.NextFloat(); };
        auto Next2D = [&]() { return Point2(rng.NextFloat(), rng.NextFloat()); };

        std::vector<Photon>& surface_photons = tl_photons.Get();
        std::vector<Photon>& volume_photons = tl_vol_photons.Get();

        SampledLight sampled_light;
        if (!light_sampler->Sample(&sampled_light, Intersection{}, Next1D()))
        {
            return;
        }

        LightSampleLe light_sample;
        if (!sampled_light.light->Sample_Le(&light_sample, Next2D(), Next2D(), lambda))
        {
            return;
        }

        Ray ray = light_sample.ray;
        const Medium* medium = light_sample.medium;

        SpectrumSample beta = light_sample.Le / (sampled_light.pmf * light_sample.pdf_p * light_sample.pdf_w);
        if (light_sample.normal != Vec3::zero)
        {
            beta *= AbsDot(light_sample.normal, ray.d);
        }

        int32 bounce = 0;
        Float eta_scale = 1;

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
                Float u = Next1D();

                SpectrumSample T_maj = Sample_MajorantTransmittance(
                    medium, lambda, ray, t_max, u, rng,
                    [&](Point3 point, MediumSample ms, SpectrumSample sigma_maj, SpectrumSample T_maj) -> bool {
                        if (beta.IsBlack())
                        {
                            terminated = true;
                            return false;
                        }

                        SpectrumSample sigma_a = ms.sigma_a;
                        SpectrumSample sigma_s = ms.sigma_s;

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

                            if (!sample_direct_light || bounce > 1)
                            {
                                Photon photon;
                                photon.primitive = nullptr;
                                photon.p = point;
                                photon.normal = Vec3::zero;
                                photon.wi = wo;
                                photon.beta = beta;
                                photon.secondary_terminated = lambda.IsCollapse();
                                volume_photons.push_back(photon);
                            }

                            PhaseFunctionSample phase_sample;
                            if (!ms.phase->Sample_p(&phase_sample, wo, Next2D()))
                            {
                                terminated = true;
                                return false;
                            }

                            beta *= phase_sample.p / phase_sample.pdf;
                            ray.o = point;
                            ray.d = phase_sample.wi;
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

                            return !beta.IsBlack();
                        }

                        default:
                            BulbitAssert(false);
                            return false;
                        }
                    }
                );

                if (terminated || beta.IsBlack())
                {
                    break;
                }

                if (scattered)
                {
                    continue;
                }

                beta *= T_maj / T_maj[hero];
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

            if (IsNonSpecular(bsdf.Flags()) && (!sample_direct_light || bounce > 1))
            {
                Photon photon;
                photon.primitive = isect.primitive;
                photon.p = isect.point;
                photon.normal = isect.normal;
                photon.wi = wo;
                photon.beta = beta;
                photon.secondary_terminated = lambda.IsCollapse();
                surface_photons.push_back(photon);
            }

            BSDFSample bsdf_sample;
            if (!bsdf.Sample_f(
                    &bsdf_sample, wo, rng.NextFloat(), { rng.NextFloat(), rng.NextFloat() }, TransportDirection::ToCamera
                ))
            {
                break;
            }

            if (bsdf_sample.IsTransmission())
            {
                eta_scale *= Sqr(bsdf_sample.eta);
            }

            ray = Ray(isect.point, bsdf_sample.wi);
            medium = isect.GetMedium(bsdf_sample.wi);
            beta *= bsdf_sample.f * AbsDot(isect.shading.normal, bsdf_sample.wi) / bsdf_sample.pdf;

            if (bounce > min_bounces)
            {
                if (Float p = beta.MaxComponent() * eta_scale; p < 1)
                {
                    if (rng.NextFloat() > p)
                    {
                        break;
                    }
                    beta /= p;
                }
            }
        }

        progress->phase_works_dones[phase_index].fetch_add(1, std::memory_order_relaxed);
    });

    photons.clear();
    photons.reserve(n_photons * min_bounces);
    vol_photons.clear();
    vol_photons.reserve(n_photons * min_bounces);

    tl_photons.ForEach([&](std::thread::id tid, std::vector<Photon>& ps) {
        BulbitNotUsed(tid);
        photons.insert(photons.end(), ps.begin(), ps.end());
    });

    tl_vol_photons.ForEach([&](std::thread::id tid, std::vector<Photon>& ps) {
        BulbitNotUsed(tid);
        vol_photons.insert(vol_photons.end(), ps.begin(), ps.end());
    });

    photon_map.Build(photons, radius);
    vol_photon_map.Build(vol_photons, vol_radius);
}

SpectrumSample VolPhotonMappingIntegrator::SampleDirectLight(
    const Vec3& wo,
    const Intersection& isect,
    const Medium* medium,
    const BSDF* bsdf,
    const PhaseFunction* phase,
    const WavelengthSample& lambda,
    Sampler& sampler,
    const SpectrumSample& beta,
    SpectrumSample r_p
) const
{
    BulbitNotUsed(r_p);

    SampledLight sampled_light;
    if (!light_sampler->Sample(&sampled_light, isect, sampler.Next1D()))
    {
        return SpectrumSample(0);
    }

    LightSampleLi light_sample;
    if (!sampled_light.light->Sample_Li(&light_sample, isect, sampler.Next2D(), lambda))
    {
        return SpectrumSample(0);
    }

    if (light_sample.Li.IsBlack())
    {
        return SpectrumSample(0);
    }

    SpectrumSample visibility = Tr(this, isect.point, light_sample.point, medium, lambda);
    if (visibility.IsBlack())
    {
        return SpectrumSample(0);
    }

    Float pdf = light_sample.pdf * sampled_light.pmf;
    SpectrumSample Ld = beta * light_sample.Li * visibility / pdf;
    if (bsdf)
    {
        Ld *= bsdf->f(wo, light_sample.wi) * AbsDot(isect.shading.normal, light_sample.wi);
    }
    else
    {
        Ld *= phase->p(wo, light_sample.wi);
    }

    return Ld;
}

Vec3 VolPhotonMappingIntegrator::Li(
    const Ray& primary_ray, const Medium* primary_medium, WavelengthSample& lambda, Sampler& sampler
) const
{
    constexpr int32 hero = WavelengthSample::hero_lane;

    int32 bounce = 0;
    Vec3 L(0);
    SpectrumSample beta(1);
    SpectrumSample r_u(1);

    bool specular_bounce = false;
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
                [&](Point3 point, MediumSample ms, SpectrumSample sigma_maj, SpectrumSample T_maj) -> bool {
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
                            L += spectral::SpectrumSampleToXYZ(beta_e * sigma_a * Le / r_e.Average(), lambda);
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

                        if (sample_direct_light)
                        {
                            Intersection medium_isect{ .point = point };
                            L += spectral::SpectrumSampleToXYZ(
                                SampleDirectLight(wo, medium_isect, medium, nullptr, ms.phase, lambda, sampler, beta, r_u), lambda
                            );
                        }

                        Vec3 Li(0);
                        vol_photon_map.Query<Photon>(vol_photons, point, vol_radius, [&](const Photon& p) {
                            SpectrumSample contribution = beta * SpectrumSample(ms.phase->p(wo, p.wi)) * p.beta;
                            WavelengthSample photon_lambda = lambda;
                            if (p.secondary_terminated)
                            {
                                photon_lambda.CollapseToPrimary();
                            }
                            Li += spectral::SpectrumSampleToXYZ(contribution, photon_lambda);
                        });

                        Float sphere_volume = 4 / 3.0f * pi * vol_radius * vol_radius * vol_radius;
                        Li *= 1 / (sphere_volume * n_photons);
                        L += Li;

                        terminated = true;
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
        }

        if (!found_intersection)
        {
            if (bounce == 0 || specular_bounce)
            {
                for (Light* light : infinite_lights)
                {
                    L += spectral::SpectrumSampleToXYZ(beta * light->Le(ray, lambda) / r_u.Average(), lambda);
                }
            }

            break;
        }

        if (const Light* area_light = GetAreaLight(isect); area_light)
        {
            SpectrumSample Le = area_light->Le(isect, wo, lambda);
            if (!Le.IsBlack() && (bounce == 0 || specular_bounce))
            {
                L += spectral::SpectrumSampleToXYZ(beta * Le / r_u.Average(), lambda);
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
            medium = isect.GetMedium(ray.d);
            ray.o = isect.point;
            --bounce;
            continue;
        }

        if (IsNonSpecular(bsdf.Flags()))
        {
            if (sample_direct_light)
            {
                L += spectral::SpectrumSampleToXYZ(
                    SampleDirectLight(wo, isect, medium, &bsdf, nullptr, lambda, sampler, beta, r_u), lambda
                );
            }

            Vec3 Li(0);
            photon_map.Query<Photon>(photons, isect.point, radius, [&](const Photon& p) {
                if (isect.primitive->GetMaterial() != p.primitive->GetMaterial())
                {
                    return;
                }

                if (Dot(p.normal, isect.normal) < 0)
                {
                    return;
                }

                SpectrumSample contribution = beta * bsdf.f(wo, p.wi) * AbsDot(isect.shading.normal, p.wi) * p.beta;
                WavelengthSample photon_lambda = lambda;
                if (p.secondary_terminated)
                {
                    photon_lambda.CollapseToPrimary();
                }
                Li += spectral::SpectrumSampleToXYZ(contribution, photon_lambda);
            });

            Li *= 1 / (pi * Sqr(radius) * n_photons);
            L += Li;
            break;
        }

        BSDFSample bsdf_sample;
        if (!bsdf.Sample_f(&bsdf_sample, wo, sampler.Next1D(), sampler.Next2D()))
        {
            break;
        }

        specular_bounce = bsdf_sample.IsSpecular();
        ray = Ray(isect.point, bsdf_sample.wi);
        medium = isect.GetMedium(bsdf_sample.wi);
        beta *= bsdf_sample.f * AbsDot(isect.shading.normal, bsdf_sample.wi) / bsdf_sample.pdf;
    }

    return L;
}

void VolPhotonMappingIntegrator::GatherPhotons(
    const Camera* camera, WavelengthSample lambda, int32 tile_size, MultiPhaseRendering* progress, int32 phase_index
)
{
    Point2i res = camera->GetScreenResolution();

    ParallelFor2D(
        res,
        [&](AABB2i tile) {
            int8 mem[64];
            BufferResource buffer(mem, sizeof(mem));
            Allocator alloc(&buffer);
            Sampler* sampler = sampler_prototype->Clone(alloc);

            for (Point2i pixel : tile)
            {
                sampler->StartPixelSample(pixel, phase_index / 2);

                PrimaryRay primary_ray;
                camera->SampleRay(&primary_ray, pixel, sampler->Next2D(), sampler->Next2D());

                Vec3 L = Li(primary_ray.ray, camera->GetMedium(), lambda, *sampler);
                if (!L.IsNullish())
                {
                    progress->film.AddSample(pixel, primary_ray.weight * L);
                }
            }

            progress->phase_works_dones[phase_index].fetch_add(1, std::memory_order_relaxed);
        },
        tile_size
    );
}

Rendering* VolPhotonMappingIntegrator::Render(Allocator& alloc, const Camera* camera)
{
    const int32 tile_size = 16;
    const int32 n_iterations = std::max<int32>(1, sampler_prototype->samples_per_pixel);

    Point2i res = camera->GetScreenResolution();
    Point2i num_tiles = (res + (tile_size - 1)) / tile_size;
    int32 tile_count = num_tiles.x * num_tiles.y;

    std::vector<size_t> phase_works(2 * n_iterations);
    for (int32 i = 0; i < n_iterations; ++i)
    {
        phase_works[2 * i] = size_t(n_photons);
        phase_works[2 * i + 1] = size_t(tile_count);
    }

    MultiPhaseRendering* progress = alloc.new_object<MultiPhaseRendering>(camera, phase_works);

    progress->job = RunAsync([=, this]() {
        for (int32 iteration = 0; iteration < n_iterations; ++iteration)
        {
            WavelengthSample lambda = IterationLambda(iteration, n_iterations);
            EmitPhotons(progress, lambda, 2 * iteration);
            progress->phase_dones[2 * iteration].store(true, std::memory_order_release);

            GatherPhotons(camera, lambda, tile_size, progress, 2 * iteration + 1);
            progress->phase_dones[2 * iteration + 1].store(true, std::memory_order_release);
        }

        return true;
    });

    return progress;
}

} // namespace bulbit
