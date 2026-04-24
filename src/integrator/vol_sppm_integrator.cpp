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

VolSPPMIntegrator::VolSPPMIntegrator(
    const Intersectable* accel,
    std::vector<Light*> lights,
    const Sampler* sampler,
    int32 max_bounces,
    int32 photons_per_iteration,
    Float radius_surface,
    Float radius_volume,
    bool sample_direct_light
)
    : Integrator(accel, std::move(lights), std::make_unique<PowerLightSampler>())
    , sampler_prototype{ sampler }
    , max_bounces{ max_bounces }
    , photons_per_iteration{ photons_per_iteration }
    , initial_radius_surface{ radius_surface }
    , initial_radius_volume{ radius_volume }
    , sample_direct_light{ sample_direct_light }
{
    AABB world_bounds = accel->GetAABB();
    Point3 world_center;
    Float world_radius;
    world_bounds.ComputeBoundingSphere(&world_center, &world_radius);

    if (initial_radius_surface <= 0)
    {
        initial_radius_surface = 2 * world_radius * 3e-3f;
    }
    if (initial_radius_volume <= 0)
    {
        initial_radius_volume = 2 * world_radius * 1e-2f;
    }
}

SpectrumSample VolSPPMIntegrator::SampleDirectLight(
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

    if (light_sample.Li.IsBlack() || light_sample.pdf == 0)
    {
        return SpectrumSample(0);
    }

    Float light_pdf = sampled_light.pmf * light_sample.pdf;
    Vec3 wi = light_sample.wi;

    SpectrumSample scattering_f;
    Float scattering_pdf;
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

    return beta * scattering_f * T_ray * light_sample.Li / (r_u + r_l).Average();
}

Rendering* VolSPPMIntegrator::Render(Allocator& alloc, const Camera* camera)
{
    constexpr int32 hero = WavelengthSample::hero_lane;

    const int32 n_iterations = std::max<int32>(1, sampler_prototype->samples_per_pixel);
    const int32 tile_size = 16;

    Point2i res = camera->GetScreenResolution();
    Point2i num_tiles = (res + (tile_size - 1)) / tile_size;
    int32 tile_count = num_tiles.x * num_tiles.y;

    std::vector<size_t> phase_works(2 * n_iterations);
    for (size_t i = 0; i < phase_works.size(); i += 2)
    {
        phase_works[i] = size_t(tile_count);
        phase_works[i + 1] = size_t(photons_per_iteration);
    }

    MultiPhaseRendering* progress = alloc.new_object<MultiPhaseRendering>(camera, phase_works);
    progress->job = RunAsync([=, this]() {
        std::vector<std::unique_ptr<BufferResource>> thread_buffers;
        ThreadLocal<Allocator> thread_allocators([&thread_buffers]() {
            thread_buffers.push_back(std::make_unique<BufferResource>(1024 * 1024));
            BufferResource* ptr = thread_buffers.back().get();
            return Allocator(ptr);
        });

        int32 n_pixels = res.x * res.y;
        std::vector<VisiblePoint> visible_points(n_pixels);
        for (VisiblePoint& vp : visible_points)
        {
            vp.radius = initial_radius_surface;
            vp.radius_vol = initial_radius_volume;
            for (int32 c = 0; c < 3; ++c)
            {
                vp.phi_i[c].store(0, std::memory_order_relaxed);
                vp.phi_i_vol[c].store(0, std::memory_order_relaxed);
            }
        }

        for (int32 iteration = 0; iteration < n_iterations; ++iteration)
        {
            WavelengthSample lambda = IterationLambda(iteration, n_iterations);
            // camera pass
            ParallelFor2D(
                res,
                [&](AABB2i tile) {
                    int8 mem[64];
                    BufferResource buffer(mem, sizeof(mem));
                    Allocator sampler_alloc(&buffer);
                    Sampler* sampler = sampler_prototype->Clone(sampler_alloc);

                    for (Point2i pixel : tile)
                    {
                        sampler->StartPixelSample(pixel, iteration);

                        PrimaryRay primary_ray;
                        camera->SampleRay(&primary_ray, pixel, sampler->Next2D(), sampler->Next2D());

                        int32 index = res.x * pixel.y + pixel.x;
                        VisiblePoint& vp = visible_points[index];
                        vp.secondary_terminated = false;

                        Float eta_scale = 1;
                        int32 bounce = 0;
                        bool specular_bounce = true;
                        bool found_visible_point = false;

                        SpectrumSample beta(primary_ray.weight);
                        SpectrumSample r_u(1);
                        SpectrumSample r_l(1);

                        Ray ray = primary_ray.ray;
                        const Medium* medium = camera->GetMedium();

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
                                Float u = sampler->Next1D();

                                uint64 hash0 = Hash(sampler->Next1D());
                                uint64 hash1 = Hash(sampler->Next1D());
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
                                                vp.Ld +=
                                                    spectral::SpectrumSampleToXYZ(beta_e * sigma_a * Le / r_e.Average(), lambda);
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
                                            if (bounce++ >= max_bounces || found_visible_point)
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
                                                vp.Ld += spectral::SpectrumSampleToXYZ(
                                                    SampleDirectLight(
                                                        wo, medium_isect, medium, nullptr, ms.phase, lambda, *sampler, beta, r_u
                                                    ),
                                                    lambda
                                                );
                                            }

                                            vp.primitive = nullptr;
                                            vp.p = point;
                                            vp.normal = Vec3::zero;
                                            vp.wo = wo;
                                            vp.secondary_terminated = lambda.IsCollapse();
                                            vp.bsdf = {};
                                            vp.phase = ms.phase;
                                            vp.beta = beta;

                                            if (!sample_direct_light)
                                            {
                                                terminated = true;
                                                return false;
                                            }

                                            found_visible_point = true;

                                            PhaseFunctionSample phase_sample;
                                            if (!ms.phase->Sample_p(&phase_sample, wo, sampler->Next2D()))
                                            {
                                                terminated = true;
                                                return false;
                                            }

                                            beta *= phase_sample.p / phase_sample.pdf;
                                            r_l = r_u / phase_sample.pdf;
                                            ray.o = point;
                                            ray.d = phase_sample.wi;
                                            specular_bounce = false;
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
                                SpectrumSample L(0);
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

                                vp.Ld += spectral::SpectrumSampleToXYZ(L, lambda);
                                break;
                            }

                            if (const Light* area_light = GetAreaLight(isect); area_light)
                            {
                                SpectrumSample Le = area_light->Le(isect, wo, lambda);
                                if (!Le.IsBlack())
                                {
                                    SpectrumSample L(0);
                                    if (bounce == 0 || specular_bounce)
                                    {
                                        L += beta * Le / r_u.Average();
                                    }
                                    else
                                    {
                                        Float light_pdf =
                                            isect.primitive->GetShape()->PDF(isect, ray) * light_sampler->EvaluatePMF(area_light);
                                        L += beta * Le / (r_u + r_l * light_pdf).Average();
                                    }

                                    vp.Ld += spectral::SpectrumSampleToXYZ(L, lambda);
                                }
                            }

                            if (bounce++ >= max_bounces || found_visible_point)
                            {
                                break;
                            }

                            Allocator& thread_alloc = thread_allocators.Get();
                            BSDF bsdf;
                            WavelengthSample path_lambda = lambda;
                            if (!isect.GetBSDF(&bsdf, wo, path_lambda, thread_alloc))
                            {
                                medium = isect.GetMedium(ray.d);
                                ray = Ray(isect.point, -wo);
                                --bounce;
                                continue;
                            }

                            if (sample_direct_light)
                            {
                                vp.Ld += spectral::SpectrumSampleToXYZ(
                                    SampleDirectLight(wo, isect, medium, &bsdf, nullptr, path_lambda, *sampler, beta, r_u),
                                    path_lambda
                                );
                            }

                            BxDF_Flags flags = bsdf.Flags();
                            if (IsDiffuse(flags) || (IsGlossy(flags) && bounce == max_bounces) ||
                                (IsNonSpecular(flags) && !sample_direct_light))
                            {
                                vp.primitive = isect.primitive;
                                vp.p = isect.point;
                                vp.normal = isect.normal;
                                vp.wo = wo;
                                vp.secondary_terminated = path_lambda.IsCollapse();
                                vp.bsdf = bsdf;
                                vp.phase = nullptr;
                                vp.beta = beta;

                                if (!sample_direct_light)
                                {
                                    break;
                                }

                                found_visible_point = true;
                            }

                            BSDFSample bsdf_sample;
                            if (!bsdf.Sample_f(&bsdf_sample, wo, sampler->Next1D(), sampler->Next2D()))
                            {
                                break;
                            }

                            specular_bounce = bsdf_sample.IsSpecular();
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

                            constexpr int32 min_bounces = 2;
                            if (bounce > min_bounces)
                            {
                                SpectrumSample rr = beta * eta_scale / r_u.Average();
                                if (Float p = rr.MaxComponent(); p < 1)
                                {
                                    if (sampler->Next1D() > p)
                                    {
                                        break;
                                    }
                                    beta /= p;
                                }
                            }
                        }
                    }

                    progress->phase_works_dones[2 * iteration].fetch_add(1, std::memory_order_relaxed);
                },
                tile_size
            );

            Float max_radius = 0;
            for (const VisiblePoint& vp : visible_points)
            {
                if (vp.beta.IsBlack())
                {
                    continue;
                }

                max_radius = std::max({ max_radius, vp.radius, vp.radius_vol });
            }

            HashGrid grid;
            grid.Build(visible_points, max_radius);

            progress->phase_dones[2 * iteration].store(true, std::memory_order_release);

            ParallelFor(0, photons_per_iteration, [&](int32 begin, int32 end) {
                int8 mem[64];
                BufferResource buffer(mem, sizeof(mem));
                Allocator alloc(&buffer);
                Sampler* sampler = sampler_prototype->Clone(alloc);

                for (int32 i = begin; i < end; ++i)
                {
                    sampler->StartPixelSample({ -i, -i }, iteration);

                    SampledLight sampled_light;
                    if (!light_sampler->Sample(&sampled_light, Intersection{}, sampler->Next1D()))
                    {
                        continue;
                    }

                    LightSampleLe light_sample;
                    if (!sampled_light.light->Sample_Le(&light_sample, sampler->Next2D(), sampler->Next2D(), lambda))
                    {
                        continue;
                    }

                    Ray ray = light_sample.ray;
                    const Medium* medium = light_sample.medium;
                    SpectrumSample beta = light_sample.Le / (sampled_light.pmf * light_sample.pdf_p * light_sample.pdf_w);
                    if (light_sample.normal != Vec3::zero)
                    {
                        beta *= AbsDot(light_sample.normal, ray.d);
                    }

                    if (beta.IsBlack())
                    {
                        continue;
                    }

                    int32 bounce = 0;
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
                            Float u = sampler->Next1D();

                            uint64 hash0 = Hash(sampler->Next1D());
                            uint64 hash1 = Hash(sampler->Next1D());
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
                                            grid.Query<VisiblePoint>(visible_points, point, [&](VisiblePoint& vp) {
                                                if (vp.primitive || !vp.phase)
                                                {
                                                    return;
                                                }

                                                if (Dist2(point, vp.p) > Sqr(vp.radius_vol))
                                                {
                                                    return;
                                                }

                                                SpectrumSample phi = beta * SpectrumSample(vp.phase->p(vp.wo, wo));
                                                WavelengthSample photon_lambda = lambda;
                                                if (vp.secondary_terminated)
                                                {
                                                    photon_lambda.CollapseToPrimary();
                                                }

                                                Vec3 phi_xyz = spectral::SpectrumSampleToXYZ(vp.beta * phi, photon_lambda);
                                                for (int32 c = 0; c < 3; ++c)
                                                {
                                                    vp.phi_i_vol[c].fetch_add(phi_xyz[c], std::memory_order_relaxed);
                                                }

                                                vp.m_vol.fetch_add(1, std::memory_order_relaxed);
                                            });
                                        }

                                        PhaseFunctionSample phase_sample;
                                        if (!ms.phase->Sample_p(&phase_sample, wo, sampler->Next2D()))
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

                        if (!sample_direct_light || bounce > 1)
                        {
                            grid.Query<VisiblePoint>(visible_points, isect.point, [&](VisiblePoint& vp) {
                                if (!vp.primitive)
                                {
                                    return;
                                }

                                if (isect.primitive->GetMaterial() != vp.primitive->GetMaterial())
                                {
                                    return;
                                }

                                if (Dot(vp.normal, isect.normal) < 0)
                                {
                                    return;
                                }

                                if (Dist2(isect.point, vp.p) > Sqr(vp.radius))
                                {
                                    return;
                                }

                                SpectrumSample phi = beta * vp.bsdf.f(vp.wo, wo);
                                WavelengthSample photon_lambda = lambda;
                                if (vp.secondary_terminated)
                                {
                                    photon_lambda.CollapseToPrimary();
                                }

                                Vec3 phi_xyz = spectral::SpectrumSampleToXYZ(vp.beta * phi, photon_lambda);
                                for (int32 c = 0; c < 3; ++c)
                                {
                                    vp.phi_i[c].fetch_add(phi_xyz[c], std::memory_order_relaxed);
                                }

                                vp.m.fetch_add(1, std::memory_order_relaxed);
                            });
                        }

                        int8 bsdf_mem[max_bxdf_size];
                        BufferResource bsdf_res(bsdf_mem, sizeof(bsdf_mem));
                        Allocator bsdf_alloc(&bsdf_res);
                        BSDF bsdf;
                        WavelengthSample path_lambda = lambda;
                        if (!isect.GetBSDF(&bsdf, wo, path_lambda, bsdf_alloc))
                        {
                            medium = isect.GetMedium(ray.d);
                            ray = Ray(isect.point, -wo);
                            --bounce;
                            continue;
                        }

                        BSDFSample bsdf_sample;
                        if (!bsdf.Sample_f(&bsdf_sample, wo, sampler->Next1D(), sampler->Next2D(), TransportDirection::ToCamera))
                        {
                            continue;
                        }

                        SpectrumSample beta0 = beta;
                        beta *= bsdf_sample.f * AbsDot(isect.shading.normal, bsdf_sample.wi) / bsdf_sample.pdf;
                        ray = Ray(isect.point, bsdf_sample.wi);
                        medium = isect.GetMedium(bsdf_sample.wi);

                        if (Float p = beta.MaxComponent() / beta0.MaxComponent(); p < 1)
                        {
                            if (sampler->Next1D() > p)
                            {
                                break;
                            }
                            beta /= p;
                        }
                    }
                }

                progress->phase_works_dones[2 * iteration + 1].fetch_add(end - begin, std::memory_order_relaxed);
            });

            ParallelFor2D(
                res,
                [&](AABB2i tile) {
                    for (Point2i pixel : tile)
                    {
                        int32 index = res.x * pixel.y + pixel.x;
                        VisiblePoint& vp = visible_points[index];

                        constexpr Float gamma = 2.0f / 3.0f;
                        if (int32 m = vp.m.load(std::memory_order_relaxed); m > 0)
                        {
                            Float n_new = vp.n + gamma * m;
                            Float r_new = vp.radius * std::sqrt(n_new / (vp.n + m));

                            Vec3 phi_i_xyz(0);
                            for (int32 c = 0; c < 3; ++c)
                            {
                                phi_i_xyz[c] = vp.phi_i[c].load(std::memory_order_relaxed);
                            }

                            vp.tau = (vp.tau + phi_i_xyz) * Sqr(r_new / vp.radius);
                            vp.n = n_new;
                            vp.radius = r_new;
                            vp.m.store(0, std::memory_order_relaxed);
                            for (int32 c = 0; c < 3; ++c)
                            {
                                vp.phi_i[c].store(0, std::memory_order_relaxed);
                            }
                        }

                        if (int32 m_vol = vp.m_vol.load(std::memory_order_relaxed); m_vol > 0)
                        {
                            Float n_new = vp.n_vol + gamma * m_vol;
                            Float r_new = vp.radius_vol * std::sqrt(n_new / (vp.n_vol + m_vol));

                            Vec3 phi_i_vol_xyz(0);
                            for (int32 c = 0; c < 3; ++c)
                            {
                                phi_i_vol_xyz[c] = vp.phi_i_vol[c].load(std::memory_order_relaxed);
                            }

                            vp.tau_vol = (vp.tau_vol + phi_i_vol_xyz) * Sqr(r_new / vp.radius_vol);
                            vp.n_vol = n_new;
                            vp.radius_vol = r_new;
                            vp.m_vol.store(0, std::memory_order_relaxed);
                            for (int32 c = 0; c < 3; ++c)
                            {
                                vp.phi_i_vol[c].store(0, std::memory_order_relaxed);
                            }
                        }

                        vp.beta = SpectrumSample(0);
                        vp.bsdf = {};
                        vp.phase = nullptr;
                    }
                },
                tile_size
            );

            if (iteration < n_iterations - 1)
            {
                progress->phase_dones[2 * iteration + 1].store(true, std::memory_order_release);
            }

            for (std::unique_ptr<BufferResource>& thread_buffer : thread_buffers)
            {
                thread_buffer->release();
            }
        }

        size_t total_photons = n_iterations * photons_per_iteration;

        ParallelFor2D(
            res,
            [&](AABB2i tile) {
                for (Point2i pixel : tile)
                {
                    int32 index = res.x * pixel.y + pixel.x;
                    VisiblePoint& vp = visible_points[index];

                    Vec3 L = vp.Ld / n_iterations;
                    Float circle_area = pi * vp.radius * vp.radius;
                    Float sphere_volume = 4 / 3.0f * pi * vp.radius_vol * vp.radius_vol * vp.radius_vol;
                    L += vp.tau / (total_photons * circle_area);
                    L += vp.tau_vol / (total_photons * sphere_volume);
                    progress->film.AddSample(pixel, L);
                }
            },
            tile_size
        );

        progress->phase_dones[2 * (n_iterations - 1) + 1].store(true, std::memory_order_release);
        return true;
    });

    return progress;
}

} // namespace bulbit
