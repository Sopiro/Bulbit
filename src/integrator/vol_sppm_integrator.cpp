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
    , sample_dl{ sample_direct_light }
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

// Exactly same as volpath integrator
Spectrum VolSPPMIntegrator::SampleDirectLight(
    const Vec3& wo,
    const Intersection& isect,
    const Medium* medium,
    const BSDF* bsdf,
    const PhaseFunction* phase,
    int32 wavelength,
    Sampler& sampler,
    Spectrum beta,
    Spectrum r_p
) const
{
    Float u0 = sampler.Next1D();
    Point2 u12 = sampler.Next2D();
    SampledLight sampled_light;
    if (!light_sampler->Sample(&sampled_light, isect, u0))
    {
        return Spectrum::black;
    }

    LightSampleLi light_sample;
    if (!sampled_light.light->Sample_Li(&light_sample, isect, u12))
    {
        return Spectrum::black;
    }

    if (light_sample.Li.IsBlack() || light_sample.pdf == 0)
    {
        return Spectrum::black;
    }

    Float light_pdf = sampled_light.pmf * light_sample.pdf;

    Spectrum scattering_f;
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
        scattering_f = Spectrum(phase->p(wo, wi));
        scattering_pdf = phase->PDF(wo, wi);
    }

    if (scattering_f.IsBlack())
    {
        return Spectrum::black;
    }

    Ray light_ray(isect.point, wi);
    Float visibility = light_sample.visibility;

    Spectrum T_ray(1); // Transmittance estimate by ratio tracking
    Spectrum r_u(1);   // Rescaled null scattered distance sampling pdf
    Spectrum r_l(1);   // Rescaled ratio tracking pdf

    RNG rng(Hash(light_ray.o), Hash(light_ray.d));

    while (visibility > 0)
    {
        Intersection light_isect;
        bool found_intersection = Intersect(&light_isect, light_ray, Ray::epsilon, visibility);

        if (found_intersection && light_isect.primitive->GetMaterial())
        {
            // Intersects opaque surface
            return Spectrum::black;
        }

        if (medium)
        {
            Float t_max = found_intersection ? light_isect.t : visibility;
            Float u = rng.NextFloat();

            Spectrum T_maj = Sample_MajorantTransmittance(
                medium, wavelength, light_ray, t_max, u, rng,
                [&](Point3 p, MediumSample ms, Spectrum sigma_maj, Spectrum T_maj) -> bool {
                    BulbitNotUsed(p);

                    // Estimate transmittance along the light ray by ratio tracking
                    Spectrum sigma_n = Max<Float>(sigma_maj - ms.sigma_a - ms.sigma_s, 0);
                    Float pdf = T_maj[wavelength] * sigma_maj[wavelength];
                    T_ray *= T_maj * sigma_n / pdf;
                    r_l *= T_maj * sigma_maj / pdf;
                    r_u *= T_maj * sigma_n / pdf;

                    // Stochastically terminate distance sampling with russian roulette
                    Spectrum Tr = T_ray / (r_u + r_l).Average();
                    if (Tr.MaxComponent() < 0.05f)
                    {
                        constexpr Float rr = 0.75f;
                        if (rng.NextFloat() < rr)
                        {
                            T_ray = Spectrum::black;
                        }
                        else
                        {
                            T_ray /= 1 - rr;
                        }
                    }

                    return !T_ray.IsBlack();
                }
            );

            // Update transmittance estimate for last majorant segment
            T_ray *= T_maj / T_maj[wavelength];
            r_l *= T_maj / T_maj[wavelength];
            r_u *= T_maj / T_maj[wavelength];
        }

        if (T_ray.IsBlack())
        {
            return Spectrum::black;
        }

        if (!found_intersection)
        {
            break;
        }

        // Move the ray origin toward the intersection point
        light_ray.o = light_isect.point;
        visibility -= light_isect.t;
        medium = light_isect.GetMedium(light_ray.d);
    }

    // Multiply the rescaled path probabilities for path sampling of the path
    // up to the last real-scattering vertex where direct lighting is being computed
    // Division by the light path PDF canceled out
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

Rendering* VolSPPMIntegrator::Render(Allocator& alloc, const Camera* camera)
{
    ComputeReflectanceTextures();

    const int32 n_interations = sampler_prototype->samples_per_pixel;
    const int32 tile_size = 16;

    Point2i res = camera->GetScreenResolution();
    Point2i num_tiles = (res + (tile_size - 1)) / tile_size;
    int32 tile_count = num_tiles.x * num_tiles.y;

    std::vector<size_t> phase_works(2 * n_interations);
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
        }

        for (int32 iteration = 0; iteration < n_interations; ++iteration)
        {
            // Generate visible points
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

                        Float eta_scale = 1;

                        int32 bounce = 0;
                        bool specular_bounce = true;
                        bool found_visible_point = false;

                        int32 wavelength = std::min<int32>(int32(sampler->Next1D() * 3), 2);

                        Spectrum beta(primary_ray.weight);

                        Spectrum r_u(1); // Indirect path sampling pdf
                        Spectrum r_l(1); // Light path sampling pdf

                        Ray ray = primary_ray.ray;
                        const Medium* medium = camera->GetMedium();

                        // Trace camera ray and find appropriate visible point
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

                                Spectrum T_maj = Sample_MajorantTransmittance(
                                    medium, wavelength, ray, t_max, u, rng,
                                    [&](Point3 point, MediumSample ms, Spectrum sigma_maj, Spectrum T_maj) -> bool {
                                        if (beta.IsBlack())
                                        {
                                            terminated = true;
                                            return false;
                                        }

                                        if (bounce < max_bounces && !ms.Le.IsBlack())
                                        {
                                            // Add medium emission
                                            Float pdf = sigma_maj[wavelength] * T_maj[wavelength];
                                            Spectrum beta_e = beta * T_maj / pdf;

                                            // Rescaled sampling probability for emission event
                                            Spectrum r_e = r_u * sigma_maj * T_maj / pdf;

                                            if (!r_e.IsBlack())
                                            {
                                                // Single sample wavelength-wise MIS estimator with balance heuristic
                                                vp.Ld += beta_e * ms.sigma_a * ms.Le / r_e.Average();
                                            }
                                        }

                                        Float p_absorb = ms.sigma_a[wavelength] / sigma_maj[wavelength];
                                        Float p_scatter = ms.sigma_s[wavelength] / sigma_maj[wavelength];
                                        Float p_null = std::max<Float>(0, 1 - p_absorb - p_scatter);
                                        Float events[3] = { p_absorb, p_scatter, p_null };

                                        int32 event = SampleDiscrete(events, rng.NextFloat());
                                        switch (event)
                                        {
                                        case 0:
                                        {
                                            // Sampled absorption event
                                            // Add medium emission with MIS weight of 0
                                            terminated = true;
                                            return false;
                                        }

                                        case 1:
                                        {
                                            // Sampled real scattering event
                                            if (bounce++ >= max_bounces || found_visible_point)
                                            {
                                                terminated = true;
                                                return false;
                                            }

                                            Float pdf = T_maj[wavelength] * ms.sigma_s[wavelength];
                                            beta *= T_maj * ms.sigma_s / pdf;
                                            r_u *= T_maj * ms.sigma_s / pdf;

                                            // Add direct light
                                            if (sample_dl)
                                            {
                                                Intersection medium_isect{ .point = point };
                                                vp.Ld += SampleDirectLight(
                                                    wo, medium_isect, medium, nullptr, ms.phase, wavelength, *sampler, beta, r_u
                                                );
                                            }

                                            // Create visible point inside medium
                                            {
                                                vp.primitive = nullptr;
                                                vp.p = point;
                                                vp.normal = Vec3::zero;
                                                vp.wo = wo;
                                                vp.bsdf = {};
                                                vp.phase = ms.phase;
                                                vp.beta = beta;

                                                if (sample_dl)
                                                {
                                                    terminated = true;
                                                    return false;
                                                }

                                                found_visible_point = true;
                                            }

                                            // Sample phase function to find next path direction
                                            PhaseFunctionSample phase_sample;
                                            if (!ms.phase->Sample_p(&phase_sample, wo, sampler->Next2D()))
                                            {
                                                terminated = true;
                                            }

                                            beta *= phase_sample.p / phase_sample.pdf;
                                            // light sampling PDF at this vertex will be incorporated into r_l when it intersects
                                            // the light source
                                            r_l = r_u / phase_sample.pdf;

                                            ray.o = point;
                                            ray.d = phase_sample.wi;

                                            specular_bounce = false;

                                            scattered = true;

                                            return false;
                                        }

                                        case 2:
                                        {
                                            // Sampled null scattering event, continue sampling
                                            Spectrum sigma_n = Max<Float>(sigma_maj - ms.sigma_a - ms.sigma_s, 0);
                                            Float pdf = T_maj[wavelength] * sigma_n[wavelength];
                                            if (pdf == 0)
                                            {
                                                beta = Spectrum::black;
                                            }
                                            else
                                            {
                                                beta *= T_maj * sigma_n / pdf;
                                            }

                                            r_u *= T_maj * sigma_n / pdf;
                                            r_l *= T_maj * sigma_maj / pdf; // Rescaled ratio tracking pdf

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
                                    // Continue medium sampling
                                    continue;
                                }

                                // It past the medium extent
                                beta *= T_maj / T_maj[wavelength];
                                r_u *= T_maj / T_maj[wavelength];
                                r_l *= T_maj / T_maj[wavelength];
                            }

                            if (!found_intersection)
                            {
                                Spectrum L(0);
                                if (bounce == 0 || specular_bounce || !sample_dl)
                                {
                                    for (Light* light : infinite_lights)
                                    {
                                        L += beta * light->Le(ray) / r_u.Average();
                                    }
                                }
                                else
                                {
                                    // Evaluate BSDF sample MIS for infinite light
                                    for (Light* light : infinite_lights)
                                    {
                                        Float light_pdf = light->EvaluatePDF_Li(ray) * light_sampler->EvaluatePMF(light);
                                        r_l *= light_pdf;
                                        L += beta * light->Le(ray) / (r_u + r_l).Average();
                                    }
                                }

                                vp.Ld += L;
                                break;
                            }

                            if (const Light* area_light = GetAreaLight(isect); area_light)
                            {
                                if (Spectrum Le = area_light->Le(isect, wo); !Le.IsBlack())
                                {
                                    Spectrum L(0);

                                    if (bounce == 0 || specular_bounce || !sample_dl)
                                    {
                                        L += beta * Le / r_u.Average();
                                    }
                                    else
                                    {
                                        // Evaluate BSDF sample with MIS for area light
                                        Float light_pdf =
                                            isect.primitive->GetShape()->PDF(isect, ray) * light_sampler->EvaluatePMF(area_light);
                                        r_l *= light_pdf;
                                        L += beta * Le / (r_u + r_l).Average();
                                    }

                                    vp.Ld += L;
                                }
                            }

                            if (bounce++ >= max_bounces || found_visible_point)
                            {
                                break;
                            }

                            Allocator& alloc = thread_allocators.Get();
                            BSDF bsdf;
                            if (!isect.GetBSDF(&bsdf, wo, alloc))
                            {
                                medium = isect.GetMedium(ray.d);
                                ray = Ray(isect.point, -wo);
                                --bounce;
                                continue;
                            }

                            if (sample_dl)
                            {
                                vp.Ld += SampleDirectLight(wo, isect, medium, &bsdf, nullptr, wavelength, *sampler, beta, r_u);
                            }

                            BxDF_Flags flags = bsdf.Flags();
                            if (IsDiffuse(flags) || (IsGlossy(flags) && bounce == max_bounces))
                            {
                                // Create visible point on surface
                                vp.primitive = isect.primitive;
                                vp.p = isect.point;
                                vp.normal = isect.normal;
                                vp.wo = wo;
                                vp.bsdf = bsdf;
                                vp.phase = nullptr;
                                vp.beta = beta;

                                if (sample_dl)
                                {
                                    break;
                                }

                                found_visible_point = true;
                                // Trace one bounce more for the unidirectional MIS contribution
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

                            // Terminate path with russian roulette
                            constexpr int32 min_bounces = 2;
                            if (bounce > min_bounces)
                            {
                                Spectrum rr = beta * eta_scale / r_u.Average();
                                if (Float p = rr.MaxComponent(); p < 1)
                                {
                                    if (sampler->Next1D() > p)
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

            // Build hash grid of visible points
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

                    const Light* light = sampled_light.light;

                    LightSampleLe light_sample;
                    if (!light->Sample_Le(&light_sample, sampler->Next2D(), sampler->Next2D()))
                    {
                        continue;
                    }

                    int32 wavelength = std::min<int32>(int32(sampler->Next1D() * 3), 2);

                    Ray ray = light_sample.ray;
                    const Medium* medium = light_sample.medium;

                    Spectrum beta = light_sample.Le / (sampled_light.pmf * light_sample.pdf_p * light_sample.pdf_w);

                    if (light_sample.normal != Vec3::zero)
                    {
                        beta *= AbsDot(light_sample.normal, ray.d);
                    }

                    if (beta.IsBlack())
                    {
                        continue;
                    }

                    // Trace photon path and add indirect illumination to nearby visible points
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

                            Spectrum T_maj = Sample_MajorantTransmittance(
                                medium, wavelength, ray, t_max, u, rng,
                                [&](Point3 point, MediumSample ms, Spectrum sigma_maj, Spectrum T_maj) -> bool {
                                    if (beta.IsBlack())
                                    {
                                        terminated = true;
                                        return false;
                                    }

                                    Float p_absorb = ms.sigma_a[wavelength] / sigma_maj[wavelength];
                                    Float p_scatter = ms.sigma_s[wavelength] / sigma_maj[wavelength];
                                    Float p_null = std::max<Float>(0, 1 - p_absorb - p_scatter);
                                    Float events[3] = { p_absorb, p_scatter, p_null };

                                    int32 event = SampleDiscrete(events, rng.NextFloat());
                                    switch (event)
                                    {
                                    case 0:
                                    {
                                        // Sampled absorption event
                                        terminated = true;
                                        return false;
                                    }

                                    case 1:
                                    {
                                        // Sampled real scattering event
                                        if (bounce++ >= max_bounces)
                                        {
                                            terminated = true;
                                            return false;
                                        }

                                        Float pdf = T_maj[wavelength] * ms.sigma_s[wavelength];
                                        beta *= T_maj * ms.sigma_s / pdf;

                                        if (!sample_dl || bounce > 1)
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

                                                Spectrum phi = beta * vp.phase->p(vp.wo, wo);
                                                for (int32 s = 0; s < 3; ++s)
                                                {
                                                    vp.phi_i_vol[s] += phi[s];
                                                }

                                                ++vp.m_vol;
                                            });
                                        }

                                        // Sample phase function to find next path direction
                                        PhaseFunctionSample phase_sample;
                                        if (!ms.phase->Sample_p(&phase_sample, wo, sampler->Next2D()))
                                        {
                                            terminated = true;
                                        }

                                        beta *= phase_sample.p / phase_sample.pdf;

                                        ray.o = point;
                                        ray.d = phase_sample.wi;

                                        scattered = true;

                                        return false;
                                    }

                                    case 2:
                                    {
                                        // Sampled null scattering event, continue sampling
                                        Spectrum sigma_n = Max<Float>(sigma_maj - ms.sigma_a - ms.sigma_s, 0);
                                        Float pdf = T_maj[wavelength] * sigma_n[wavelength];
                                        if (pdf == 0)
                                        {
                                            beta = Spectrum::black;
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
                                // Continue medium sampling
                                continue;
                            }

                            // It past the medium extent
                            beta *= T_maj / T_maj[wavelength];
                        }

                        if (!found_intersection)
                        {
                            break;
                        }

                        if (bounce++ >= max_bounces)
                        {
                            break;
                        }

                        if (!sample_dl || bounce > 1)
                        {
                            // Query hash grid for nearby visible points
                            grid.Query<VisiblePoint>(visible_points, isect.point, [&](VisiblePoint& vp) {
                                if (!vp.primitive)
                                {
                                    return;
                                }

                                if (isect.primitive->GetMaterial() != vp.primitive->GetMaterial())
                                {
                                    return;
                                }

                                if (Dot(vp.normal, isect.normal) < 0.95f)
                                {
                                    return;
                                }

                                if (Dist2(isect.point, vp.p) > Sqr(vp.radius))
                                {
                                    return;
                                }

                                // Compute photon flux and record to visible point
                                Spectrum phi = beta * vp.bsdf.f(vp.wo, wo);

                                for (int32 s = 0; s < 3; ++s)
                                {
                                    vp.phi_i[s] += phi[s];
                                }

                                ++vp.m;
                            });
                        }

                        int8 bsdf_mem[max_bxdf_size];
                        BufferResource bsdf_res(bsdf_mem, sizeof(bsdf_mem));
                        Allocator bsdf_alloc(&bsdf_res);
                        BSDF bsdf;
                        if (!isect.GetBSDF(&bsdf, wo, bsdf_alloc))
                        {
                            ray = Ray(isect.point, -wo);
                            --bounce;
                            continue;
                        }

                        BSDFSample bsdf_sample;
                        if (!bsdf.Sample_f(&bsdf_sample, wo, sampler->Next1D(), sampler->Next2D(), TransportDirection::ToCamera))
                        {
                            continue;
                        }

                        Spectrum beta0 = beta;
                        beta *= bsdf_sample.f * AbsDot(isect.shading.normal, bsdf_sample.wi) / bsdf_sample.pdf;
                        ray = Ray(isect.point, bsdf_sample.wi);

                        // Terminate path with russian roulette based on beta ratio
                        if (Float p = beta.MaxComponent() / beta0.MaxComponent(); p < 1)
                        {
                            if (sampler->Next1D() > p)
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

                progress->phase_works_dones[2 * iteration + 1].fetch_add(end - begin, std::memory_order_relaxed);
            });

            ParallelFor2D(
                res,
                [&](AABB2i tile) {
                    for (Point2i pixel : tile)
                    {
                        int32 index = res.x * pixel.y + pixel.x;
                        VisiblePoint& vp = visible_points[index];

                        const Float gamma = 2.0f / 3.0f;
                        if (int32 m = vp.m.load(std::memory_order_relaxed); m > 0)
                        {
                            // Update effective photon count
                            Float n_new = vp.n + gamma * m;

                            // Update search radius so that the expected photon density
                            // inside the new radius matches the updated photon count
                            // r_{i+1} = r_i * sqrt(N_{i+1} / (N_i + M_i))
                            Float r_new = vp.radius * std::sqrt(n_new / (vp.n + m));

                            // Update τ, the accumulated flux scaled to remain consistent after the radius update:
                            // tau_{i+1} = (tau_i + phi_i) * (r_{i+1}^2 / r_i^2)
                            Spectrum phi_i(vp.phi_i[0], vp.phi_i[1], vp.phi_i[2]);
                            vp.tau = (vp.tau + vp.beta * phi_i) * Sqr(r_new / vp.radius);

                            vp.n = n_new;
                            vp.radius = r_new;

                            vp.m = 0;
                            for (int32 s = 0; s < 3; ++s)
                            {
                                vp.phi_i[s] = 0;
                            }
                        }

                        if (int32 m_vol = vp.m_vol.load(std::memory_order_relaxed); m_vol > 0)
                        {
                            Float n_new = vp.n_vol + gamma * m_vol;
                            Float r_new = vp.radius_vol * std::sqrt(n_new / (vp.n_vol + m_vol));

                            Spectrum phi_i_vol(vp.phi_i_vol[0], vp.phi_i_vol[1], vp.phi_i_vol[2]);
                            vp.tau_vol = (vp.tau_vol + vp.beta * phi_i_vol) * Sqr(r_new / vp.radius_vol);

                            vp.n_vol = n_new;
                            vp.radius_vol = r_new;

                            vp.m_vol = 0;
                            for (int32 s = 0; s < 3; ++s)
                            {
                                vp.phi_i_vol[s] = 0;
                            }
                        }

                        vp.beta = Spectrum::black;
                        vp.bsdf = {};
                        vp.phase = nullptr;
                    }
                },
                tile_size
            );

            if (iteration < n_interations - 1)
            {
                progress->phase_dones[2 * iteration + 1].store(true, std::memory_order_release);
            }

            for (size_t i = 0; i < thread_buffers.size(); ++i)
            {
                thread_buffers[i]->release();
            }
        }

        size_t total_photons = n_interations * photons_per_iteration;

        ParallelFor2D(
            res,
            [&](AABB2i tile) {
                for (Point2i pixel : tile)
                {
                    int32 index = res.x * pixel.y + pixel.x;
                    VisiblePoint& vp = visible_points[index];

                    Spectrum L = vp.Ld / n_interations;

                    const Float circle_area = pi * vp.radius * vp.radius;
                    const Float sphere_volume = 4 / 3.0f * pi * vp.radius_vol * vp.radius_vol * vp.radius_vol;
                    L += vp.tau / (total_photons * circle_area);
                    L += vp.tau_vol / (total_photons * sphere_volume);

                    progress->film.AddSample(pixel, L);
                }
            },
            tile_size
        );

        progress->phase_dones[2 * (n_interations - 1) + 1].store(true, std::memory_order_release);

        return true;
    });

    return progress;
}

} // namespace bulbit
