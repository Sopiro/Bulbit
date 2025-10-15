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
    , sample_dl{ sample_direct_light }
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

void VolPhotonMappingIntegrator::EmitPhotons(MultiPhaseRendering* progress)
{
    const int32 min_bounces = 2;

    ThreadLocal<std::vector<Photon>> tl_photons;
    ThreadLocal<std::vector<Photon>> tl_photons_vol;

    ParallelFor(0, n_photons, [&](int32 i) {
        RNG rng(Hash(n_photons, radius, vol_radius), Hash(i));

        auto Next1D = [&]() { return rng.NextFloat(); };
        auto Next2D = [&]() { return Point2{ rng.NextFloat(), rng.NextFloat() }; };

        std::vector<Photon>& ps = tl_photons.Get();
        std::vector<Photon>& ps_vol = tl_photons_vol.Get();

        SampledLight sampled_light;
        if (!light_sampler->Sample(&sampled_light, Intersection{}, rng.NextFloat()))
        {
            return;
        }

        LightSampleLe light_sample;
        if (!sampled_light.light->Sample_Le(&light_sample, Next2D(), Next2D()))
        {
            return;
        }

        Ray ray = light_sample.ray; // Photon ray
        const Medium* medium = light_sample.medium;

        Spectrum beta = light_sample.Le / (sampled_light.pmf * light_sample.pdf_p * light_sample.pdf_w);
        Spectrum r_u(1);

        int32 wavelength = std::min<int32>(int32(Next1D() * 3), 2);

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
                            r_u *= T_maj * ms.sigma_s / pdf;

                            // Store volume photon
                            if (!sample_dl || bounce > 1)
                            {
                                Photon vp;
                                vp.primitive = nullptr;
                                vp.p = point;
                                vp.normal = Vec3::zero;
                                vp.wi = wo;
                                vp.beta = beta;

                                ps_vol.push_back(vp);
                            }

                            // Sample phase function to find next path direction
                            PhaseFunctionSample phase_sample;
                            if (!ms.phase->Sample_p(&phase_sample, wo, Next2D()))
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

                            r_u *= T_maj * sigma_n / pdf;

                            return !beta.IsBlack();
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
            if (!isect.GetBSDF(&bsdf, wo, alloc))
            {
                medium = isect.GetMedium(ray.d);
                ray.o = isect.point;
                --bounce;
                continue;
            }

            // Store photon to non specular surface
            if ((!sample_dl || (bounce > 1)) && IsNonSpecular(bsdf.Flags()))
            {
                Photon p;
                p.primitive = isect.primitive;
                p.p = isect.point;
                p.normal = isect.normal;
                p.wi = wo;
                p.beta = beta;

                ps.push_back(p);
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
            beta *= bsdf_sample.f * AbsDot(isect.shading.normal, bsdf_sample.wi) / bsdf_sample.pdf;

            if (bounce > min_bounces)
            {
                if (Float p = beta.MaxComponent() * eta_scale; p < 1)
                {
                    if (rng.NextFloat() > p)
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

        progress->phase_works_dones[0].fetch_add(1, std::memory_order_relaxed);
    });

    photons.reserve(n_photons * min_bounces);
    vol_photons.reserve(n_photons * min_bounces);

    tl_photons.ForEach([&](std::thread::id tid, std::vector<Photon>& ps) {
        BulbitNotUsed(tid);
        photons.insert(photons.end(), ps.begin(), ps.end());
    });

    tl_photons_vol.ForEach([&](std::thread::id tid, std::vector<Photon>& ps) {
        BulbitNotUsed(tid);
        vol_photons.insert(vol_photons.end(), ps.begin(), ps.end());
    });

    photon_map.Build(photons, radius);
    vol_photon_map.Build(vol_photons, vol_radius);
}

Spectrum VolPhotonMappingIntegrator::SampleDirectLight(
    const Vec3& wo,
    const Intersection& isect,
    const Medium* medium,
    const BSDF* bsdf,
    const PhaseFunction* phase,
    int32 wavelength,
    Sampler& sampler,
    const Spectrum& beta,
    Spectrum r_p
) const
{

    BulbitNotUsed(r_p);

    SampledLight sampled_light;
    if (!light_sampler->Sample(&sampled_light, isect, sampler.Next1D()))
    {
        return Spectrum::black;
    }

    LightSampleLi light_sample;
    if (!sampled_light.light->Sample_Li(&light_sample, isect, sampler.Next2D()))
    {
        return Spectrum::black;
    }

    if (light_sample.Li.IsBlack())
    {
        return Spectrum::black;
    }

    Spectrum V = Tr(this, isect.point, light_sample.point, medium, wavelength);
    if (V.IsBlack())
    {
        return Spectrum::black;
    }

    Float pdf = light_sample.pdf * sampled_light.pmf;
    Spectrum l = beta * light_sample.Li * V / pdf;

    if (bsdf)
    {
        l *= bsdf->f(wo, light_sample.wi) * AbsDot(isect.shading.normal, light_sample.wi);
    }
    else
    {
        l *= phase->p(wo, light_sample.wi);
    }

    return l;
}

Spectrum VolPhotonMappingIntegrator::Li(const Ray& primary_ray, const Medium* primary_medium, Sampler& sampler) const
{
    int32 bounce = 0;

    Spectrum L(0);
    Spectrum beta(1);
    Spectrum r_u(1);

    bool was_specular_bounce = false;

    Ray ray = primary_ray;
    const Medium* medium = primary_medium;
    int32 wavelength = std::min<int32>(int32(sampler.Next1D() * 3), 2);

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
                        Float pdf = T_maj[wavelength] * ms.sigma_a[wavelength];
                        beta *= T_maj / pdf;
                        r_u *= T_maj * ms.sigma_a / pdf;

                        L += beta * ms.sigma_a * ms.Le / r_u.Average();
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
                        r_u *= T_maj * ms.sigma_s / pdf;

                        if (sample_dl)
                        {
                            Intersection medium_isect{ .point = point };
                            L += SampleDirectLight(wo, medium_isect, medium, nullptr, ms.phase, wavelength, sampler, beta, r_u);
                        }

                        // Estimate volumetric indirect light contribution using volume photon map
                        Spectrum L_i(0);

                        vol_photon_map.Query<Photon>(vol_photons, point, vol_radius, [&](const Photon& p) {
                            L_i += ms.phase->p(wo, p.wi) * p.beta;
                        });

                        const Float sphere_volume = 4 / 3.0f * pi * vol_radius * vol_radius * vol_radius;
                        L_i *= 1 / (sphere_volume * n_photons);

                        L += beta * L_i;

                        // Done!
                        terminated = true;
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
        }

        if (!found_intersection)
        {
            if (bounce == 0 || was_specular_bounce || !sample_dl)
            {
                for (Light* light : infinite_lights)
                {
                    L += beta * light->Le(ray);
                }
            }

            break;
        }

        if (const Light* area_light = GetAreaLight(isect); area_light)
        {
            if (Spectrum Le = area_light->Le(isect, wo); !Le.IsBlack())
            {
                if (bounce == 0 || was_specular_bounce || !sample_dl)
                {
                    L += beta * Le;
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
        if (!isect.GetBSDF(&bsdf, wo, alloc))
        {
            medium = isect.GetMedium(ray.d);
            ray.o = isect.point;
            --bounce;
            continue;
        }

        if (IsNonSpecular(bsdf.Flags()))
        {
            // Estimate direct light
            if (sample_dl)
            {
                L += SampleDirectLight(wo, isect, medium, &bsdf, nullptr, wavelength, sampler, beta, r_u);
            }

            // Estimate indirect light by gathering nearby photons
            Spectrum L_i(0);

            photon_map.Query<Photon>(photons, isect.point, radius, [&](const Photon& p) {
                if (isect.primitive->GetMaterial() != p.primitive->GetMaterial())
                {
                    return;
                }

                if (Dot(p.normal, isect.normal) < 0.95f)
                {
                    return;
                }

                L_i += bsdf.f(wo, p.wi) * AbsDot(isect.shading.normal, p.wi) * p.beta;
            });

            L_i *= 1 / (pi * Sqr(radius) * n_photons);
            L += beta * L_i;

            // Done!
            break;
        }
        else
        {
            was_specular_bounce = true;
        }

        BSDFSample bsdf_sample;
        if (!bsdf.Sample_f(&bsdf_sample, wo, sampler.Next1D(), sampler.Next2D()))
        {
            break;
        }

        ray = Ray(isect.point, bsdf_sample.wi);
        beta *= bsdf_sample.f * AbsDot(isect.shading.normal, bsdf_sample.wi) / bsdf_sample.pdf;
    }

    return L;
}

void VolPhotonMappingIntegrator::GatherPhotons(const Camera* camera, int32 tile_size, MultiPhaseRendering* progress)
{
    Point2i res = camera->GetScreenResolution();
    const int32 spp = sampler_prototype->samples_per_pixel;

    ParallelFor2D(
        res,
        [&](AABB2i tile) {
            int8 mem[64];
            BufferResource buffer(mem, sizeof(mem));
            Allocator alloc(&buffer);
            Sampler* sampler = sampler_prototype->Clone(alloc);

            for (Point2i pixel : tile)
            {
                for (int32 sample = 0; sample < spp; ++sample)
                {
                    sampler->StartPixelSample(pixel, sample);

                    PrimaryRay primary_ray;
                    camera->SampleRay(&primary_ray, pixel, sampler->Next2D(), sampler->Next2D());

                    Spectrum L = Li(primary_ray.ray, camera->GetMedium(), *sampler);
                    if (!L.IsNullish())
                    {
                        progress->film.AddSample(pixel, primary_ray.weight * L);
                    }
                }
            }

            progress->phase_works_dones[1].fetch_add(1, std::memory_order_relaxed);
        },
        tile_size
    );
}

Rendering* VolPhotonMappingIntegrator::Render(Allocator& alloc, const Camera* camera)
{
    ComputeReflectanceTextures();

    const int32 tile_size = 16;

    Point2i res = camera->GetScreenResolution();
    Point2i num_tiles = (res + (tile_size - 1)) / tile_size;
    int32 tile_count = num_tiles.x * num_tiles.y;

    std::array<size_t, 2> phase_works = { size_t(n_photons), size_t(tile_count) };
    MultiPhaseRendering* progress = alloc.new_object<MultiPhaseRendering>(camera, phase_works);

    progress->job = RunAsync([=, this]() {
        EmitPhotons(progress);
        progress->phase_dones[0].store(true, std::memory_order_release);
        GatherPhotons(camera, tile_size, progress);
        progress->phase_dones[1].store(true, std::memory_order_release);
        return true;
    });

    return progress;
}

} // namespace bulbit
