#include "bulbit/async_job.h"
#include "bulbit/bsdf.h"
#include "bulbit/bxdfs.h"
#include "bulbit/film.h"
#include "bulbit/integrators.h"
#include "bulbit/microfacet.h"
#include "bulbit/parallel_for.h"
#include "bulbit/progresses.h"
#include "bulbit/sampler.h"
#include "bulbit/visibility.h"

namespace bulbit
{

PhotonMappingIntegrator::PhotonMappingIntegrator(
    const Intersectable* accel,
    std::vector<Light*> lights,
    const Sampler* sampler,
    int32 max_bounces,
    int32 n_photons,
    Float radius,
    bool sample_direct_light
)
    : Integrator(accel, std::move(lights), std::make_unique<PowerLightSampler>())
    , sampler_prototype{ sampler }
    , max_bounces{ max_bounces }
    , n_photons{ n_photons }
    , gather_radius{ radius }
    , sample_direct_light{ sample_direct_light }
{
    if (gather_radius <= 0)
    {
        AABB world_bounds = accel->GetAABB();
        Point3 world_center;
        Float world_radius;
        world_bounds.ComputeBoundingSphere(&world_center, &world_radius);

        gather_radius = 2 * world_radius * 5e-4f;
    }
}

void PhotonMappingIntegrator::EmitPhotons(MultiPhaseRendering* progress, WavelengthSample lambda, int32 phase_index)
{
    const int32 min_bounces = 2;

    ThreadLocal<std::vector<Photon>> tl_photons;

    ParallelFor(0, n_photons, [&](int32 i) {
        RNG rng(Hash(n_photons, gather_radius), Hash(i));

        std::vector<Photon>& ps = tl_photons.Get();

        SampledLight sampled_light;
        if (!light_sampler->Sample(&sampled_light, Intersection{}, rng.NextFloat()))
        {
            return;
        }

        Point2 u0(rng.NextFloat(), rng.NextFloat());
        Point2 u1(rng.NextFloat(), rng.NextFloat());

        LightSampleLe light_sample;
        if (!sampled_light.light->Sample_Le(&light_sample, u0, u1, lambda))
        {
            return;
        }

        Ray photon_ray = light_sample.ray;
        SpectrumSample beta = light_sample.Le / (sampled_light.pmf * light_sample.pdf_p * light_sample.pdf_w);
        if (light_sample.normal != Vec3::zero)
        {
            beta *= AbsDot(light_sample.normal, photon_ray.d);
        }

        int32 bounce = 0;
        Float eta_scale = 1;

        while (true)
        {
            Vec3 wo = Normalize(-photon_ray.d);

            Intersection isect;
            if (!Intersect(&isect, photon_ray, Ray::epsilon, infinity))
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
                photon_ray.o = isect.point;
                --bounce;
                continue;
            }

            if (IsNonSpecular(bsdf.Flags()) && (!sample_direct_light || (bounce > 1)))
            {
                Photon p;
                p.primitive = isect.primitive;
                p.p = isect.point;
                p.normal = isect.normal;
                p.wi = wo;
                p.beta = beta;
                p.wavelength_collapsed = lambda.IsCollapsed();
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

            photon_ray = Ray(isect.point, bsdf_sample.wi);
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

    tl_photons.ForEach([&](std::thread::id tid, std::vector<Photon>& ps) {
        BulbitNotUsed(tid);
        photons.insert(photons.end(), ps.begin(), ps.end());
    });

    photon_map.Build(photons, gather_radius);
}

SpectrumSample PhotonMappingIntegrator::SampleDirectLight(
    const Vec3& wo,
    const Intersection& isect,
    const BSDF* bsdf,
    const WavelengthSample& lambda,
    Sampler& sampler,
    const SpectrumSample& beta
) const
{
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

    if (!V(this, isect.point, light_sample.point))
    {
        return SpectrumSample(0);
    }

    Float pdf = light_sample.pdf * sampled_light.pmf;
    return beta * light_sample.Li * bsdf->f(wo, light_sample.wi) * AbsDot(isect.shading.normal, light_sample.wi) / pdf;
}

Vec3 PhotonMappingIntegrator::Li(const Ray& primary_ray, WavelengthSample& lambda, Sampler& sampler) const
{
    int32 bounce = 0;
    Vec3 L(0);
    SpectrumSample beta(1);

    Ray ray = primary_ray;
    bool specular_bounce = false;

    while (true)
    {
        Intersection isect;
        if (!Intersect(&isect, ray, Ray::epsilon, infinity))
        {
            if (bounce == 0 || specular_bounce)
            {
                for (Light* light : infinite_lights)
                {
                    L += spectral::ToXYZ(beta * light->Le(ray, lambda), lambda);
                }
            }

            break;
        }

        Vec3 wo = Normalize(-ray.d);

        if (const Light* area_light = GetAreaLight(isect); area_light)
        {
            SpectrumSample Le = area_light->Le(isect, wo, lambda);
            if (!Le.IsBlack() && (bounce == 0 || specular_bounce))
            {
                L += spectral::ToXYZ(beta * Le, lambda);
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
            ray.o = isect.point;
            --bounce;
            continue;
        }

        if (IsNonSpecular(bsdf.Flags()))
        {
            if (sample_direct_light)
            {
                L += spectral::ToXYZ(SampleDirectLight(wo, isect, &bsdf, lambda, sampler, beta), lambda);
            }

            Vec3 Li(0);
            photon_map.Query<Photon>(photons, isect.point, gather_radius, [&](const Photon& p) {
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
                if (p.wavelength_collapsed)
                {
                    photon_lambda.CollapseToPrimary();
                }
                Li += spectral::ToXYZ(contribution, photon_lambda);
            });

            Li *= 1 / (pi * Sqr(gather_radius) * n_photons);
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
        beta *= bsdf_sample.f * AbsDot(isect.shading.normal, bsdf_sample.wi) / bsdf_sample.pdf;
    }

    return L;
}

void PhotonMappingIntegrator::GatherPhotons(
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

                Vec3 L = Li(primary_ray.ray, lambda, *sampler);
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

Rendering* PhotonMappingIntegrator::Render(Allocator& alloc, const Camera* camera)
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
            WavelengthSample lambda = WavelengthSample::SampleIteration(iteration, n_iterations);
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
