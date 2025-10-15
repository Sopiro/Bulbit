#include "bulbit/integrators.h"
#include "bulbit/renderer_info.h"

#include "bulbit/async_job.h"
#include "bulbit/camera.h"
#include "bulbit/film.h"
#include "bulbit/media.h"
#include "bulbit/microfacet.h"
#include "bulbit/parallel_for.h"
#include "bulbit/progresses.h"
#include "bulbit/sampler.h"

namespace bulbit
{

Integrator* Integrator::Create(
    Allocator& alloc,
    const IntegratorInfo& ii,
    const Intersectable* accel,
    const std::vector<Light*>& lights,
    const Sampler* sampler
)
{
    int32 max_bounces = ii.max_bounces;
    int32 rr_min_bounces = ii.rr_min_bounces;

    switch (ii.type)
    {
    case IntegratorType::path:
        return alloc.new_object<PathIntegrator>(accel, lights, sampler, max_bounces, rr_min_bounces, ii.regularize_bsdf);

    case IntegratorType::vol_path:
        return alloc.new_object<VolPathIntegrator>(accel, lights, sampler, max_bounces, rr_min_bounces, ii.regularize_bsdf);

    case IntegratorType::light_path:
        return alloc.new_object<LightPathIntegrator>(accel, lights, sampler, max_bounces, rr_min_bounces);

    case IntegratorType::light_vol_path:
        return alloc.new_object<LightVolPathIntegrator>(accel, lights, sampler, max_bounces, rr_min_bounces);

    case IntegratorType::bdpt:
        return alloc.new_object<BiDirectionalPathIntegrator>(accel, lights, sampler, max_bounces, rr_min_bounces);

    case IntegratorType::vol_bdpt:
        return alloc.new_object<BiDirectionalVolPathIntegrator>(accel, lights, sampler, max_bounces, rr_min_bounces);

    case IntegratorType::pm:
        return alloc.new_object<PhotonMappingIntegrator>(
            accel, lights, sampler, max_bounces, ii.n_photons, ii.initial_radius_surface, ii.sample_direct_light
        );

    case IntegratorType::vol_pm:
        return alloc.new_object<VolPhotonMappingIntegrator>(
            accel, lights, sampler, max_bounces, ii.n_photons, ii.initial_radius_surface, ii.initial_radius_volume,
            ii.sample_direct_light
        );

    case IntegratorType::sppm:
        return alloc.new_object<SPPMIntegrator>(
            accel, lights, sampler, max_bounces, ii.n_photons, ii.initial_radius_surface, ii.sample_direct_light
        );

    case IntegratorType::vol_sppm:
        return alloc.new_object<VolSPPMIntegrator>(
            accel, lights, sampler, max_bounces, ii.n_photons, ii.initial_radius_surface, ii.initial_radius_volume,
            ii.sample_direct_light
        );

    case IntegratorType::naive_path:
        return alloc.new_object<NaivePathIntegrator>(accel, lights, sampler, max_bounces, rr_min_bounces);

    case IntegratorType::naive_vol_path:
        return alloc.new_object<NaiveVolPathIntegrator>(accel, lights, sampler, max_bounces, rr_min_bounces);

    case IntegratorType::random_walk:
        return alloc.new_object<RandomWalkIntegrator>(accel, lights, sampler, max_bounces);

    case IntegratorType::ao:
        return alloc.new_object<AOIntegrator>(accel, lights, sampler, ii.ao_range);

    case IntegratorType::albedo:
        return alloc.new_object<AlbedoIntegrator>(accel, lights, sampler);

    case IntegratorType::debug:
        return alloc.new_object<DebugIntegrator>(accel, lights, sampler);

    default:
        return nullptr;
    }
}

Integrator::Integrator(const Intersectable* accel, std::vector<Light*> lights, std::unique_ptr<LightSampler> l_sampler)
    : accel{ accel }
    , all_lights{ std::move(lights) }
    , light_sampler{ std::move(l_sampler) }
{
    AABB world_bounds = accel->GetAABB();
    for (size_t i = 0; i < all_lights.size(); ++i)
    {
        Light* light = all_lights[i];
        light->Preprocess(world_bounds);

        switch (light->type_index)
        {
        case Light::TypeIndexOf<UniformInfiniteLight>():
        case Light::TypeIndexOf<ImageInfiniteLight>():
        {
            infinite_lights.push_back(light);
        }
        break;
        case Light::TypeIndexOf<DiffuseAreaLight>():
        {
            DiffuseAreaLight* area_light = light->Cast<DiffuseAreaLight>();
            area_lights.emplace(area_light->primitive, area_light);
        }
        break;
        case Light::TypeIndexOf<SpotAreaLight>():
        {
            SpotAreaLight* area_light = light->Cast<SpotAreaLight>();
            area_lights.emplace(area_light->primitive, area_light);
        }
        // We don't need to add directional area light because it's delta light
        break;
        default:
            break;
        }
    }

    if (light_sampler)
    {
        light_sampler->Init(all_lights);
    }
}

UniDirectionalRayIntegrator::UniDirectionalRayIntegrator(
    const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler, std::unique_ptr<LightSampler> light_sampler
)
    : Integrator(accel, std::move(lights), std::move(light_sampler))
    , sampler_prototype{ sampler }
{
}

Rendering* UniDirectionalRayIntegrator::Render(Allocator& alloc, const Camera* camera)
{
    ComputeReflectanceTextures();

    Point2i resolution = camera->GetScreenResolution();

    const int32 spp = sampler_prototype->samples_per_pixel;
    const int32 tile_size = 16;

    Point2i num_tiles = (resolution + (tile_size - 1)) / tile_size;
    int32 tile_count = num_tiles.x * num_tiles.y;

    SinglePhaseRendering* progress = alloc.new_object<SinglePhaseRendering>(camera, size_t(tile_count));
    progress->job = RunAsync([=, this]() {
        ParallelFor2D(
            resolution,
            [&](AABB2i tile) {
                // Thread local sampler for current tile
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

                progress->work_dones.fetch_add(1, std::memory_order_relaxed);
            },
            tile_size
        );

        progress->done.store(true, std::memory_order_release);
        return true;
    });

    return progress;
}

BiDirectionalRayIntegrator::BiDirectionalRayIntegrator(
    const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler, std::unique_ptr<LightSampler> light_sampler
)
    : Integrator(accel, std::move(lights), std::move(light_sampler))
    , sampler_prototype{ sampler }
{
}

Rendering* BiDirectionalRayIntegrator::Render(Allocator& alloc, const Camera* camera)
{
    ComputeReflectanceTextures();

    Point2i resolution = camera->GetScreenResolution();

    const int32 spp = sampler_prototype->samples_per_pixel;
    const int32 tile_size = 16;

    Point2i num_tiles = (resolution + (tile_size - 1)) / tile_size;
    int32 tile_count = num_tiles.x * num_tiles.y;

    SinglePhaseRendering* progress = alloc.new_object<SinglePhaseRendering>(camera, size_t(tile_count));
    progress->job = RunAsync([=, this]() {
        ParallelFor2D(
            resolution,
            [&](AABB2i tile) {
                // Thread local sampler for current tile
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

                        Spectrum Li = L(primary_ray.ray, camera->GetMedium(), camera, progress->film, *sampler);
                        if (!Li.IsNullish())
                        {
                            progress->film.AddSample(pixel, primary_ray.weight * Li);
                        }
                    }
                }

                progress->work_dones.fetch_add(1, std::memory_order_relaxed);
            },
            tile_size
        );

        progress->film.WeightSplats(1.0f / spp);
        progress->done.store(true, std::memory_order_release);
        return true;
    });

    return progress;
}

} // namespace bulbit
