#include "bulbit/integrators.h"

#include "bulbit/async_job.h"
#include "bulbit/camera.h"
#include "bulbit/film.h"
#include "bulbit/media.h"
#include "bulbit/microfacet.h"
#include "bulbit/parallel_for.h"
#include "bulbit/progress.h"
#include "bulbit/sampler.h"

namespace bulbit
{

Integrator::Integrator(const Intersectable* accel, std::vector<Light*> lights)
    : accel{ accel }
    , all_lights{ std::move(lights) }
{
    AABB world_bounds = accel->GetAABB();
    for (size_t i = 0; i < all_lights.size(); i++)
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
        case Light::TypeIndexOf<AreaLight>():
        {
            AreaLight* area_light = light->Cast<AreaLight>();
            area_lights.emplace(area_light->GetPrimitive(), area_light);
        }
        break;
        default:
            break;
        }
    }

    light_sampler.Init(all_lights);
}

UniDirectionalRayIntegrator::UniDirectionalRayIntegrator(
    const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler
)
    : Integrator(accel, std::move(lights))
    , sampler_prototype{ sampler }
{
}

std::unique_ptr<Rendering> UniDirectionalRayIntegrator::Render(const Camera* camera)
{
    ComoputeReflectanceTextures();

    Point2i resolution = camera->GetScreenResolution();

    const int32 spp = sampler_prototype->samples_per_pixel;
    const int32 tile_size = 16;

    std::unique_ptr<Rendering> progress = std::make_unique<Rendering>(camera, tile_size);

    progress->job = RunAsync([=, this, &progress]() {
        ParallelFor2D(
            resolution,
            [&](AABB2i tile) {
                // Thread local sampler for current tile
                std::unique_ptr<Sampler> sampler = sampler_prototype->Clone();

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

                progress->tile_done++;
            },
            tile_size
        );

        progress->done = true;
        return true;
    });

    return progress;
}

BiDirectionalRayIntegrator::BiDirectionalRayIntegrator(
    const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler
)
    : Integrator(accel, std::move(lights))
    , sampler_prototype{ sampler }
{
}

std::unique_ptr<Rendering> BiDirectionalRayIntegrator::Render(const Camera* camera)
{
    ComoputeReflectanceTextures();

    Point2i resolution = camera->GetScreenResolution();

    const int32 spp = sampler_prototype->samples_per_pixel;
    const int32 tile_size = 16;

    std::unique_ptr<Rendering> progress = std::make_unique<Rendering>(camera, tile_size);

    progress->job = RunAsync([=, this, &progress]() {
        ParallelFor2D(
            resolution,
            [&](AABB2i tile) {
                // Thread local sampler for current tile
                std::unique_ptr<Sampler> sampler = sampler_prototype->Clone();

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

                progress->tile_done++;
            },
            tile_size
        );

        progress->film.WeightSplats(1.0f / spp);
        progress->done = true;
        return true;
    });

    return progress;
}

} // namespace bulbit
