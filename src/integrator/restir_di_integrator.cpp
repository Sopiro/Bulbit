#include "bulbit/async_job.h"
#include "bulbit/bsdf.h"
#include "bulbit/bxdfs.h"
#include "bulbit/camera.h"
#include "bulbit/hash.h"
#include "bulbit/integrators.h"
#include "bulbit/lights.h"
#include "bulbit/parallel_for.h"
#include "bulbit/progresses.h"
#include "bulbit/sampler.h"
#include "bulbit/visibility.h"

namespace bulbit
{

struct ReSTIRDIVisiblePoint
{
    Intersection isect;
    Vec3 wo;

    Spectrum Le;
};

struct ReSTIRDISample
{
    LightSampleLi x;
    Float p_hat = 0; // p_hat(y): target function value of selected sample
    Float W = 0;
};

class ReSTIRDIReservoir
{
public:
    ReSTIRDIReservoir(uint64 seed = 0)
        : w{ 0 }
        , w_sum{ 0 }
        , rng(seed)
    {
    }

    void Seed(uint64 seed)
    {
        rng.Seed(seed);
    }

    bool Add(const ReSTIRDISample& sample, Float weight)
    {
        w_sum += weight;

        if (rng.NextFloat() < weight / w_sum)
        {
            y = sample;
            w = weight;
            return true;
        }

        return false;
    }

    bool HasSample() const
    {
        return w_sum > 0;
    }

    Float GetSampleProbability() const
    {
        return w / w_sum;
    }

    void Reset()
    {
        w = 0;
        w_sum = 0;
    }

    ReSTIRDISample y{};
    Float w;
    Float w_sum;

private:
    RNG rng;
};

ReSTIRDIIntegrator::ReSTIRDIIntegrator(const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler)
    : Integrator(accel, std::move(lights), std::make_unique<PowerLightSampler>())
    , sampler_prototype{ sampler }
{
}

Rendering* ReSTIRDIIntegrator::Render(Allocator& alloc, const Camera* camera)
{
    Point2i resolution = camera->GetScreenResolution();

    const int32 spp = sampler_prototype->samples_per_pixel;
    const int32 tile_size = 16;

    const int32 num_pixels = resolution.x * resolution.y;
    const Point2i num_tiles = (resolution + (tile_size - 1)) / tile_size;
    int32 tile_count = num_tiles.x * num_tiles.y;

    const int32 num_passes = 3;

    size_t total_works = size_t(std::max(spp, 1) * tile_count * num_passes);

    SinglePhaseRendering* progress = alloc.new_object<SinglePhaseRendering>(camera, total_works);
    progress->job = RunAsync([=, this]() {
        std::vector<ReSTIRDIVisiblePoint> visible_points(num_pixels);
        std::vector<ReSTIRDISample> samples(num_pixels);
        std::vector<ReSTIRDIReservoir> reservoirs(num_pixels);

        const int32 sample = 0;

        // Trace primary rays and generate initial sample
        ParallelFor2D(
            resolution,
            [&](AABB2i tile) {
                int8 sample_mem[64];
                BufferResource sampler_buffer(sample_mem, sizeof(sample_mem));
                Allocator sampler_alloc(&sampler_buffer);
                Sampler* sampler = sampler_prototype->Clone(sampler_alloc);

                int8 bxdf_mem[max_bxdf_size];
                BufferResource bsdf_buffer(bxdf_mem, sizeof(bxdf_mem));
                Allocator bsdf_alloc(&bsdf_buffer);

                for (Point2i pixel : tile)
                {
                    sampler->StartPixelSample(pixel, sample);
                    const int32 index = resolution.x * pixel.y + pixel.x;

                    PrimaryRay primary_ray;
                    camera->SampleRay(&primary_ray, pixel, sampler->Next2D(), sampler->Next2D());

                    Ray ray = primary_ray.ray;

                    ReSTIRDIVisiblePoint& vp = visible_points[index];
                    Intersection& isect = vp.isect;

                    vp.wo = Normalize(-ray.d);

                    bool found_intersection = false;
                    while (true)
                    {
                        // Find visible point
                        found_intersection = Intersect(&isect, ray, Ray::epsilon, infinity);
                        if (!found_intersection)
                        {
                            for (Light* light : infinite_lights)
                            {
                                vp.Le += light->Le(ray);
                            }
                            isect.primitive = nullptr;
                            break;
                        }

                        bsdf_buffer.release();
                        BSDF bsdf;
                        if (!isect.GetBSDF(&bsdf, vp.wo, bsdf_alloc))
                        {
                            ray.o = isect.point;
                            continue;
                        }

                        break;
                    }

                    if (!found_intersection)
                    {
                        continue;
                    }

                    bsdf_buffer.release();
                    BSDF bsdf;
                    if (!isect.GetBSDF(&bsdf, vp.wo, bsdf_alloc))
                    {
                        continue;
                    }

                    ReSTIRDIReservoir& reservoir = reservoirs[index];
                    reservoir.Reset();
                    reservoir.Seed(Hash(pixel, sample));

                    // RIS: draw M initial candidates, keep one by WRS, then compute W(y)
                    const int32 M = 64;
                    const Float mis_weight = 1.0f / M;

                    for (int32 i = 0; i < M; ++i)
                    {
                        ReSTIRDISample candidate{};
                        Float w = 0;

                        SampledLight sampled_light;
                        if (!light_sampler->Sample(&sampled_light, isect, sampler->Next1D()))
                        {
                            continue;
                        }

                        LightSampleLi light_sample;
                        if (!sampled_light.light->Sample_Li(&light_sample, isect, sampler->Next2D()))
                        {
                            continue;
                        }

                        Float p = sampled_light.pmf * light_sample.pdf;
                        if (p > 0 && !light_sample.Li.IsBlack())
                        {
                            Spectrum f_cos = bsdf.f(vp.wo, light_sample.wi) * AbsDot(isect.shading.normal, light_sample.wi);
                            Float p_hat = (light_sample.Li * f_cos).Luminance();
                            if (p_hat > 0)
                            {
                                candidate.x = light_sample;
                                candidate.p_hat = p_hat;
                                w = mis_weight * p_hat / p;
                            }
                        }

                        reservoir.Add(candidate, w);
                    }

                    ReSTIRDISample& sample = samples[index];
                    if (reservoir.HasSample())
                    {
                        sample = reservoir.y;

                        if (sample.p_hat > 0)
                        {
                            sample.W = (1 / sample.p_hat) * reservoir.w_sum;
                        }
                    }
                }

                progress->work_dones.fetch_add(1, std::memory_order_relaxed);
            },
            tile_size
        );

        // Visibility pass
        ParallelFor2D(
            resolution,
            [&](AABB2i tile) {
                for (Point2i pixel : tile)
                {
                    const int32 index = resolution.x * pixel.y + pixel.x;
                    ReSTIRDIVisiblePoint& vp = visible_points[index];
                    Intersection& isect = vp.isect;
                    ReSTIRDISample& sample = samples[index];
                    if (!isect.primitive)
                    {
                        continue;
                    }

                    if (!V(this, isect.point, sample.x.point))
                    {
                        sample.W = 0;
                    }
                }

                progress->work_dones.fetch_add(1, std::memory_order_relaxed);
            },
            tile_size
        );

        // Shade
        ParallelFor2D(
            resolution,
            [&](AABB2i tile) {
                for (Point2i pixel : tile)
                {
                    const int32 index = resolution.x * pixel.y + pixel.x;
                    ReSTIRDIVisiblePoint& vp = visible_points[index];
                    Intersection& isect = vp.isect;

                    if (!isect.primitive)
                    {
                        progress->film.AddSample(pixel, vp.Le);
                        continue;
                    }
                    ReSTIRDISample& sample = samples[index];

                    int8 bxdf_mem[max_bxdf_size];
                    BufferResource bsdf_buffer(bxdf_mem, sizeof(bxdf_mem));
                    Allocator bsdf_alloc(&bsdf_buffer);
                    BSDF bsdf;
                    if (!isect.GetBSDF(&bsdf, vp.wo, bsdf_alloc))
                    {
                        BulbitAssert(false);
                    }

                    Spectrum f_cos = bsdf.f(vp.wo, sample.x.wi) * AbsDot(isect.shading.normal, sample.x.wi);
                    Spectrum L = sample.x.Li * f_cos * sample.W;
                    if (!L.IsBlack())
                    {
                        progress->film.AddSample(pixel, L);
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

} // namespace bulbit
