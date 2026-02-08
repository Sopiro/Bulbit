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
    Float w;
};

class ReSTIRDIReservoir
{
public:
    ReSTIRDIReservoir(uint64 seed = 0)
        : rng(seed)
        , w{ 0 }
        , w_sum{ 0 }
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
            s = sample;
            w = weight;
            return true;
        }

        return false;
    }

    bool HasSample() const
    {
        return w_sum > 0;
    }

    const ReSTIRDISample& GetSample() const
    {
        return s;
    }

    Float GetSampleProbability() const
    {
        return w / w_sum;
    }

    Float GetWeightSum() const
    {
        return w_sum;
    }

    void Reset()
    {
        w = 0;
        w_sum = 0;
    }

private:
    RNG rng;

    ReSTIRDISample s{};
    Float w;
    Float w_sum;
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

    size_t total_works = size_t(std::max(spp, 1)) * size_t(tile_count);

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
                int8 mem[64];
                BufferResource sampler_resource(mem, sizeof(mem));
                Allocator sampler_alloc(&sampler_resource);
                Sampler* sampler = sampler_prototype->Clone(sampler_alloc);

                for (Point2i pixel : tile)
                {
                    sampler->StartPixelSample(pixel, sample);
                    const int32 index = resolution.x * pixel.y + pixel.x;

                    PrimaryRay primary_ray;
                    camera->SampleRay(&primary_ray, pixel, sampler->Next2D(), sampler->Next2D());

                    Ray ray = primary_ray.ray;

                    ReSTIRDIVisiblePoint& vp = visible_points[index];
                    Intersection& isect = vp.isect;
                    vp.wo = -ray.d;

                    bool found_intersection = false;
                    while (true)
                    {
                        // Find visible point
                        found_intersection = Intersect(&isect, primary_ray.ray, Ray::epsilon, infinity);
                        if (!found_intersection)
                        {
                            for (Light* light : infinite_lights)
                            {
                                vp.Le += light->Le(primary_ray.ray);
                            }
                            isect.primitive = nullptr;
                            break;
                        }

                        int8 mem[max_bxdf_size];
                        BufferResource res(mem, sizeof(mem));
                        Allocator alloc(&res);
                        BSDF bsdf;
                        if (!isect.GetBSDF(&bsdf, ray.d, alloc))
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

                    // Sample canonical light sample

                    ReSTIRDISample& sample = samples[index];
                    sample.w = 0;

                    Float u0 = sampler->Next1D();
                    Point2 u12 = sampler->Next2D();
                    SampledLight sampled_light;
                    if (!light_sampler->Sample(&sampled_light, isect, u0))
                    {
                        continue;
                    }

                    if (!sampled_light.light->Sample_Li(&sample.x, isect, u12))
                    {
                        continue;
                    }

                    sample.w = 1 / (sample.x.pdf * sampled_light.pmf);
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

                    int8 mem[max_bxdf_size];
                    BufferResource res(mem, sizeof(mem));
                    Allocator alloc(&res);
                    BSDF bsdf;
                    if (!isect.GetBSDF(&bsdf, vp.wo, alloc))
                    {
                        BulbitAssert(false);
                    }

                    Spectrum f_cos = bsdf.f(vp.wo, sample.x.wi) * AbsDot(isect.shading.normal, sample.x.wi);
                    Spectrum L = sample.x.Li * f_cos * sample.w;
                    if (!L.IsBlack() && V(this, isect.point, sample.x.point))
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
