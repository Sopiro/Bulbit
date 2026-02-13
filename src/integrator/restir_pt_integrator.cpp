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
#include "bulbit/sampling.h"

namespace bulbit
{

struct ReSTIRPTVisiblePoint
{
    Float primary_weight;

    Intersection isect;
    BSDF bsdf;
    Vec3 wo;

    Spectrum Le;

    ReSTIRPTVisiblePoint()
        : bsdf_buffer(&bxdf_mem, sizeof(bxdf_mem))
    {
    }

    int8 bxdf_mem[max_bxdf_size];
    BufferResource bsdf_buffer;
};

struct ReSTIRPTSample
{
    Spectrum contribution;

    uint64 base_rng_seed;
    int32 path_length;
    int32 reconnection_vertex; // first index of consecutive diffuse pair

    Float p_hat = 0;           // luminance of selected path contribution
    Float W = 0;               // UCW
};

class ReSTIRPTReservoir
{
public:
    ReSTIRPTReservoir(uint64 seed = 0)
        : w{ 0 }
        , w_sum{ 0 }
        , M{ 0 }
        , rng(seed)
    {
    }

    void Seed(uint64 seed)
    {
        rng.Seed(seed);
    }

    bool Add(const ReSTIRPTSample& sample, Float weight)
    {
        if (weight <= 0)
        {
            return false;
        }

        ++M;
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

    void Reset()
    {
        w = 0;
        w_sum = 0;
        M = 0;
    }

    ReSTIRPTSample y{};
    Float w;
    Float w_sum;
    Float M;

private:
    RNG rng;
};

ReSTIRPTIntegrator::ReSTIRPTIntegrator(
    const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler, int32 max_bounces, int32 rr_min_bounces
)
    : Integrator(accel, std::move(lights), std::make_unique<PowerLightSampler>())
    , sampler_prototype{ sampler }
    , max_bounces{ max_bounces }
    , rr_min_bounces{ rr_min_bounces }
{
}

Rendering* ReSTIRPTIntegrator::Render(Allocator& alloc, const Camera* camera)
{
    Point2i resolution = camera->GetScreenResolution();

    const int32 spp = sampler_prototype->samples_per_pixel;
    const int32 tile_size = 16;

    const int32 num_pixels = resolution.x * resolution.y;
    const Point2i num_tiles = (resolution + (tile_size - 1)) / tile_size;
    const int32 tile_count = num_tiles.x * num_tiles.y;
    const int32 num_passes = 2;
    const size_t total_works = size_t(std::max(spp, 1) * tile_count * num_passes);

    SinglePhaseRendering* progress = alloc.new_object<SinglePhaseRendering>(camera, total_works);
    progress->job = RunAsync([=, this]() {
        for (int32 s = 0; s < spp; ++s)
        {
            std::vector<ReSTIRPTVisiblePoint> visible_points(num_pixels);
            std::vector<ReSTIRPTReservoir> base_reservoirs(num_pixels);

            // Generate visible points and base path reservoirs using RIS in path space
            ParallelFor2D(
                resolution,
                [&](AABB2i tile) {
                    int8 sampler_mem[64];
                    BufferResource sampler_buffer(sampler_mem, sizeof(sampler_mem));
                    Allocator sampler_alloc(&sampler_buffer);
                    Sampler* sampler = sampler_prototype->Clone(sampler_alloc);

                    for (Point2i pixel : tile)
                    {
                        sampler->StartPixelSample(pixel, s);

                        const int32 index = resolution.x * pixel.y + pixel.x;
                        ReSTIRPTReservoir& reservoir = base_reservoirs[index];
                        reservoir.Seed(Hash(pixel, s, 123));

                        PrimaryRay primary_ray;
                        camera->SampleRay(&primary_ray, pixel, sampler->Next2D(), sampler->Next2D());

                        // Path tracer loop
                        while (true)
                        {
                        }

                        if (reservoir.HasSample())
                        {
                            reservoir.y.W = (1 / reservoir.y.p_hat) * reservoir.w_sum;
                        }
                        else
                        {
                            reservoir.y.W = 0;
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
                        const ReSTIRPTVisiblePoint& vp = visible_points[index];

                        Spectrum L = vp.Le;
                        if (!vp.isect.primitive)
                        {
                            progress->film.AddSample(pixel, vp.primary_weight * vp.Le);
                            continue;
                        }

                        const ReSTIRPTSample& sample = base_reservoirs[index].y;
                        if (sample.W > 0)
                        {
                            L += sample.contribution * sample.W;
                        }

                        progress->film.AddSample(pixel, vp.primary_weight * L);
                    }

                    progress->work_dones.fetch_add(1, std::memory_order_relaxed);
                },
                tile_size
            );
        }

        progress->done.store(true, std::memory_order_release);
        return true;
    });

    return progress;
}

} // namespace bulbit
