#include "bulbit/async_job.h"
#include "bulbit/bsdf.h"
#include "bulbit/bxdfs.h"
#include "bulbit/camera.h"
#include "bulbit/film.h"
#include "bulbit/integrators.h"
#include "bulbit/parallel_for.h"
#include "bulbit/primitive.h"
#include "bulbit/progresses.h"
#include "bulbit/sampler.h"
#include "bulbit/visibility.h"

namespace bulbit
{

struct VCMSubPathState
{
    Point3 origin;
    Vec3 direction;
    Spectrum beta;

    int32 path_length : 30 = 1;
    int32 is_finite_light : 1 = false;
    int32 specular_path : 1 = true;

    Float eta_scale = 1;

    // Cache variables for efficient subpath MIS weight evaluation
    // See below for details..
    // http://www.iliyan.com/publications/ImplementingVCM
    Float d_vcm = 0; // MIS cache used for vertex connection and merging
    Float d_vc = 0;  // MIS cache used for vertex connection
    Float d_vm = 0;  // MIS cache used for vertex merging
};

struct VCMLightVertex
{
    Point3 p;
    Vec3 wo;
    Vec3 normal;
    Vec3 shading_normal;

    Spectrum beta;
    BSDF* bsdf;

    int32 path_length = 0;
    Float d_vcm = 0;
    Float d_vc = 0;
    Float d_vm = 0;
    Float cont_prob = 1;
};

VCMIntegrator::VCMIntegrator(
    const Intersectable* accel,
    std::vector<Light*> lights,
    const Sampler* sampler,
    int32 max_bounces,
    int32 rr_min_bounces,
    Float merge_radius,
    Float radius_alpha
)
    : Integrator(accel, std::move(lights), std::make_unique<PowerLightSampler>())
    , sampler_prototype{ sampler }
    , max_bounces{ max_bounces }
    , rr_min_bounces{ rr_min_bounces }
    , initial_radius{ merge_radius }
    , radius_alpha{ radius_alpha }
{
    if (initial_radius <= 0)
    {
        AABB world_bounds = accel->GetAABB();
        Point3 world_center;
        Float world_radius;
        world_bounds.ComputeBoundingSphere(&world_center, &world_radius);

        initial_radius = 2 * world_radius * 3e-3f;
    }
}

Rendering* VCMIntegrator::Render(Allocator& alloc, const Camera* camera)
{
    const int32 tile_size = 16;
    const int32 n_iterations = std::max<int32>(1, sampler_prototype->samples_per_pixel);

    Point2i res = camera->GetScreenResolution();
    const int32 path_count = res.x * res.y;

    Point2i num_tiles = (res + (tile_size - 1)) / tile_size;
    const int32 tile_count = num_tiles.x * num_tiles.y;

    std::vector<size_t> phase_works(2 * size_t(n_iterations));
    for (int32 i = 0; i < n_iterations; ++i)
    {
        phase_works[2 * i] = size_t(path_count);
        phase_works[2 * i + 1] = size_t(tile_count);
    }

    MultiPhaseRendering* progress = alloc.new_object<MultiPhaseRendering>(camera, phase_works);
    progress->job = RunAsync([]() { return true; });

    return progress;
}

} // namespace bulbit