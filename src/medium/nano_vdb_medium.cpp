#include "bulbit/media.h"
#include "bulbit/parallel_for.h"

#include <nanovdb/util/SampleFromVoxels.h>

namespace bulbit
{

NanoVDBMedium::NanoVDBMedium(
    const Transform& transform,
    Spectrum sigma_a,
    Spectrum sigma_s,
    Float sigma_scale,
    Float g,
    nanovdb::GridHandle<nanovdb::HostBuffer> dg
)
    : Medium(TypeIndexOf<NanoVDBMedium>())
    , transform{ transform }
    , sigma_a{ sigma_a * sigma_scale }
    , sigma_s{ sigma_s * sigma_scale }
    , phase{ g }
    , density_grid{ std::move(dg) }
    , density_float_grid{ density_grid.grid<float>() }
{
    nanovdb::BBox<nanovdb::Vec3R> aabb = density_float_grid->worldBBox();
    bounds = AABB3(
        Point3(Float(aabb.min()[0]), Float(aabb.min()[1]), Float(aabb.min()[2])),
        Point3(Float(aabb.max()[0]), Float(aabb.max()[1]), Float(aabb.max()[2]))
    );

    const Point3i res(64);
    majorant_grid = VoxelGrid<Float>(bounds, res);

    int32 grid_size = res.x * res.y * res.z;

    ParallelFor(0, grid_size, [&](int32 i) {
        int32 x = i % res.x;
        int32 y = (i / res.x) % res.y;
        int32 z = i / (res.x * res.y);

        Point3 t0(Float(x) / res.x, Float(y) / res.y, Float(z) / res.z);
        Point3 t1(Float(x + 1) / res.x, Float(y + 1) / res.y, Float(z + 1) / res.z);

        // World space sub bounds of current voxel
        AABB3 sub_bounds;
        sub_bounds.min = Lerp(bounds.min, bounds.max, t0);
        sub_bounds.max = Lerp(bounds.min, bounds.max, t1);

        // Compute corresponding NanoVDB index-space bounds in floating-point
        nanovdb::Vec3f i0 = density_float_grid->worldToIndexF<nanovdb::Vec3f>(
            nanovdb::Vec3f(sub_bounds.min.x, sub_bounds.min.y, sub_bounds.min.z)
        );
        nanovdb::Vec3f i1 = density_float_grid->worldToIndexF<nanovdb::Vec3f>(
            nanovdb::Vec3f(sub_bounds.max.x, sub_bounds.max.y, sub_bounds.max.z)
        );

        nanovdb::BBox<nanovdb::Coord> bbox = density_float_grid->indexBBox();

        // Find integer index-space bounds
        const Float delta = 1.0f; // Small margin to expand the sampling region slightly
        int nx0 = std::max(int(std::floor(i0[0] - delta)), bbox.min()[0]);
        int nx1 = std::min(int(std::ceil(i1[0] + delta)), bbox.max()[0]);
        int ny0 = std::max(int(std::floor(i0[1] - delta)), bbox.min()[1]);
        int ny1 = std::min(int(std::ceil(i1[1] + delta)), bbox.max()[1]);
        int nz0 = std::max(int(std::floor(i0[2] - delta)), bbox.min()[2]);
        int nz1 = std::min(int(std::ceil(i1[2] + delta)), bbox.max()[2]);

        float max_value = 0;
        auto accessor = density_float_grid->getAccessor();
        // Note nanovdb integer bounding boxes are inclusive on the upper end
        for (int nz = nz0; nz <= nz1; ++nz)
            for (int ny = ny0; ny <= ny1; ++ny)
                for (int nx = nx0; nx <= nx1; ++nx)
                    max_value = std::max(max_value, accessor.getValue({ nx, ny, nz }));

        majorant_grid(x, y, z) = max_value;
    });
}

bool NanoVDBMedium::IsEmissive() const
{
    return false;
}

MediumSample NanoVDBMedium::SamplePoint(Point3 p) const
{
    Point3 p_medium = MulT(transform, p);
    nanovdb::Vec3<float> pIndex = density_float_grid->worldToIndexF(nanovdb::Vec3<float>(p_medium.x, p_medium.y, p_medium.z));

    // Get medium density using trilinear sampler
    using Sampler = nanovdb::SampleFromVoxels<nanovdb::FloatGrid::TreeType, 1, false>;
    Float density = Sampler(density_float_grid->tree())(pIndex);

    return MediumSample{ sigma_a * density, sigma_s * density, Spectrum::black, &phase };
}

DDAMajorantIterator NanoVDBMedium::SampleRay(Ray ray, Float t_max) const
{
    Ray ray_medium = MulT(transform, ray);
    Float t_hit0, t_hit1;
    if (!bounds.Intersect(ray_medium, 0, t_max, &t_hit0, &t_hit1))
    {
        return {};
    }

    return DDAMajorantIterator(ray_medium, t_hit0, t_hit1, &majorant_grid, sigma_a + sigma_s);
}

RayMajorantIterator* NanoVDBMedium::SampleRay(Ray ray, Float t_max, Allocator& alloc) const
{
    Ray ray_medium = MulT(transform, ray);
    Float t_hit0, t_hit1;
    if (!bounds.Intersect(ray_medium, 0, t_max, &t_hit0, &t_hit1))
    {
        return nullptr;
    }

    return alloc.new_object<DDAMajorantIterator>(ray_medium, t_hit0, t_hit1, &majorant_grid, sigma_a + sigma_s);
}

} // namespace bulbit
