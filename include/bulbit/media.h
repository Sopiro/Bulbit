#pragma once

#include "allocator.h"

#include "medium.h"
#include "random.h"
#include "sampling.h"
#include "voxel_grid.h"

#define NANOVDB_USE_ZIP 1
#include <nanovdb/NanoVDB.h>
#include <nanovdb/util/IO.h>

namespace bulbit
{

class HenyeyGreensteinPhaseFunction : public PhaseFunction
{
public:
    HenyeyGreensteinPhaseFunction(Float g)
        : g(g)
    {
    }

    virtual Float p(Vec3 wo, Vec3 wi) const override;
    virtual Float PDF(Vec3 wo, Vec3 wi) const override;

    virtual bool Sample_p(PhaseFunctionSample* sample, Vec3 wo, Point2 u) const override;

private:
    Float g;
};

class HomogeneousMajorantIterator : public RayMajorantIterator
{
public:
    HomogeneousMajorantIterator()
        : called{ true }
    {
    }

    HomogeneousMajorantIterator(Float t_min, Float t_max, Spectrum sigma_maj)
        : segment{ t_min, t_max, sigma_maj }
        , called{ false }
    {
    }

    virtual bool Next(RayMajorantSegment* next_segment) override
    {
        if (called)
        {
            return false;
        }

        *next_segment = segment;

        called = true;
        return true;
    }

private:
    RayMajorantSegment segment;
    bool called;
};

class DDAMajorantIterator : public RayMajorantIterator
{
public:
    DDAMajorantIterator() = default;
    DDAMajorantIterator(Ray ray, Float t_min, Float t_max, const VoxelGrid<Float>* grid, Spectrum sigma_t)
        : sigma_t{ sigma_t }
        , t_min{ t_min }
        , t_max{ t_max }
        , grid{ grid }
    {
        Vec3 extents = grid->bounds.GetExtents();

        // Transform ray into local grid space [0,1]^3
        Point3 grid_origin = (ray.o - grid->bounds.min) / extents;
        Vec3 grid_direction = ray.d / extents;
        Ray ray_grid(grid_origin, grid_direction);

        Point3 grid_isect = ray_grid.At(t_min);
        for (int32 axis = 0; axis < 3; ++axis)
        {
            // Initialize ray stepping parameters for each axis
            // Compute current voxel for axis and handle negative zero direction
            const int32 resolution = grid->res[axis];

            current_voxel[axis] = int32(Clamp(grid_isect[axis] * resolution, 0, resolution - 1));
            delta_t[axis] = 1 / (std::abs(ray_grid.d[axis]) * resolution);

            if (ray_grid.d[axis] >= 0)
            {
                // Handle ray with positive direction for voxel stepping
                Float next_voxel_pos = Float(current_voxel[axis] + 1) / resolution;
                next_t[axis] = t_min + (next_voxel_pos - grid_isect[axis]) / ray_grid.d[axis];

                step[axis] = 1;
                voxel_end[axis] = resolution;
            }
            else
            {
                // Handle ray with negative direction for voxel stepping
                Float next_voxel_pos = Float(current_voxel[axis]) / resolution;
                next_t[axis] = t_min + (next_voxel_pos - grid_isect[axis]) / ray_grid.d[axis];

                step[axis] = -1;
                voxel_end[axis] = -1;
            }
        }
    }

    virtual bool Next(RayMajorantSegment* next_segment) override
    {
        if (t_min >= t_max)
        {
            return false;
        }

        // Find next step_axis for stepping to next voxel and exit point t_voxel_exit by hacky bit wise comparing
        int32 bits = ((next_t[0] < next_t[1]) << 2) + ((next_t[0] < next_t[2]) << 1) + ((next_t[1] < next_t[2]));
        const int32 compare_to_axis[8] = { 2, 1, 2, 1, 2, 2, 0, 0 };

        int32 step_axis = compare_to_axis[bits];
        Float t_voxel_exit = std::min(t_max, next_t[step_axis]);

        // Initialize next majorant segment
        next_segment->t_min = t_min;
        next_segment->t_max = t_voxel_exit;
        next_segment->sigma_maj = sigma_t * grid->LookUp(current_voxel[0], current_voxel[1], current_voxel[2]);

        // Advance to next voxel
        t_min = t_voxel_exit;
        if (next_t[step_axis] > t_max)
        {
            t_min = t_max;
        }
        current_voxel[step_axis] += step[step_axis];

        if (current_voxel[step_axis] == voxel_end[step_axis])
        {
            t_min = t_max;
        }
        next_t[step_axis] += delta_t[step_axis];

        return true;
    }

private:
    Spectrum sigma_t;
    Float t_min = infinity, t_max = -infinity;
    const VoxelGrid<Float>* grid;

    // State variables for DDA
    Float next_t[3], delta_t[3];
    int32 step[3], voxel_end[3], current_voxel[3];
};

class HomogeneousMedium : public Medium
{
public:
    using MajorantIterator = HomogeneousMajorantIterator;

    HomogeneousMedium(Spectrum sigma_a, Spectrum sigma_s, Spectrum Le, Float g);
    void Destroy() {}

    bool IsEmissive() const;
    MediumSample SamplePoint(Point3 p) const;
    HomogeneousMajorantIterator SampleRay(Ray ray, Float t_max) const;
    RayMajorantIterator* SampleRay(Ray ray, Float t_max, Allocator& alloc) const;

private:
    Spectrum sigma_a, sigma_s, Le;
    HenyeyGreensteinPhaseFunction phase;
};

class NanoVDBMedium : public Medium
{
public:
    using MajorantIterator = DDAMajorantIterator;

    NanoVDBMedium(
        const Transform& transform,
        Spectrum sigma_a,
        Spectrum sigma_s,
        Float sigma_scale,
        Float g,
        nanovdb::GridHandle<nanovdb::HostBuffer> density_grid,
        nanovdb::GridHandle<nanovdb::HostBuffer> temperature_grid = {},
        Float Le_scale = 1,
        Float temperature_offset = 0,
        Float temperature_scale = 1
    );
    void Destroy();

    bool IsEmissive() const;
    MediumSample SamplePoint(Point3 p) const;
    DDAMajorantIterator SampleRay(Ray ray, Float t_max) const;
    RayMajorantIterator* SampleRay(Ray ray, Float t_max, Allocator& alloc) const;

private:
    AABB3 bounds;
    Transform transform;

    Spectrum sigma_a, sigma_s;
    HenyeyGreensteinPhaseFunction phase;
    VoxelGrid<Float> majorant_grid;

    nanovdb::GridHandle<nanovdb::HostBuffer> density_grid;
    const nanovdb::FloatGrid* density_float_grid = nullptr;
    nanovdb::GridHandle<nanovdb::HostBuffer> temperature_grid;
    const nanovdb::FloatGrid* temperature_float_grid = nullptr;

    Float Le_scale;
    Float temperature_offset, temperature_scale;
};

inline Medium::~Medium()
{
    Dispatch([](auto medium) { return medium->Destroy(); });
}

inline bool Medium::IsEmissive() const
{
    return Dispatch([](auto medium) { return medium->IsEmissive(); });
}

inline MediumSample Medium::SamplePoint(Point3 p) const
{
    return Dispatch([&](auto medium) { return medium->SamplePoint(p); });
}

inline RayMajorantIterator* Medium::SampleRay(Ray ray, Float t_max, Allocator& alloc) const
{
    return Dispatch([&](auto medium) { return medium->SampleRay(ray, t_max, alloc); });
}

} // namespace bulbit
