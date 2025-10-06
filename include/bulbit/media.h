#pragma once

#include "medium.h"
#include "random.h"
#include "sampling.h"
#include "voxel_grid.h"

#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-braces"
#pragma clang diagnostic ignored "-Wunused-value"
#pragma clang diagnostic ignored "-Wpadded"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-braces"
#pragma GCC diagnostic ignored "-Wunused-value"
#pragma GCC diagnostic ignored "-Wpadded"
#elif defined(_MSVC_LANG)
#pragma warning(push)
#pragma warning(disable : 4324 4456 4459 4702)
#endif

#define NANOVDB_USE_ZIP 1
#include <nanovdb/NanoVDB.h>
#include <nanovdb/util/IO.h>

#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#elif defined(_MSVC_LANG)
#pragma warning(pop)
#endif

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

class HomogeneousMedium : public Medium
{
public:
    using MajorantIterator = HomogeneousMajorantIterator;

    HomogeneousMedium(Spectrum sigma_a, Spectrum sigma_s, Spectrum Le, Float g);

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
    using MajorantIterator = HomogeneousMajorantIterator;

    NanoVDBMedium(
        const Transform& transform,
        Spectrum sigma_a,
        Spectrum sigma_s,
        Float sigma_scale,
        Float g,
        nanovdb::GridHandle<nanovdb::HostBuffer> density_grid
    );

    bool IsEmissive() const;
    MediumSample SamplePoint(Point3 p) const;
    HomogeneousMajorantIterator SampleRay(Ray ray, Float t_max) const;
    RayMajorantIterator* SampleRay(Ray ray, Float t_max, Allocator& alloc) const;

private:
    AABB3 bounds;
    Transform transform;
    Spectrum sigma_a, sigma_s;
    HenyeyGreensteinPhaseFunction phase;
    VoxelGrid<Float> majorant_grid;
    nanovdb::GridHandle<nanovdb::HostBuffer> density_grid;
    const nanovdb::FloatGrid* density_float_grid = nullptr;
};

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
