#pragma once

#include "bounding_box.h"

namespace bulbit
{

template <typename T>
struct VoxelGrid
{
    VoxelGrid() = default;
    VoxelGrid(const AABB3& bounds, const Point3i& res)
        : bounds{ bounds }
        , res{ res }
        , voxels(res.x * res.y * res.z)
    {
    }

    T& operator()(int32 x, int32 y, int32 z)
    {
        BulbitAssert(x >= 0 && x < res.x && y >= 0 && y < res.y && z >= 0 && z < res.z);
        return voxels[x + res.x * (y + res.y * z)];
    }

    const T& operator()(int32 x, int32 y, int32 z) const
    {
        BulbitAssert(x >= 0 && x < res.x && y >= 0 && y < res.y && z >= 0 && z < res.z);
        return voxels[x + res.x * (y + res.y * z)];
    }

    const T& LookUp(int32 x, int32 y, int32 z) const
    {
        BulbitAssert(x >= 0 && x < res.x && y >= 0 && y < res.y && z >= 0 && z < res.z);
        return voxels[x + res.x * (y + res.y * z)];
    }

    AABB3 VoxelBounds(int32 x, int32 y, int32 z) const
    {
        BulbitAssert(x >= 0 && x < res.x && y >= 0 && y < res.y && z >= 0 && z < res.z);
        Point3 p0(Float(x) / res.x, Float(y) / res.y, Float(z) / res.z);
        Point3 p1(Float(x + 1) / res.x, Float(y + 1) / res.y, Float(z + 1) / res.z);
        return AABB3(p0, p1);
    }

    AABB3 bounds;
    Point3i res;
    std::vector<T> voxels;
};

} // namespace bulbit
