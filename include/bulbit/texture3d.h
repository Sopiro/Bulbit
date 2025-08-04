#pragma once

#include "texture.h"

namespace bulbit
{

template <typename T>
struct Texture3D
{
    virtual ~Texture3D() = default;

    virtual T Evaluate(const Point3& uvw) const = 0;
    virtual T Average() const = 0;
};

using FloatTexture3D = Texture3D<Float>;
using SpectrumTexture3D = Texture3D<Spectrum>;

inline void FilterTexCoord3D(int32* u, int32* v, int32* w, int32 dim_x, int32 dim_y, int32 dim_z, TexCoordFilter texcoord_filter)
{
    switch (texcoord_filter)
    {
    case TexCoordFilter::repeat:
    {
        *u = *u >= 0 ? *u % dim_x : dim_x - (-(*u) % dim_x) - 1;
        *v = *v >= 0 ? *v % dim_y : dim_y - (-(*v) % dim_y) - 1;
        *w = *w >= 0 ? *w % dim_z : dim_z - (-(*w) % dim_z) - 1;
    }
    break;
    case TexCoordFilter::clamp:
    {
        *u = Clamp(*u, 0, dim_x - 1);
        *v = Clamp(*v, 0, dim_y - 1);
        *w = Clamp(*w, 0, dim_z - 1);
    }
    break;
    default:
        BulbitAssert(false);
        break;
    }
}

} // namespace bulbit