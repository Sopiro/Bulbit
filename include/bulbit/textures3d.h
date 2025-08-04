#pragma once

#include "image3d.h"
#include "texture3d.h"

namespace bulbit
{

template <typename T>
class ImageTexture3D : public Texture3D<T>
{
public:
    ImageTexture3D(Image3D<T> image, TexCoordFilter texcoord_filter)
        : image{ std::move(image) }
        , texcoord_filter{ texcoord_filter }
    {
    }

    int32 GetWidth() const
    {
        return image.dim_x;
    }

    int32 GetHeight() const
    {
        return image.dim_y;
    }

    int32 GetDepth() const
    {
        return image.dim_z;
    }

    T Evaluate(const Point3& uvw) const
    {
#if 0
        // Nearest sampling
        Float w = uvw.x * image.dim_x + 0.5f;
        Float h = uvw.y * image.dim_y + 0.5f;
        Float d = uvw.z * image.dim_z + 0.5f;

        int32 i = int32(std::floor(w));
        int32 j = int32(std::floor(h));
        int32 k = int32(std::floor(d));

        FilterTexCoord3D(&i, &j, &k, image.dim_x, image.dim_y, image.dim_z, texcoord_filter);

        return image(i, j, k);
#else
        // Trilinear sampling
        Float w = uvw.x * image.dim_x - 0.5f;
        Float h = uvw.y * image.dim_y - 0.5f;
        Float d = uvw.z * image.dim_z - 0.5f;

        int32 i0 = int32(std::floor(w)), i1 = i0 + 1;
        int32 j0 = int32(std::floor(h)), j1 = j0 + 1;
        int32 k0 = int32(std::floor(d)), k1 = k0 + 1;

        FilterTexCoord3D(&i0, &j0, &k0, image.dim_x, image.dim_y, image.dim_z, texcoord_filter);
        FilterTexCoord3D(&i1, &j1, &k1, image.dim_x, image.dim_y, image.dim_z, texcoord_filter);

        Float fu = w - std::floor(w);
        Float fv = h - std::floor(h);
        Float fw = d - std::floor(d);

        // 8 closest corner values
        T v000 = image(i0, j0, k0);
        T v001 = image(i0, j0, k1);
        T v010 = image(i0, j1, k0);
        T v011 = image(i0, j1, k1);
        T v100 = image(i1, j0, k0);
        T v101 = image(i1, j0, k1);
        T v110 = image(i1, j1, k0);
        T v111 = image(i1, j1, k1);

        // Lerp along x axis
        T v00 = (1 - fu) * v000 + fu * v100;
        T v01 = (1 - fu) * v001 + fu * v101;
        T v10 = (1 - fu) * v010 + fu * v110;
        T v11 = (1 - fu) * v011 + fu * v111;

        // Lerp along y axis
        T v0 = (1 - fv) * v00 + fv * v10;
        T v1 = (1 - fv) * v01 + fv * v11;

        // Lerp along z axis
        return (1 - fw) * v0 + fw * v1;
#endif
    }

    T Average() const
    {
        T sum(0);

        for (int32 z = 0; z < image.dim_z; ++z)
        {
            for (int32 y = 0; y < image.dim_y; ++y)
            {
                for (int32 x = 0; x < image.dim_x; ++x)
                {
                    sum += image(x, y, z);
                }
            }
        }

        return sum / (image.dim_x * image.dim_y * image.dim_z);
    }

private:
    Image3D<T> image;
    TexCoordFilter texcoord_filter;
};

template <typename T>
class CheckerTexture3D : public Texture3D<T>
{
public:
    CheckerTexture3D(const Texture3D<T>* a, const Texture3D<T>* b, const Point3& resolution)
        : a{ a }
        , b{ b }
        , resolution{ resolution }
    {
    }

    virtual T Evaluate(const Point3& uvw) const override
    {
        Point3i scale = uvw * resolution;
        int32 c = scale.x + scale.y + scale.z;

        if (c % 2)
        {
            return a->Evaluate(uvw);
        }
        else
        {
            return b->Evaluate(uvw);
        }
    }

    virtual T Average() const
    {
        return 0.5f * (a->Average() + b->Average());
    }

private:
    const Texture3D<T>* a;
    const Texture3D<T>* b;
    Point3 resolution;
};

using FloatImageTexture3D = ImageTexture3D<Float>;
using SpectrumImageTexture3D = ImageTexture3D<Spectrum>;

using FloatCheckerTexture3D = CheckerTexture3D<Float>;
using SpectrumCheckerTexture3D = CheckerTexture3D<Spectrum>;

} // namespace bulbit
