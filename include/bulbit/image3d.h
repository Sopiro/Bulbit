#pragma once

#include "image.h"

namespace bulbit
{

template <typename T>
struct Image3D
{
    using Type = T;

    Image3D()
        : dim_x{ 0 }
        , dim_y{ 0 }
        , dim_z{ 0 }
        , data{ nullptr }
    {
    }

    Image3D(int32 dim_x, int32 dim_y, int32 dim_z)
        : dim_x{ dim_x }
        , dim_y{ dim_y }
        , dim_z{ dim_z }
    {
        data = std::make_unique<T[]>(dim_x * dim_y * dim_z);
    }

    operator bool() const
    {
        return bool(data);
    }

    T& operator[](int32 i)
    {
        return data[i];
    }

    const T& operator[](int32 i) const
    {
        return data[i];
    }

    T& operator()(int32 x)
    {
        return data[x];
    }

    const T& operator()(int32 x) const
    {
        return data[x];
    }

    T& operator()(int32 x, int32 y, int32 z)
    {
        return data[(z * dim_y + y) * dim_x + x];
    }

    const T& operator()(int32 x, int32 y, int32 z) const
    {
        return data[(z * dim_y + y) * dim_x + x];
    }

    int32 dim_x, dim_y, dim_z;
    std::unique_ptr<T[]> data;
};

using Image3D1f = Image3D<Float>;
using Image3D3f = Image3D<RGBSpectrum>;
using Image3D4f = Image3D<Vec4f>;
using Image3D1 = Image3D1f;
using Image3D3 = Image3D3f;
using Image3D4 = Image3D4f;

Image3D1 ReadImage3D(
    const std::filesystem::path& filename,
    int32 channel,
    Point3i resolution,
    bool non_color = false,
    std::function<Image3D1::Type(Image3D1::Type)> transform = {}
);
Image3D3 ReadImage3D(
    const std::filesystem::path& filename,
    Point3i resolution,
    bool non_color = false,
    std::function<Image3D3::Type(Image3D3::Type)> transform = {}
);

void WriteImage3D(const Image3D1& image, const std::filesystem::path& filename);
void WriteImage3D(const Image3D3& image, const std::filesystem::path& filename);

} // namespace bulbit
