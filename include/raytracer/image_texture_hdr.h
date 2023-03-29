#pragma once

#include "image_texture.h"

namespace spt
{

class ImageTextureHDR : public ImageTexture
{
public:
    virtual ~ImageTextureHDR() = default;

    virtual Color Value(const UV& uv, const Vec3& p) const override;

protected:
    friend class ImageTexture;

    ImageTextureHDR() = default;
    ImageTextureHDR(std::string path, bool srgb);
};

inline ImageTextureHDR::ImageTextureHDR(std::string path, bool srgb)
{
    int32 components_per_pixel = bytes_per_pixel;

    data = stbi_loadf(path.data(), &width, &height, &components_per_pixel, components_per_pixel);

    if (!data)
    {
        std::cerr << "ERROR: Could not load texture image file '" << path << "'.\n";
        width = 0;
        height = 0;
    }
    else
    {
        if (srgb)
        {
#pragma omp parallel for
            for (int32 i = 0; i < width * height * bytes_per_pixel; ++i)
            {
                // Convert to linear space
                ((float*)data)[i] = static_cast<float>(pow(((float*)data)[i], 2.2));
            }
        }
    }

    bytes_per_scanline = bytes_per_pixel * width;
}

inline Color ImageTextureHDR::Value(const UV& uv, const Vec3& p) const
{
    // Clamp input texture coordinates to [0,1] x [1,0]
    double u = Clamp(uv.x, 0.0, 1.0);
    double v = 1.0 - Clamp(uv.y, 0.0, 1.0); // Flip V to image coordinates

    int32 i = static_cast<int32>(u * width);
    int32 j = static_cast<int32>(v * height);

    // Clamp integer mapping, since actual coordinates should be less than 1.0
    if (i >= width)
    {
        i = width - 1;
    }
    if (j >= height)
    {
        j = height - 1;
    }

    float* pixel = (float*)(data) + j * bytes_per_scanline + i * bytes_per_pixel;

    return Color{ pixel[0], pixel[1], pixel[2] };
}

} // namespace spt