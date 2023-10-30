#pragma once

#include "common.h"

#include <stb/stb_image.h>
#include <stb/stb_image_write.h>

namespace bulbit
{

class Bitmap
{
public:
    Bitmap(int32 width, int32 height);
    ~Bitmap();

    int32 GetWidth() const;
    int32 GetHeight() const;

    void Set(int32 x, int32 y, const Spectrum& color);
    void WriteToFile(const char* filename) const;

    inline static constexpr int32 color_channels = 3;

private:
    int32 width, height;
    uint8* pixels;
};

inline Bitmap::Bitmap(int32 _width, int32 _height)
    : width{ _width }
    , height{ _height }
{
    pixels = new uint8[width * height * color_channels];
}

inline Bitmap::~Bitmap()
{
    delete[] pixels;
}

inline int32 Bitmap::GetWidth() const
{
    return width;
}

inline int32 Bitmap::GetHeight() const
{
    return height;
}

inline void Bitmap::Set(int32 x, int32 y, const Spectrum& color)
{
    pixels[(x + (height - y - 1) * width) * color_channels + 0] = int32(std::fmin(Clamp(color.r, 0.0, 1.0) * 256.0, 255.0));
    pixels[(x + (height - y - 1) * width) * color_channels + 1] = int32(std::fmin(Clamp(color.g, 0.0, 1.0) * 256.0, 255.0));
    pixels[(x + (height - y - 1) * width) * color_channels + 2] = int32(std::fmin(Clamp(color.b, 0.0, 1.0) * 256.0, 255.0));
}

inline void Bitmap::WriteToFile(const char* filename) const
{
    // stbi_write_png(filename, width, height, color_channels, pixels, width * color_channels);
    stbi_write_jpg(filename, width, height, color_channels, pixels, 100);
}

} // namespace bulbit