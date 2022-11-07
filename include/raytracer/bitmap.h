#pragma once

#include "common.h"
#include "stb/stb_image_write.h"

#define COLOR_CHANNEL (3)

class Bitmap
{
public:
    Bitmap(int32 width, int32 height)
        : width{ width }
        , height{ height }
    {
        pixels = new uint8[width * height * COLOR_CHANNEL];
    }

    void Set(int32 x, int32 y, const Color& color)
    {
        pixels[(x + (height - y - 1) * width) * COLOR_CHANNEL + 0] = static_cast<int32>(color.x * (255.99));
        pixels[(x + (height - y - 1) * width) * COLOR_CHANNEL + 1] = static_cast<int32>(color.y * (255.99));
        pixels[(x + (height - y - 1) * width) * COLOR_CHANNEL + 2] = static_cast<int32>(color.z * (255.99));
    }

    void WriteToFile(char const* filename)
    {
        stbi_write_png(filename, width, height, COLOR_CHANNEL, pixels, width * COLOR_CHANNEL);
    }

    ~Bitmap()
    {
        delete[] pixels;
    }

private:
    int32 width;
    int32 height;

    uint8* pixels;
};