#include <stb/stb_image.h>
#include <stb/stb_image_write.h>

#include "bulbit/parallel_for.h"
#include "bulbit/texture.h"

namespace bulbit
{

FloatImageTexture::FloatImageTexture()
    : floats{ nullptr }
    , width{ 0 }
    , height{ 0 }
    , texcoord_filter{ repeat }
{
}

FloatImageTexture::FloatImageTexture(const std::string& filename, int32 channel, bool srgb)
    : FloatImageTexture({ filename, channel }, srgb)
{
}

FloatImageTexture::FloatImageTexture(const std::pair<std::string, int32>& filename_channel, bool srgb)
    : texcoord_filter{ repeat }
{
    assert(0 <= channel && channel <= 3);

    std::string filename = filename_channel.first;
    int32 channel = filename_channel.second;

    stbi_set_flip_vertically_on_load(true);
    stbi_ldr_to_hdr_scale(1.0f);
    stbi_ldr_to_hdr_gamma(srgb ? 2.2f : 1.0f);

    int32 components_per_pixel;
    float* data = stbi_loadf(filename.data(), &width, &height, &components_per_pixel, STBI_rgb_alpha);

    if (!data)
    {
        std::cerr << "ERROR: Could not load texture '" << filename << std::endl;
        width = 1;
        height = 1;
        floats = std::make_unique<Float[]>(width * height);
        floats[0] = 0;
        return;
    }

    floats = std::make_unique<Float[]>(width * height);

    if (channel < 3)
    {
        if (width * height > 64 * 1024)
        {
            ParallelFor(0, width * height, [=](int32 i) { floats[i] = (Float)std::fmax(0, data[STBI_rgb_alpha * i + channel]); });
        }
        else
        {
            for (int32 i = 0; i < width * height; ++i)
            {
                floats[i] = (Float)std::fmax(0, data[STBI_rgb_alpha * i + channel]);
            }
        }
    }
    else
    {
        if (components_per_pixel == STBI_rgb_alpha)
        {
            floats = std::make_unique<Float[]>(width * height);

            for (int32 i = 0; i < width * height; ++i)
            {
                floats[i] = (Float)std::fmax(0, data[STBI_rgb_alpha * i + 3]);
            }
        }
    }

    stbi_image_free(data);
}

Float FloatImageTexture::Evaluate(const Point2& uv) const
{
#if 0
    // Nearest sampling
    Float w = uv.x * width + Float(0.5);
    Float h = uv.y * height + Float(0.5);

    int32 i = int32(w);
    int32 j = int32(h);

    FilterTexCoord(&i, &j);

    return floats[i + j * width];
#else
    // Bilinear sampling
    Float w = uv.x * width + Float(0.5);
    Float h = uv.y * height + Float(0.5);

    int32 i0 = int32(w), i1 = int32(w) + 1;
    int32 j0 = int32(h), j1 = int32(h) + 1;

    FilterTexCoord(&i0, &j0, width, height, texcoord_filter);
    FilterTexCoord(&i1, &j1, width, height, texcoord_filter);

    Float fu = w - std::floor(w);
    Float fv = h - std::floor(h);

    Float sp00 = floats[i0 + j0 * width], sp10 = floats[i1 + j0 * width];
    Float sp01 = floats[i0 + j1 * width], sp11 = floats[i1 + j1 * width];

    // clang-format off
    return (1-fu) * (1-fv) * sp00 + (1-fu) * (fv) * sp01 +
           (  fu) * (1-fv) * sp10 + (  fu) * (fv) * sp11;
    // clang-format on
#endif
}

} // namespace bulbit