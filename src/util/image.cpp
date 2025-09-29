#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include <stb_image.h>
#include <stb_image_write.h>

#include "bulbit/image.h"
#include "bulbit/parallel_for.h"
#include "bulbit/spectrum.h"

namespace bulbit
{

Image1 ReadImage1(
    const std::filesystem::path& filename, int32 channel, bool non_color, std::function<Image1::Type(Image1::Type)> transform
)
{
    stbi_set_flip_vertically_on_load(true);
    stbi_ldr_to_hdr_gamma(non_color ? 1.0f : 2.2f);

    int32 width, height;
    int32 components_per_pixel;
    float* data = stbi_loadf(filename.string().c_str(), &width, &height, &components_per_pixel, STBI_rgb_alpha);

    if (!data)
    {
        return {};
    }

    if (channel < 0 || channel >= components_per_pixel)
    {
        return {};
    }

    constexpr int32 stride = STBI_rgb_alpha;
    Image1 image(width, height);

    if (channel < 3)
    {
        if (width * height > 64 * 1024)
        {
            if (transform)
            {
                ParallelFor(0, width * height, [&](int32 i) {
                    image[i] = transform(Float(std::fmax(0, data[stride * i + channel])));
                });
            }
            else
            {
                ParallelFor(0, width * height, [&](int32 i) { image[i] = Float(std::fmax(0, data[stride * i + channel])); });
            }
        }
        else
        {
            if (transform)
            {
                for (int32 i = 0; i < width * height; ++i)
                {
                    image[i] = transform(Float(std::fmax(0, data[stride * i + channel])));
                }
            }
            else
            {
                for (int32 i = 0; i < width * height; ++i)
                {
                    image[i] = Float(std::fmax(0, data[stride * i + channel]));
                }
            }
        }
    }
    else if (components_per_pixel == STBI_rgb_alpha)
    {
        if (transform)
        {
            for (int32 i = 0; i < width * height; ++i)
            {
                image[i] = transform(Float(std::fmax(0, data[stride * i + channel])));
            }
        }
        else
        {
            for (int32 i = 0; i < width * height; ++i)
            {
                image[i] = Float(std::fmax(0, data[stride * i + channel]));
            }
        }
    }

    stbi_image_free(data);
    return image;
}

Image3 ReadImage3(const std::filesystem::path& filename, bool non_color, std::function<Image3::Type(Image3::Type)> transform)
{
    stbi_set_flip_vertically_on_load(true);
    stbi_ldr_to_hdr_gamma(non_color ? 1.0f : 2.2f);

    int32 width, height;
    int32 components_per_pixel;
    float* data = stbi_loadf(filename.string().c_str(), &width, &height, &components_per_pixel, STBI_rgb);

    if (!data)
    {
        return {};
    }

    constexpr int32 stride = STBI_rgb;
    Image3 image(width, height);

    if (width * height > 64 * 1024)
    {
        if (transform)
        {
            ParallelFor(0, width * height, [&](int32 i) {
                image[i] = transform(Max(Spectrum{ data[stride * i + 0], data[stride * i + 1], data[stride * i + 2] }, 0));
            });
        }
        else
        {
            ParallelFor(0, width * height, [&](int32 i) {
                image[i] = Max(Spectrum{ data[stride * i + 0], data[stride * i + 1], data[stride * i + 2] }, 0);
            });
        }
    }
    else
    {
        if (transform)
        {
            for (int32 i = 0; i < width * height; ++i)
            {
                image[i] = transform(Max(Spectrum{ data[stride * i + 0], data[stride * i + 1], data[stride * i + 2] }, 0));
            }
        }
        else
        {
            for (int32 i = 0; i < width * height; ++i)
            {
                image[i] = Max(Spectrum{ data[stride * i + 0], data[stride * i + 1], data[stride * i + 2] }, 0);
            }
        }
    }

    stbi_image_free(data);
    return image;
}

Image4 ReadImage4(const std::filesystem::path& filename, bool non_color, std::function<Image4::Type(Image4::Type)> transform)
{
    stbi_set_flip_vertically_on_load(true);
    stbi_ldr_to_hdr_gamma(non_color ? 1.0f : 2.2f);

    int32 width, height;
    int32 components_per_pixel;
    float* data = stbi_loadf(filename.string().c_str(), &width, &height, &components_per_pixel, STBI_rgb_alpha);

    if (!data)
    {
        return {};
    }

    constexpr int32 stride = STBI_rgb_alpha;
    Image4 image(width, height);

    if (width * height > 64 * 1024)
    {
        if (transform)
        {
            ParallelFor(0, width * height, [&](int32 i) {
                image[i] = transform(
                    Max(Vec4{ data[stride * i + 0], data[stride * i + 1], data[stride * i + 2],
                              (components_per_pixel == STBI_rgb_alpha ? data[stride * i + 3] : 1) },
                        Vec4(0))
                );
            });
        }
        else
        {
            ParallelFor(0, width * height, [&](int32 i) {
                image[i] =
                    Max(Vec4{ data[stride * i + 0], data[stride * i + 1], data[stride * i + 2],
                              (components_per_pixel == STBI_rgb_alpha ? data[stride * i + 3] : 1) },
                        Vec4(0));
            });
        }
    }
    else
    {
        if (transform)
        {
            for (int32 i = 0; i < width * height; ++i)
            {
                image[i] = transform(
                    Max(Vec4{ data[stride * i + 0], data[stride * i + 1], data[stride * i + 2],
                              (components_per_pixel == STBI_rgb_alpha ? data[stride * i + 3] : 1) },
                        Vec4(0))
                );
            }
        }
        else
        {
            for (int32 i = 0; i < width * height; ++i)
            {
                image[i] =
                    Max(Vec4{ data[stride * i + 0], data[stride * i + 1], data[stride * i + 2],
                              (components_per_pixel == STBI_rgb_alpha ? data[stride * i + 3] : 1) },
                        Vec4(0));
            }
        }
    }

    stbi_image_free(data);
    return image;
}

void WriteImage(const Image3& image, const std::filesystem::path& filename, ToneMappingCallback* callback)
{
    stbi_flip_vertically_on_write(true);

    std::string extension = filename.extension().string();
    if (extension == ".hdr")
    {
        stbi_write_hdr(filename.string().c_str(), image.width, image.height, 3, &image[0].r);
    }
    else if (extension == ".jpg" || extension == ".png")
    {
        std::vector<uint8> pixels(image.width * image.height * 3);

        if (image.width * image.height > 64 * 1024)
        {
            ParallelFor(0, image.width * image.height, [&](int32 i) {
                Vec3 mapped = callback({ image[i].r, image[i].g, image[i].b });

                pixels[i * 3 + 0] = uint8(std::min(std::clamp(mapped[0], 0.0f, 1.0f) * 256.0, 255.0));
                pixels[i * 3 + 1] = uint8(std::min(std::clamp(mapped[1], 0.0f, 1.0f) * 256.0, 255.0));
                pixels[i * 3 + 2] = uint8(std::min(std::clamp(mapped[2], 0.0f, 1.0f) * 256.0, 255.0));
            });
        }
        else
        {
            for (int32 i = 0; i < image.width * image.height; ++i)
            {
                Vec3 mapped = callback({ image[i].r, image[i].g, image[i].b });

                pixels[i * 3 + 0] = uint8(std::min(std::clamp(mapped[0], 0.0f, 1.0f) * 256.0, 255.0));
                pixels[i * 3 + 1] = uint8(std::min(std::clamp(mapped[1], 0.0f, 1.0f) * 256.0, 255.0));
                pixels[i * 3 + 2] = uint8(std::min(std::clamp(mapped[2], 0.0f, 1.0f) * 256.0, 255.0));
            }
        }

        if (extension == ".jpg")
        {
            stbi_write_jpg(filename.string().c_str(), image.width, image.height, 3, &pixels[0], 100);
        }
        else
        {
            stbi_write_png(filename.string().c_str(), image.width, image.height, 3, &pixels[0], image.width * 3);
        }
    }
    else
    {
        std::cout << "Faild to write image, extention not supported: " << extension << std::endl;
        std::cout << "Supported extensions: .jpg .png .hdr" << std::endl;
    }
}

void WriteImage(const Image1& image, const std::filesystem::path& filename, ToneMappingCallback* callback)
{
    stbi_flip_vertically_on_write(true);

    std::string extension = filename.extension().string();
    if (extension == ".hdr")
    {
        stbi_write_hdr(filename.string().c_str(), image.width, image.height, 1, &image[0]);
    }
    else if (extension == ".jpg" || extension == ".png")
    {
        std::vector<uint8> pixels(image.width * image.height * 3);

        if (image.width * image.height > 64 * 1024)
        {
            ParallelFor(0, image.width * image.height, [&](int32 i) {
                Vec3 mapped = callback({ image[i], image[i], image[i] });

                pixels[i * 3 + 0] = uint8(std::min(std::clamp(mapped[0], 0.0f, 1.0f) * 256.0, 255.0));
                pixels[i * 3 + 1] = uint8(std::min(std::clamp(mapped[1], 0.0f, 1.0f) * 256.0, 255.0));
                pixels[i * 3 + 2] = uint8(std::min(std::clamp(mapped[2], 0.0f, 1.0f) * 256.0, 255.0));
            });
        }
        else
        {
            for (int32 i = 0; i < image.width * image.height; ++i)
            {
                Vec3 mapped = callback({ image[i], image[i], image[i] });

                pixels[i * 3 + 0] = uint8(std::min(std::clamp(mapped[0], 0.0f, 1.0f) * 256.0, 255.0));
                pixels[i * 3 + 1] = uint8(std::min(std::clamp(mapped[1], 0.0f, 1.0f) * 256.0, 255.0));
                pixels[i * 3 + 2] = uint8(std::min(std::clamp(mapped[2], 0.0f, 1.0f) * 256.0, 255.0));
            }
        }

        if (extension == ".jpg")
        {
            stbi_write_jpg(filename.string().c_str(), image.width, image.height, 3, &pixels[0], 100);
        }
        else
        {
            stbi_write_png(filename.string().c_str(), image.width, image.height, 3, &pixels[0], image.width * 3);
        }
    }
    else
    {
        std::cout << "Faild to write image, extention not supported: " << extension << std::endl;
        std::cout << "Supported extensions: .jpg .png .hdr" << std::endl;
    }
}

} // namespace bulbit
