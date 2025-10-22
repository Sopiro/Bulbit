#include <stb_image.h>
#include <stb_image_write.h>

#include "bulbit/image3d.h"
#include "bulbit/parallel_for.h"
#include "bulbit/spectrum.h"

namespace bulbit
{

Image3D1 ReadImage3D(
    const std::filesystem::path& filename,
    int32 channel,
    Point3i resolution,
    bool non_color,
    std::function<Image3D1::Type(Image3D1::Type)> transform
)
{
    stbi_set_flip_vertically_on_load(true);
    stbi_ldr_to_hdr_gamma(non_color ? 1.0f : 2.2f);

    int32 dim_x, dim_y, dim_z;
    int32 components_per_pixel;
    float* data = stbi_loadf(filename.string().c_str(), &dim_x, &dim_y, &components_per_pixel, STBI_rgb_alpha);

    if (!data || channel < 0 || channel >= components_per_pixel)
    {
        std::cerr << "Failed to read image3D1: " << filename.string().c_str() << std::endl;
        return {};
    }

    dim_z = dim_y / resolution.y;
    dim_y /= resolution.z;

    if (dim_x != resolution.x || dim_y != resolution.y || dim_z != resolution.z)
    {
        std::cerr << "Failed to read image3D1: " << filename.string().c_str() << "\n";
        std::cerr << "Dimension not matched: (" << dim_x << ", " << dim_y << ", " << dim_z << ")" << std::endl;
        return {};
    }

    constexpr int32 stride = STBI_rgb_alpha;
    Image3D1 image(dim_x, dim_y, dim_z);

    if (channel < 3)
    {
        if (dim_x * dim_y * dim_z > 64 * 1024)
        {
            if (transform)
            {
                ParallelFor(0, dim_x * dim_y * dim_z, [&](int32 i) {
                    image[i] = transform(Float(std::fmax(0, data[stride * i + channel])));
                });
            }
            else
            {
                ParallelFor(0, dim_x * dim_y * dim_z, [&](int32 i) {
                    image[i] = Float(std::fmax(0, data[stride * i + channel]));
                });
            }
        }
        else
        {
            if (transform)
            {
                for (int32 i = 0; i < dim_x * dim_y * dim_z; ++i)
                {
                    image[i] = transform(Float(std::fmax(0, data[stride * i + channel])));
                }
            }
            else
            {
                for (int32 i = 0; i < dim_x * dim_y * dim_z; ++i)
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
            for (int32 i = 0; i < dim_x * dim_y * dim_z; ++i)
            {
                image[i] = transform(Float(std::fmax(0, data[stride * i + channel])));
            }
        }
        else
        {
            for (int32 i = 0; i < dim_x * dim_y * dim_z; ++i)
            {
                image[i] = Float(std::fmax(0, data[stride * i + channel]));
            }
        }
    }

    stbi_image_free(data);
    return image;
}

Image3D3 ReadImage3D(
    const std::filesystem::path& filename,
    Point3i resolution,
    bool non_color,
    std::function<Image3D3::Type(Image3D3::Type)> transform
)
{
    stbi_set_flip_vertically_on_load(true);
    stbi_ldr_to_hdr_gamma(non_color ? 1.0f : 2.2f);

    int32 dim_x, dim_y, dim_z;
    int32 components_per_pixel;
    float* data = stbi_loadf(filename.string().c_str(), &dim_x, &dim_y, &components_per_pixel, STBI_rgb);

    if (!data)
    {
        std::cerr << "Failed to read image3D3: " << filename.string().c_str() << std::endl;
        return {};
    }

    dim_z = dim_y / resolution.y;
    dim_y /= resolution.z;

    if (dim_x != resolution.x || dim_y != resolution.y || dim_z != resolution.z)
    {
        std::cerr << "Failed to read image3D3: " << filename.string().c_str() << "\n";
        std::cerr << "Dimension not matched: (" << dim_x << ", " << dim_y << ", " << dim_z << ")" << std::endl;
        return {};
    }

    constexpr int32 stride = STBI_rgb;
    Image3D3 image(dim_x, dim_y, dim_z);

    if (dim_x * dim_y * dim_z > 64 * 1024)
    {
        if (transform)
        {
            ParallelFor(0, dim_x * dim_y * dim_z, [&](int32 i) {
                image[i] = transform(Max(Spectrum{ data[stride * i + 0], data[stride * i + 1], data[stride * i + 2] }, 0));
            });
        }
        else
        {
            ParallelFor(0, dim_x * dim_y * dim_z, [&](int32 i) {
                image[i] = Max(Spectrum{ data[stride * i + 0], data[stride * i + 1], data[stride * i + 2] }, 0);
            });
        }
    }
    else
    {
        if (transform)
        {
            for (int32 i = 0; i < dim_x * dim_y * dim_z; ++i)
            {
                image[i] = transform(Max(Spectrum{ data[stride * i + 0], data[stride * i + 1], data[stride * i + 2] }, 0));
            }
        }
        else
        {
            for (int32 i = 0; i < dim_x * dim_y * dim_z; ++i)
            {
                image[i] = Max(Spectrum{ data[stride * i + 0], data[stride * i + 1], data[stride * i + 2] }, 0);
            }
        }
    }

    stbi_image_free(data);
    return image;
}

void WriteImage3D(const Image3D1& image, const std::filesystem::path& filename)
{
    stbi_flip_vertically_on_write(true);

    std::string extension = filename.extension().string();

    if (extension == ".hdr")
    {
        stbi_write_hdr(filename.string().c_str(), image.dim_x, image.dim_y * image.dim_z, 1, &image[0]);
    }
    else
    {
        std::cerr << "Faild to write image3D, extention not supported: " << extension << std::endl;
        std::cerr << "Supported extensions: .hdr" << std::endl;
    }
}

void WriteImage3D(const Image3D3& image, const std::filesystem::path& filename)
{
    stbi_flip_vertically_on_write(true);

    std::string extension = filename.extension().string();

    if (extension == ".hdr")
    {
        stbi_write_hdr(filename.string().c_str(), image.dim_x, image.dim_y * image.dim_z, 3, &image[0].r);
    }
    else
    {
        std::cerr << "Faild to write image3D, extention not supported: " << extension << std::endl;
        std::cerr << "Supported extensions: .hdr" << std::endl;
    }
}

} // namespace bulbit
