#include <stb_image.h>
#include <stb_image_write.h>

#include "bulbit/image3d.h"
#include "bulbit/parallel_for.h"
#include "bulbit/spectrum.h"

namespace bulbit
{

Image3D1 ReadImage3D(
    const std::filesystem::path& filename, int32 channel, Point3i resolution, bool non_color, Image3D1::Type multiplier
)
{
    stbi_set_flip_vertically_on_load(true);
    stbi_ldr_to_hdr_gamma(non_color ? 1.0f : 2.2f);

    int32 dim_x, dim_y, dim_z;
    int32 components_per_pixel;
    float* data = stbi_loadf(filename.string().c_str(), &dim_x, &dim_y, &components_per_pixel, STBI_rgb);

    if (!data)
    {
        return {};
    }

    if (channel < 0 || channel >= components_per_pixel)
    {
        return {};
    }

    if (dim_x != resolution.x || dim_y != resolution.y * resolution.z)
    {
        return {};
    }

    dim_z = dim_y / resolution.y;
    dim_y /= resolution.z;

    if (dim_y != resolution.y || dim_z != resolution.z)
    {
        return {};
    }

    Image3D1 image(dim_x, dim_y, dim_z);

    if (channel < 3)
    {
        if (dim_x * dim_y * dim_z > 64 * 1024)
        {
            ParallelFor(0, dim_x * dim_y * dim_z, [&](int32 i) {
                image[i] = Float(std::fmax(0, multiplier * data[STBI_rgb_alpha * i + channel]));
            });
        }
        else
        {
            for (int32 i = 0; i < dim_x * dim_y * dim_z; ++i)
            {
                image[i] = Float(std::fmax(0, multiplier * data[STBI_rgb_alpha * i + channel]));
            }
        }
    }
    else if (components_per_pixel == STBI_rgb_alpha)
    {
        for (int32 i = 0; i < dim_x * dim_y * dim_z; ++i)
        {
            image[i] = Float(std::fmax(0, multiplier * data[STBI_rgb_alpha * i + channel]));
        }
    }

    stbi_image_free(data);
    return image;
}

Image3D3 ReadImage3D(const std::filesystem::path& filename, Point3i resolution, bool non_color, Image3D3::Type multiplier)
{
    stbi_set_flip_vertically_on_load(true);
    stbi_ldr_to_hdr_gamma(non_color ? 1.0f : 2.2f);

    int32 dim_x, dim_y, dim_z;
    int32 components_per_pixel;
    float* data = stbi_loadf(filename.string().c_str(), &dim_x, &dim_y, &components_per_pixel, STBI_rgb);

    if (!data)
    {
        return {};
    }

    if (dim_x != resolution.x || dim_y != resolution.y * resolution.z)
    {
        return {};
    }

    dim_z = dim_y / resolution.y;
    dim_y /= resolution.z;

    if (dim_y != resolution.y || dim_z != resolution.z)
    {
        return {};
    }

    Image3D3 image(dim_x, dim_y, dim_z);

    if (dim_x * dim_y * dim_z > 64 * 1024)
    {
        ParallelFor(0, dim_x * dim_y * dim_z, [&](int32 i) {
            image[i][0] = Float(std::fmax(0, multiplier[0] * data[STBI_rgb * i + 0]));
            image[i][1] = Float(std::fmax(0, multiplier[1] * data[STBI_rgb * i + 1]));
            image[i][2] = Float(std::fmax(0, multiplier[2] * data[STBI_rgb * i + 2]));
        });
    }
    else
    {
        for (int32 i = 0; i < dim_x * dim_y * dim_z; ++i)
        {
            image[i][0] = Float(std::fmax(0, multiplier[0] * data[STBI_rgb * i + 0]));
            image[i][1] = Float(std::fmax(0, multiplier[1] * data[STBI_rgb * i + 1]));
            image[i][2] = Float(std::fmax(0, multiplier[2] * data[STBI_rgb * i + 2]));
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
        std::cout << "Faild to write image, extention not supported: " << extension << std::endl;
        std::cout << "Supported extensions: .hdr" << std::endl;
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
        std::cout << "Faild to write image, extention not supported: " << extension << std::endl;
        std::cout << "Supported extensions: .hdr" << std::endl;
    }
}

} // namespace bulbit
