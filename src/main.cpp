#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "raytracer/bitmap.h"
#include "raytracer/camera.h"
#include "raytracer/common.h"
#include "raytracer/hittable_list.h"
#include "raytracer/material.h"
#include "raytracer/ray.h"
#include "raytracer/sphere.h"

Color TraceRay(const Ray& r, const Hittable& world, int32 depth)
{
    if (depth <= 0)
    {
        return Vec3{ 0.0f };
    }

    HitRecord rec;
    if (world.Hit(r, 0.00001, infinity, rec))
    {
        Ray scattered;
        Color attenuation;

        if (rec.mat->Scatter(r, rec, attenuation, scattered))
        {
            return attenuation * TraceRay(scattered, world, depth - 1);
        }

        return Color{ 0.0 };
    }

    Vec3 unit_direction = r.dir.Normalized();
    double t = 0.5 * (unit_direction.y + 1.0);

    return Lerp(Color{ 1.0, 1.0, 1.0 }, Color{ 0.5, 0.7, 1.0 }, t);
}

int main()
{
    constexpr double aspect_ratio = 16.0 / 9.0;
    constexpr int32 width = 640;
    constexpr int32 height = static_cast<int32>(width / aspect_ratio);
    constexpr int32 samples_per_pixel = 100;
    constexpr double scale = 1.0 / samples_per_pixel;
    const int max_depth = 50;

    Bitmap bitmap{ width, height };

    HittableList world;

    auto material_ground = std::make_shared<Lambertian>(Color{ 0.8, 0.8, 0.0 });
    auto material_center = std::make_shared<Lambertian>(Color{ 0.7, 0.3, 0.3 });
    auto material_left = std::make_shared<Dielectric>(1.5);
    auto material_right = std::make_shared<Metal>(Color{ 0.8, 0.6, 0.2 }, 1.0);

    world.add(std::make_shared<Sphere>(Vec3{ 0.0, -100.5, -1.0 }, 100.0, material_ground));
    world.add(std::make_shared<Sphere>(Vec3{ 0.0, 0.0, -1.0 }, 0.5, material_center));
    world.add(std::make_shared<Sphere>(Vec3{ -1.0, 0.0, -1.0 }, 0.5, material_left));
    world.add(std::make_shared<Sphere>(Vec3(-1.0, 0.0, -1.0), -0.4, material_left));
    world.add(std::make_shared<Sphere>(Vec3{ 1.0, 0.0, -1.0 }, 0.5, material_right));

    Camera camera{ aspect_ratio };

    for (int32 y = height - 1; y >= 0; --y)
    {
        std::cout << "\rScanlines remaining: " << y << ' ' << std::flush;

        for (int32 x = 0; x < width; ++x)
        {
            Color samples{ 0.0 };

            for (int s = 0; s < samples_per_pixel; ++s)
            {
                double u = (x + Rand()) / (width - 1);
                double v = (y + Rand()) / (height - 1);

                Ray r = camera.GetRay(u, v);
                samples += TraceRay(r, world, max_depth);
            }

            // Divide the color by the number of samples and gamma-correct for gamma=2.2.
            Vec3 color =
                Vec3{ pow(samples.x * scale, 1.0 / 2), pow(samples.y * scale, 1.0 / 2), pow(samples.z * scale, 1.0 / 2) };
            bitmap.Set(x, y, color);
        }
    }

    bitmap.WriteToFile("render.png");

    return system("render.png");
}