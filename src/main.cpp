#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "raytracer/bitmap.h"
#include "raytracer/camera.h"
#include "raytracer/common.h"
#include "raytracer/hittable_list.h"
#include "raytracer/material.h"
#include "raytracer/ray.h"
#include "raytracer/sphere.h"

HittableList RandomScene()
{
    HittableList world;

    auto ground_material = std::make_shared<Lambertian>(Color(0.5, 0.5, 0.5));
    world.add(std::make_shared<Sphere>(Vec3(0, -1000, 0), 1000, ground_material));

    for (int a = -11; a < 11; a++)
    {
        for (int b = -11; b < 11; b++)
        {
            auto choose_mat = Rand();
            Vec3 center(a + 0.9 * Rand(), 0.2, b + 0.9 * Rand());

            if ((center - Vec3(4, 0.2, 0)).Length() > 0.9)
            {
                std::shared_ptr<Material> sphere_material;

                if (choose_mat < 0.8)
                {
                    // diffuse
                    auto albedo = Color::Random() * Color::Random();
                    sphere_material = std::make_shared<Lambertian>(albedo);
                    world.add(std::make_shared<Sphere>(center, 0.2, sphere_material));
                }
                else if (choose_mat < 0.95)
                {
                    // metal
                    auto albedo = Color::Random(0.5, 1);
                    auto fuzz = Rand(0, 0.5);
                    sphere_material = std::make_shared<Metal>(albedo, fuzz);
                    world.add(std::make_shared<Sphere>(center, 0.2, sphere_material));
                }
                else
                {
                    // glass
                    sphere_material = std::make_shared<Dielectric>(1.5);
                    world.add(std::make_shared<Sphere>(center, 0.2, sphere_material));
                }
            }
        }
    }

    auto material1 = std::make_shared<Dielectric>(1.5);
    world.add(std::make_shared<Sphere>(Vec3(0, 1, 0), 1.0, material1));

    auto material2 = std::make_shared<Lambertian>(Color(0.4, 0.2, 0.1));
    world.add(std::make_shared<Sphere>(Vec3(-4, 1, 0), 1.0, material2));

    auto material3 = std::make_shared<Metal>(Color(0.7, 0.6, 0.5), 0.0);
    world.add(std::make_shared<Sphere>(Vec3(4, 1, 0), 1.0, material3));

    return world;
}

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

    HittableList world = RandomScene();

    // auto material_ground = std::make_shared<Lambertian>(Color{ 0.8, 0.8, 0.0 });
    // auto material_center = std::make_shared<Lambertian>(Color{ 0.1, 0.2, 0.5 });
    // auto material_left = std::make_shared<Dielectric>(1.5);
    // auto material_right = std::make_shared<Metal>(Color{ 0.8, 0.6, 0.2 }, 0.0);

    // world.add(std::make_shared<Sphere>(Vec3{ 0.0, -100.5, -1.0 }, 100.0, material_ground));
    // world.add(std::make_shared<Sphere>(Vec3{ 0.0, 0.0, -1.0 }, 0.5, material_center));
    // world.add(std::make_shared<Sphere>(Vec3{ -1.0, 0.0, -1.0 }, 0.5, material_left));
    // world.add(std::make_shared<Sphere>(Vec3(-1.0, 0.0, -1.0), -0.45, material_left));
    // world.add(std::make_shared<Sphere>(Vec3{ 1.0, 0.0, -1.0 }, 0.5, material_right));

    Vec3 lookfrom(13, 2, 3);
    Vec3 lookat(0, 0, 0);
    Vec3 vup(0, 1, 0);
    auto dist_to_focus = 10.0;
    auto aperture = 0.1;

    Camera camera{ lookfrom, lookat, vup, 20, aspect_ratio, aperture, dist_to_focus };

    auto t0 = std::chrono::system_clock::now();
    for (int32 y = height - 1; y >= 0; --y)
    {
        // std::cout << "\rScanlines remaining: " << y << ' ' << std::flush;
        std::printf("\rProcessing... %.2lf%%", double(height - y - 1) / (height - 1) * 100.0);

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
                Vec3{ pow(samples.x * scale, 1.0 / 2.2), pow(samples.y * scale, 1.0 / 2.2), pow(samples.z * scale, 1.0 / 2.2) };
            bitmap.Set(x, y, color);
        }
    }

    auto t1 = std::chrono::system_clock::now();
    std::time_t tt = std::chrono::system_clock::to_time_t(t1);
    std::chrono::duration<double> d = t1 - t0;

    std::cout << "\nDone!: " << d.count() << 's' << std::endl;

    std::string fileName = "render-" + std::to_string(d.count()) + "s.png";

    bitmap.WriteToFile(fileName.c_str());

    return system(fileName.c_str());
}