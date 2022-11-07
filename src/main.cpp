#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "raytracer/bitmap.h"
#include "raytracer/camera.h"
#include "raytracer/common.h"
#include "raytracer/ray.h"

Color Evaluate(const Ray& r)
{
    Vec3 dir = r.dir.Normalized();
    double t = 0.5 * (dir.y + 1.0);

    return Lerp(Color{ 1.0, 1.0, 1.0 }, Color{ 0.5, 0.7, 1.0 }, t);
}

int main()
{
    constexpr double aspect_ratio = 16.0 / 9.0;
    constexpr int32 width = 640;
    constexpr int32 height = static_cast<int32>(width / aspect_ratio);

    Bitmap bitmap{ width, height };
    Camera camera{ aspect_ratio };

    for (int32 y = height - 1; y >= 0; --y)
    {
        std::cout << "\rScanlines remaining: " << y << ' ' << std::flush;

        for (int32 x = 0; x < width; ++x)
        {
            double u = double(x) / (width - 1);
            double v = double(y) / (height - 1);

            Ray ray = camera.GetRay(u, v);
            Color color = Evaluate(ray);

            bitmap.Set(x, y, color);
        }
    }

    bitmap.WriteToFile("render.png");

    return system("render.png");
}