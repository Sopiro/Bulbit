#include "../samples.h"

std::unique_ptr<Camera> BVHTest(Scene& scene)
{
    auto gray = CreateDiffuseMaterial(scene, Spectrum(0.8f, 0.8f, 0.8f));
    auto red = CreateDiffuseMaterial(scene, Spectrum(.65f, .05f, .05f));
    auto green = CreateDiffuseMaterial(scene, Spectrum(.12f, .45f, .15f));
    auto blue = CreateDiffuseMaterial(scene, Spectrum(.22f, .23f, .75f));
    auto white = CreateDiffuseMaterial(scene, Spectrum(.73f, .73f, .73f));
    auto black = CreateDiffuseMaterial(scene, Spectrum(0.0f));

    Float n = 100.0f;
    Float w = 7.0f;
    Float h = w * 9.0f / 16.0f;
    Float r = 0.05f;

    for (int32 y = 0; y < n; ++y)
    {
        for (int32 x = 0; x < n; ++x)
        {
            Vec3 pos;
            // pos.x = x / n * w - w / 2.0;
            // pos.y = y / n * w - w / 2.0;

            pos.x = Rand(-w, w);
            pos.y = Rand(-h, h);
            pos.z = -1;

            CreateSphere(scene, pos, r, green);
        }
    }

    CreateImageInfiniteLight(scene, "res/HDR/quarry_04_puresky_1k.hdr");

    Float aspect_ratio = 16.f / 9.f;
    // Float aspect_ratio = 3.f / 2.f;
    // Float aspect_ratio = 4.f / 3.f;
    // Float aspect_ratio = 1.f;
    int32 width = 500;
    int32 height = int32(width / aspect_ratio);

    Point3 lookfrom{ 0, 0, 5 };
    Point3 lookat{ 0, 0, 0 };

    Float dist_to_focus = Dist(lookfrom, lookat);
    Float aperture = 0.0f;
    Float vFov = 71.f;

    return std::make_unique<PerspectiveCamera>(lookfrom, lookat, y_axis, vFov, aperture, dist_to_focus, Point2i(width, height));
}

static int32 sample_index = Sample::Register("bvh", BVHTest);
