#include "../samples.h"

std::unique_ptr<Camera> GGXVNDFSamplingTest(Scene& scene)
{
    // Bunny
    {
        Srand(123123);
        auto mat = CreateRandomPrincipledMaterial(scene);
        // auto mat = CreateMirrorMaterial(scene, Spectrum(0.7f));
        // auto mat = CreateDielectricMaterial(scene, 1.5f);
        // auto mat = CreateThinDielectricMaterial(scene, 1.5f);
        // auto mat = CreateConductorMaterial(scene, { 0.1, 0.2, 1.9 }, { 3, 2.5, 2 }, 0.1f, 0.3f);
        // auto mix = scene.CreateMaterial<MixtureMaterial>(mat, mat, 0.1f);

        auto tf = Transform{ Vec3::zero, Quat(DegToRad(0.0f), y_axis), Vec3(3.0f) };
        SetLoaderFallbackMaterial(mat);
        LoadModel(scene, "res/stanford/bunny.obj", tf);
    }

    // CreateImageInfiniteLight(scene, "res/HDR/scythian_tombs_2_4k.hdr");
    // CreateImageInfiniteLight(scene, "res/HDR/quarry_04_puresky_1k.hdr");
    // CreateImageInfiniteLight(scene, "res/solitude_night_4k/solitude_night_4k.hdr");
    // CreateImageInfiniteLight(scene, "res/HDR/sunflowers_puresky_1k.hdr");
    CreateImageInfiniteLight(scene, "res/HDR/san_giuseppe_bridge_4k.hdr", Transform(Quat(-pi, y_axis)));

    Float aspect_ratio = 16.f / 9.f;
    // Float aspect_ratio = 3.f / 2.f;
    // Float aspect_ratio = 4.f / 3.f;
    // Float aspect_ratio = 1.f;
    int32 width = 960;
    int32 height = int32(width / aspect_ratio);

    Point3 lookfrom{ 0, 2, 10 };
    Point3 lookat{ 0, 1, 0 };

    Float dist_to_focus = Dist(lookfrom, lookat);
    Float aperture = 0;
    Float vFov = 30;

    return std::make_unique<PerspectiveCamera>(lookfrom, lookat, y_axis, vFov, aperture, dist_to_focus, Point2i(width, height));
}

static int32 sample_index = Sample::Register("ggxvndf", GGXVNDFSamplingTest);
