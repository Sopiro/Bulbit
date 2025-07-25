#include "../samples.h"

std::unique_ptr<Camera> VolumeTest(Scene& scene)
{
    // Bunny
    {
        Srand(123123);
        // auto mat = CreateRandomPrincipledMaterial(scene);
        // auto mat = CreateMirrorMaterial(scene, Spectrum(0.7f));
        // auto mat = CreateDielectricMaterial(scene, 1.5f, 0.02f);
        auto mat = CreateThinDielectricMaterial(scene, 1.5f);
        // auto mat = CreateConductorMaterial(scene,
        //     CreateSpectrumConstantTexture(scene, 0.1, 0.2, 1.9), CreateSpectrumConstantTexture(scene, 3, 2.5, 2),
        //     CreateFloatConstantTexture(scene, 0.1f), CreateFloatConstantTexture(scene, 0.3f)
        // );
        // auto mix = scene.CreateMaterial<MixtureMaterial>(mat, mat, 0.1f);

        auto tf = Transform{ Vec3(0.0f, -1.0f, 0.0f), Quat(DegToRad(0.0f), y_axis), Vec3(4.0f) };
        SetLoaderUseForceFallbackMaterial(true);
        SetLoaderFallbackMaterial(mat);

        HomogeneousMedium* hm =
            scene.CreateMedium<HomogeneousMedium>(Spectrum(0.1, 0.2, 0.3), Spectrum(1, 5, 10), Spectrum(0.0), -0.9f);
        MediumInterface mi(hm, nullptr);
        SetLoaderFallbackMediumInterface(mi);
        LoadModel(scene, "res/stanford/lucy.obj", tf);
    }

    CreateImageInfiniteLight(scene, "res/HDR/scythian_tombs_2_4k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/quarry_04_puresky_1k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/solitude_night_4k/solitude_night_4k.hdr");
    // CreateImageInfiniteLight(scene, "res/HDR/sunflowers_puresky_1k.hdr");
    // CreateImageInfiniteLight(scene, "res/HDR/san_giuseppe_bridge_4k.hdr", Transform(Quat(-pi, y_axis)));
    // CreateUniformInfiniteLight(scene, Spectrum(1));

    // Float aspect_ratio = 16.f / 9.f;
    Float aspect_ratio = 9.f / 16.f;
    // Float aspect_ratio = 3.f / 2.f;
    // Float aspect_ratio = 4.f / 3.f;
    // Float aspect_ratio = 1.f;
    int32 width = 576;
    int32 height = int32(width / aspect_ratio);

    Point3 lookfrom{ 0, 2, 10 };
    Point3 lookat{ 0, 1, 0 };

    Float dist_to_focus = Dist(lookfrom, lookat);
    Float aperture = 0.05f;
    Float vFov = 30;

    return std::make_unique<PerspectiveCamera>(lookfrom, lookat, y_axis, vFov, aperture, dist_to_focus, Point2i(width, height));
}

static int32 sample_index = Sample::Register("volume", VolumeTest);
