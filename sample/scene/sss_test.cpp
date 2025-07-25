#include "../samples.h"

std::unique_ptr<Camera> SSSTest(Scene& scene)
{
    HomogeneousMedium* hm = scene.CreateMedium<HomogeneousMedium>(Spectrum(0, 0, 0), Spectrum(10), Spectrum(0.0), -0.9f);
    MediumInterface mi(hm, nullptr);

    SetLoaderUseForceFallbackMaterial(true);

    // Floor
    {
        auto checker = CreateSpectrumCheckerTexture(scene, 0.75, 0.3, Point2(20));
        auto tf = Transform{ Vec3(0, 0, 0), Quat::FromEuler({ 0, 0, 0 }), Vec3(3) };
        auto floor = scene.CreateMaterial<DiffuseMaterial>(checker);
        SetLoaderFallbackMaterial(floor);
        LoadModel(scene, "res/background.obj", tf);
    }

    // Model
    {
        // auto normalmap = CreateSpectrumImageTexture(scene, "res/bistro/Concrete_Normal.png", true);
        auto normalmap = nullptr;
        // auto mat = CreateDiffuseMaterial(scene, Spectrum(212 / 255.f, 175 / 255.f, 55 / 255.f), 0, normalmap);
        auto mat = CreateSubsurfaceRandomWalkMaterial(
            scene, Spectrum(255 / 255.0, 195 / 255.0, 170 / 255.0) * 0.6, Spectrum(0.5, 0.25, 0.125) * 0.03, 1.5f, 0.05f
        );
        // auto mat = CreateSubsurfaceDiffusionMaterial(scene, Spectrum(1.0), Spectrum(0.01), 1.0, 0.0);
        // auto mat = CreateConductorMaterial(scene, Spectrum(0.1, 0.2, 1.9), Spectrum(3, 2.5, 2), (0.05f), (0.4f), normalmap);
        // auto mat = CreateDielectricMaterial(scene, 1.5f, 0.1f);
        // auto mat = CreateThinDielectricMaterial(scene, 1.5f);
        // auto mat = CreateMirrorMaterial(scene, Spectrum(0.7f), normalmap);

        // Srand(1213212);
        // auto mat = CreateRandomPrincipledMaterial(scene);

        SetLoaderFallbackMaterial(mat);
        // SetLoaderFallbackMediumInterface(mi);

        auto tf = Transform{ Vec3(0.2, .78, .3) * 0.5, Quat::FromEuler({ 0, (pi / 4), 0 }), Vec3(0.01) };
        LoadModel(scene, "res/xyzdragon.obj", tf);
        // auto tf = Transform{ Vec3(0), Quat::FromEuler(0, 0, 0), Vec3(1) };
        // LoadModel(scene, "res/stanford/bunny.obj", tf);
    }

    // CreateImageInfiniteLight(scene,
    //     "res/material_test_ball/envmap.hdr", Transform(Quat::FromEuler(0, DegToRad(-67.26139831542969), 0))
    // );
    // CreateImageInfiniteLight(scene, "res/HDR/photo_studio_loft_hall_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/aerodynamics_workshop_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/photo_studio_01_1k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/scythian_tombs_2_4k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/material-test.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/peppermint_powerplant_4k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/quarry_04_puresky_1k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/solitude_night_1k.hdr");
    // CreateImageInfiniteLight(scene, "res/HDR/sunflowers_puresky_1k.hdr");
    // CreateImageInfiniteLight(scene, "res/HDR/san_giuseppe_bridge_4k.hdr", Transform(Quat(-pi, y_axis)));
    // CreateUniformInfiniteLight(scene, Spectrum(1));

    {
        auto light = CreateDiffuseLightMaterial(scene, Spectrum(5.0f));
        auto tf = Transform{ 0, 2.0f, 0, Quat(pi, x_axis), Vec3(1.5f) };
        CreateRectXZ(scene, tf, light);
    }

    {
        // auto light = CreateDiffuseLightMaterial(scene, Spectrum(3.0f));
        // auto tf1 = Transform{ -1, 1.0f, -1, Quat(-pi / 4.0f, y_axis), Vec3(1.0f) };
        // auto tf2 = Transform{ 0, 0, 0, Quat(-pi / 4.0f, z_axis), Vec3(1.5f) };
        // CreateRectYZ(scene, tf1 * tf2, light);
    }

    Float aspect_ratio = 16.f / 9.f;
    // Float aspect_ratio = 9.f / 16.f;
    // Float aspect_ratio = 3.f / 2.f;
    // Float aspect_ratio = 4.f / 3.f;
    // Float aspect_ratio = 1.f;
    int32 width = 1600;
    int32 height = int32(width / aspect_ratio);

    Point3 lookfrom{ 0, 1.0, 3 };
    Point3 lookat{ 0.0, 0.3, 0.0 };

    Float dist_to_focus = Dist(lookfrom, lookat);
    Float aperture = 0.02f;
    Float vFov = 30.0;

    return std::make_unique<PerspectiveCamera>(lookfrom, lookat, y_axis, vFov, aperture, dist_to_focus, Point2i(width, height));
}

static int32 sample_index = Sample::Register("sss", SSSTest);
