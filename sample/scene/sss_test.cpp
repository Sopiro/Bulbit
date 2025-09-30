#include "../samples.h"

SceneInfo SSSTest()
{
    auto scene = std::make_unique<Scene>();

    HomogeneousMedium* hm = scene->CreateMedium<HomogeneousMedium>(Spectrum(0, 0, 0), Spectrum(10), Spectrum(0.0), -0.9f);
    MediumInterface mi(hm, nullptr);

    ModelLoaderOptions options;
    options.use_fallback_material = true;

    // Floor
    {
        auto checker = CreateSpectrumCheckerTexture(*scene, 0.75, 0.3, Point2(20));
        auto tf = Transform{ Vec3(0, 0, 0), Quat::FromEuler({ 0, 0, 0 }), Vec3(3) };
        auto floor = scene->CreateMaterial<DiffuseMaterial>(checker);
        options.fallback_material = floor;
        LoadModel(*scene, "res/background.obj", tf, options);
    }

    // Model
    {
        // auto normalmap = CreateSpectrumImageTexture(*scene, "res/bistro/Concrete_Normal.png", true);
        auto normalmap = nullptr;
        // auto mat = CreateDiffuseMaterial(*scene, Spectrum(212 / 255.f, 175 / 255.f, 55 / 255.f), 0, normalmap);
        auto mat = CreateSubsurfaceRandomWalkMaterial(
            *scene, Spectrum(255 / 255.0, 195 / 255.0, 170 / 255.0) * 0.6, Spectrum(0.5, 0.25, 0.125) * 0.03, 1.5f, 0.05f
        );
        // auto mat = CreateSubsurfaceDiffusionMaterial(*scene, Spectrum(1.0), Spectrum(0.01), 1.0, 0.0);
        // auto mat = CreateConductorMaterial(*scene, Spectrum(0.1, 0.2, 1.9), Spectrum(3, 2.5, 2), (0.05f), (0.4f), normalmap);
        // auto mat = CreateDielectricMaterial(*scene, 1.5f, 0.1f);
        // auto mat = CreateThinDielectricMaterial(*scene, 1.5f);
        // auto mat = CreateMirrorMaterial(*scene, Spectrum(0.7f), normalmap);

        // Srand(1213212);
        // auto mat = CreateRandomPrincipledMaterial(*scene);

        options.fallback_material = mat;
        // options.fallback_medium_interface = mi;

        auto tf = Transform{ Vec3(0.2, .8, .3) * 0.5, Quat::FromEuler({ 0, (pi / 4), 0 }) };
        LoadModel(*scene, "res/xyzdragon.obj", tf, options);
        // auto tf = Transform{ Vec3(0), Quat::FromEuler(0, 0, 0) };
        // LoadModel(*scene, "res/stanford/bunny.obj", tf, options);
    }

    // CreateImageInfiniteLight(*scene,
    //     "res/material_test_ball/envmap.hdr", Transform(Quat::FromEuler(0, DegToRad(-67.26139831542969), 0))
    // );
    // CreateImageInfiniteLight(*scene, "res/HDR/photo_studio_loft_hall_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(*scene, "res/HDR/aerodynamics_workshop_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(*scene, "res/HDR/photo_studio_01_1k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(*scene, "res/HDR/scythian_tombs_2_4k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(*scene, "res/HDR/material-test.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(*scene, "res/HDR/peppermint_powerplant_4k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(*scene, "res/HDR/quarry_04_puresky_1k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(*scene, "res/HDR/solitude_night_1k.hdr");
    // CreateImageInfiniteLight(*scene, "res/HDR/sunflowers_puresky_1k.hdr");
    // CreateImageInfiniteLight(*scene, "res/HDR/san_giuseppe_bridge_4k.hdr", Transform(Quat(-pi, y_axis)));
    // CreateUniformInfiniteLight(*scene, Spectrum(1));

    {
        auto light = CreateDiffuseLightMaterial(*scene, Spectrum(5.0f));
        auto tf = Transform{ 0, 2.0f, 0, Quat(pi, x_axis), Vec3(1.5f) };
        CreateRectXZ(*scene, tf, light);
    }

    {
        // auto light = CreateDiffuseLightMaterial(*scene, Spectrum(3.0f));
        // auto tf1 = Transform{ -1, 1.0f, -1, Quat(-pi / 4.0f, y_axis), Vec3(1.0f) };
        // auto tf2 = Transform{ 0, 0, 0, Quat(-pi / 4.0f, z_axis), Vec3(1.5f) };
        // CreateRectYZ(*scene, tf1 * tf2, light);
    }

    Float aspect_ratio = 16.f / 9.f;
    // Float aspect_ratio = 9.f / 16.f;
    // Float aspect_ratio = 3.f / 2.f;
    // Float aspect_ratio = 4.f / 3.f;
    // Float aspect_ratio = 1.f;
    int32 width = 1600;
    int32 height = int32(width / aspect_ratio);

    Point3 position{ 0, 1.0, 3 };
    Point3 target{ 0.0, 0.3, 0.0 };

    Float aperture = 0.02f;
    Float fov = 30.0;

    SceneInfo si;
    si.scene = std::move(scene);
    si.renderer_info.type = IntegratorType::path;
    si.renderer_info.max_bounces = 64;
    si.camera_info.type = CameraType::perspective;
    si.camera_info.tf = Transform::LookAt(position, target, y_axis);
    si.camera_info.fov = fov;
    si.camera_info.aperture = aperture;
    si.camera_info.focus_distance = Dist(position, target);
    si.camera_info.film_info.filename = "";
    si.camera_info.film_info.resolution = { width, height };
    si.camera_info.sampler_info.type = SamplerType::stratified;
    si.camera_info.sampler_info.spp = 64;

    return si;
}

SceneInfo SSSTest2()
{
    auto scene = std::make_unique<Scene>();

    ModelLoaderOptions options;
    options.use_fallback_material = true;

    // Floor
    {
        auto checker = CreateSpectrumCheckerTexture(*scene, 0.75, 0.3, Point2(20));
        auto tf = Transform{ Vec3(0, 0, 0), Quat::FromEuler({ 0, 0, 0 }), Vec3(3) };
        auto floor = scene->CreateMaterial<DiffuseMaterial>(checker);
        options.fallback_material = floor;
        LoadModel(*scene, "res/background.obj", tf, options);
    }

    // Model
    {
        Float d = 0.65;

        Spectrum r(0, 163 / 255.0, 108 / 255.0);
        Spectrum ssc(1);

        auto tf = Transform{ Vec3(-2 * d, 0, 0), Quat::FromEuler({ 0, 0, 0 }), Vec3(1) };
        auto mat = CreateSubsurfaceDiffusionMaterial(*scene, r, ssc * 0.001, 1.5f, 0.0f);
        options.fallback_material = mat;
        LoadModel(*scene, "res/stanford/lucy.obj", tf, options);

        tf = Transform{ Vec3(-1 * d, 0, 0), Quat::FromEuler({ 0, 0, 0 }), Vec3(1) };
        mat = CreateSubsurfaceDiffusionMaterial(*scene, r, ssc * 0.005, 1.5f, 0.0f);
        options.fallback_material = mat;
        LoadModel(*scene, "res/stanford/lucy.obj", tf, options);

        tf = Transform{ Vec3(0, 0, 0), Quat::FromEuler({ 0, 0, 0 }), Vec3(1) };
        mat = CreateSubsurfaceDiffusionMaterial(*scene, r, ssc * 0.008, 1.5f, 0.0f);
        options.fallback_material = mat;
        LoadModel(*scene, "res/stanford/lucy.obj", tf, options);

        tf = Transform{ Vec3(1 * d, 0, 0), Quat::FromEuler({ 0, 0, 0 }), Vec3(1) };
        mat = CreateSubsurfaceDiffusionMaterial(*scene, r, ssc * 0.01, 1.5f, 0.0f);
        options.fallback_material = mat;
        LoadModel(*scene, "res/stanford/lucy.obj", tf, options);

        tf = Transform{ Vec3(2 * d, 0, 0), Quat::FromEuler({ 0, 0, 0 }), Vec3(1) };
        mat = CreateSubsurfaceDiffusionMaterial(*scene, r, ssc * 0.02, 1.5f, 0.0f);
        options.fallback_material = mat;
        LoadModel(*scene, "res/stanford/lucy.obj", tf, options);
    }

    {
        auto tf = Transform{ 0, 3.0f, 0, Quat(pi, x_axis), Vec3(5.0f, 1, 1) };
        auto light = CreateDiffuseLightMaterial(*scene, Spectrum(5.0f));
        CreateRectXZ(*scene, tf, light);
    }

    Float aspect_ratio = 16.f / 9.f;
    int32 width = 960;
    int32 height = int32(width / aspect_ratio);

    Point3 position{ 0, 1.5, 3.5 };
    Point3 target{ 0.0, 0.5, 0.0 };

    Float aperture = 0.01f;
    Float fov = 30.0;

    SceneInfo si;
    si.scene = std::move(scene);
    si.renderer_info.type = IntegratorType::path;
    si.renderer_info.max_bounces = 64;
    si.camera_info.type = CameraType::perspective;
    si.camera_info.tf = Transform::LookAt(position, target, y_axis);
    si.camera_info.fov = fov;
    si.camera_info.aperture = aperture;
    si.camera_info.focus_distance = Dist(position, target);
    si.camera_info.film_info.filename = "";
    si.camera_info.film_info.resolution = { width, height };
    si.camera_info.sampler_info.type = SamplerType::stratified;
    si.camera_info.sampler_info.spp = 64;

    return si;
}

static int32 index1 = Sample::Register("sss", SSSTest);
static int32 index2 = Sample::Register("sss2", SSSTest2);
