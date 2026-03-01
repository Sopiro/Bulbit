#include "../samples.h"

void SDSTest(RendererInfo* ri)
{
    Scene& scene = ri->scene;

    HomogeneousMedium* hm = scene.CreateMedium<HomogeneousMedium>(Spectrum(0, 0, 0), Spectrum(10), Spectrum(0.0), -0.9f);
    MediumInterface mi(hm, nullptr);

    ModelLoaderOptions options;
    options.use_fallback_material = true;

    // Floor
    {
        auto checker = CreateSpectrumCheckerTexture(scene, 0.75, 0.3, Point2(20));
        auto tf = Transform{ Vec3(0, 0, 0), Quat::FromEuler({ 0, 0, 0 }), Vec3(3) };
        auto floor = scene.CreateMaterial<DiffuseMaterial>(checker);
        options.fallback_material = floor;
        LoadModel(scene, "res/background.obj", tf, options);
    }

    // Model
    {
        auto glass = CreateDielectricMaterial(scene, 1.5f, 0.0f);
        auto diffuse = CreateDiffuseMaterial(scene, Spectrum(0.9, 0.5, 0.6));
        // Srand(1213212);
        // auto mat = CreateRandomPrincipledMaterial(scene);

        options.fallback_material = diffuse;

        // auto tf = Transform{ Vec3(0.2, .8, .3) * 0.5, Quat::FromEuler({ 0, (pi / 4), 0 }) };
        // LoadModel(scene, "res/xyzdragon.obj", tf, options);
        auto tf = Transform{ Vec3(0, 0.2f, 0), Quat::FromEuler({ 0, 0, 0 }), Vec3(0.5f) };
        LoadModel(scene, "res/stanford/bunny.obj", tf, options);

        CreateBox(scene, Transform{ Vec3(0, 0.5f, 0), identity, Vec3(0.7f) }, glass);
    }

    // CreateImageInfiniteLight(scene, "res/HDR/san_giuseppe_bridge_1k.hdr", Transform(Quat(-pi, y_axis)));

    {
        auto white = CreateDiffuseMaterial(scene, 1.0f);
        auto tf = Transform{ 0, 2.0f, 0, Quat(pi, x_axis), Vec3(1.5f) };
        // CreateRectXZ(scene, tf, white, {}, AreaLightInfo{ .type = AreaLightType::spot, .emission = 5.0f });
        CreateSpotLight(scene, Point3(2, 4, -1.5), Point3(2, 4, -1.5), 30, 45, 30);
    }

    Float aspect_ratio = 16.f / 9.f;
    // Float aspect_ratio = 9.f / 16.f;
    // Float aspect_ratio = 3.f / 2.f;
    // Float aspect_ratio = 4.f / 3.f;
    // Float aspect_ratio = 1.f;
    int32 width = 800;
    int32 height = int32(width / aspect_ratio);

    Point3 position{ 0, 1.0, 3 };
    Point3 target{ 0.0, 0.3, 0.0 };

    Float aperture = 0.01f;
    Float fov = 30.0;

    ri->integrator_info.type = IntegratorType::vcm;
    ri->integrator_info.max_bounces = 64;
    ri->camera_info.type = CameraType::perspective;
    ri->camera_info.transform = Transform::LookAt(position, target, y_axis);
    ri->camera_info.fov = fov;
    ri->camera_info.aperture_radius = aperture;
    ri->camera_info.focus_distance = Dist(position, target);
    ri->camera_info.film_info.filename = "";
    ri->camera_info.film_info.resolution = { width, height };
    ri->camera_info.sampler_info.type = SamplerType::stratified;
    ri->camera_info.sampler_info.spp = 64;
}

static int32 index1 = Sample::Register("sds", SDSTest);