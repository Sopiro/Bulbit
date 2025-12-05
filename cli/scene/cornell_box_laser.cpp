#include "../samples.h"

void CornellBoxLaser(RendererInfo* ri)
{
    Scene& scene = ri->scene;

    // Materials
    auto red = CreateDiffuseMaterial(scene, Spectrum(.65f, .05f, .05f));
    auto green = CreateDiffuseMaterial(scene, Spectrum(.12f, .45f, .15f));
    auto blue = CreateDiffuseMaterial(scene, Spectrum(.22f, .23f, .75f));
    auto white = CreateDiffuseMaterial(scene, Spectrum(.73f, .73f, .73f));
    auto glass = CreateDielectricMaterial(scene, 1.5f);
    auto mirror = CreateMirrorMaterial(scene, Spectrum(0.73f));
    auto mix = CreateMixtureMaterial(scene, red, blue, 0.5f);

    Spectrum sigma_a(0);
    Spectrum sigma_s(.5f);

    HomogeneousMedium* hm = scene.CreateMedium<HomogeneousMedium>(sigma_a, sigma_s, Spectrum(0.0), 0.0f);

    MediumInterface mi_outside(nullptr, hm);
    MediumInterface mi_inside(hm, nullptr);
    MediumInterface mi_two_sided(hm, hm);

    // Cornell box
    {
        // front
        auto tf = Transform{ Vec3(0.5f, 0.5f, -1.0f), identity, Vec3(1.0f) };
        CreateRectXY(scene, tf, white, mi_outside);

        // left
        tf = Transform{ Vec3(0.0f, 0.5f, -0.5f), identity, Vec3(1.0f) };
        CreateRectYZ(scene, tf, red, mi_outside);

        // right
        tf = Transform{ Vec3(1.0f, 0.5f, -0.5f), Quat(pi, y_axis), Vec3(1.0f) };
        CreateRectYZ(scene, tf, green, mi_outside);

        // bottom
        tf = Transform{ Vec3(0.5f, 0.0f, -0.5f), identity, Vec3(1.0f) };
        CreateRectXZ(scene, tf, white, mi_outside);

        // top
        tf = Transform{ Vec3(0.5f, 1.0f, -0.5f), Quat(pi, x_axis), Vec3(1.0f) };
        CreateRectXZ(scene, tf, white, mi_outside);

        // back
        tf = Transform{ Vec3(0.5f, 0.5f, 0.0f), Quat(pi, y_axis), Vec3(1.0f) };
        CreateRectXY(scene, tf, nullptr, mi_outside);
    }

    {
#if 0
        ModelLoaderOptions options;
        options.use_fallback_material = true;
        auto tf = Transform{ .5f, 0.3f, -.5f, Quat::FromEuler({ 0, 0, 0 }), Vec3(1, 1, 1) };
        auto water = CreateDielectricMaterial(scene, 1.333f);
        options.fallback_material = water;
        options.fallback_medium_interface = mi_two_sided;
        LoadModel(scene, "res/caustics/water.obj", tf, options);
#else
        auto tf = Transform{ Vec3(0.5f, 0.3f, -0.5f), identity, Vec3(1.0f) };
        CreateRectXZ(scene, tf, glass, mi_two_sided);
#endif
    }

    // Lights
    {
        auto tf = Transform{ 0.5f, 0.995f, -0.5f, Quat(pi, x_axis), Vec3(0.7f) };
        // CreateRectXZ(scene, tf, white, mi_two_sided, AreaLightInfo{ .emission = 2.0f });

        tf = Transform::LookAt({ 0.99, 0.5, -0.5 }, { 0.5, 0.3, -0.5 }, y_axis) * Transform::Scale(Vec3{ 0.01f });
        CreateRectXY(scene, tf, white, mi_two_sided, AreaLightInfo{ .type = AreaLightType::directional, .emission = 50000.0f });

        // int32 s = 10;
        // Float d = 1.0f / s;

        // for (int32 j = 1; j < s; ++j)
        // {
        //     for (int32 i = 1; i < s; ++i)
        //     {
        //         Float x = i * d;
        //         Float z = -j * d;
        //         Float y = .99f;

        //         tf = Transform::LookAt({ x, y, z }, { x, 0, z }, z_axis) * Transform::Scale(Vec3{ 0.005f });
        //         CreateRectXY(
        //             scene, tf, white, mi_two_sided, AreaLightInfo{ .type = AreaLightType::directional, .emission = 3000.0f }
        //         );
        //     }
        // }
    }

    int32 width = 500;

    Point3 position{ 0.5f, 0.5f, 2.05f };
    Point3 target{ 0.5f, 0.5f, 0.0f };

    ri->integrator_info.type = IntegratorType::vol_bdpt;
    ri->integrator_info.max_bounces = 8;
    ri->camera_info.type = CameraType::perspective;
    // ri->camera_info.type = CameraType::orthographic;
    ri->camera_info.viewport_size = { 1.05, 1.05 };
    ri->camera_info.transform = Transform::LookAt(position, target, y_axis);
    ri->camera_info.fov = 28;
    ri->camera_info.aperture_radius = 0;
    ri->camera_info.focus_distance = Dist(position, target);
    ri->camera_info.film_info.filename = "";
    ri->camera_info.film_info.resolution = { width, width };
    ri->camera_info.sampler_info.type = SamplerType::stratified;
    ri->camera_info.sampler_info.spp = 64;
}

void CornellBoxLaser2(RendererInfo* ri)
{
    Scene& scene = ri->scene;

    // Materials
    auto red = CreateDiffuseMaterial(scene, Spectrum(.65f, .05f, .05f));
    auto green = CreateDiffuseMaterial(scene, Spectrum(.12f, .45f, .15f));
    auto blue = CreateDiffuseMaterial(scene, Spectrum(.22f, .23f, .75f));
    auto white = CreateDiffuseMaterial(scene, Spectrum(.73f, .73f, .73f));
    auto glass = CreateDielectricMaterial(scene, 1.5f);
    auto mirror = CreateMirrorMaterial(scene, Spectrum(0.73f));
    auto mix = CreateMixtureMaterial(scene, red, blue, 0.5f);

    Spectrum sigma_a(0);
    Spectrum sigma_s(.5f);

    HomogeneousMedium* hm = scene.CreateMedium<HomogeneousMedium>(sigma_a, sigma_s, Spectrum(0.0), 0.0f);

    MediumInterface mi_outside(nullptr, hm);
    MediumInterface mi_inside(hm, nullptr);
    MediumInterface mi_two_sided(hm, hm);

    // Cornell box
    {
        // front
        auto tf = Transform{ Vec3(0.5f, 0.5f, -1.0f), identity, Vec3(1.0f) };
        CreateRectXY(scene, tf, white, mi_outside);

        // left
        tf = Transform{ Vec3(0.0f, 0.5f, -0.5f), identity, Vec3(1.0f) };
        CreateRectYZ(scene, tf, red, mi_outside);

        // right
        tf = Transform{ Vec3(1.0f, 0.5f, -0.5f), Quat(pi, y_axis), Vec3(1.0f) };
        CreateRectYZ(scene, tf, green, mi_outside);

        // bottom
        tf = Transform{ Vec3(0.5f, 0.0f, -0.5f), identity, Vec3(1.0f) };
        CreateRectXZ(scene, tf, white, mi_outside);

        // top
        tf = Transform{ Vec3(0.5f, 1.0f, -0.5f), Quat(pi, x_axis), Vec3(1.0f) };
        CreateRectXZ(scene, tf, white, mi_outside);

        // back
        tf = Transform{ Vec3(0.5f, 0.5f, 0.0f), Quat(pi, y_axis), Vec3(1.0f) };
        CreateRectXY(scene, tf, nullptr, mi_outside);
    }

    {
        ModelLoaderOptions options;
        options.use_fallback_material = true;
        auto tf = Transform{ .5f, 0.7f, -.5f, Quat::FromEuler({ 0, 0, 0 }), Vec3(1, 1, 1) };
        auto water = CreateDielectricMaterial(scene, 1.333f);
        options.fallback_material = water;
        options.fallback_medium_interface = mi_two_sided;
        LoadModel(scene, "res/caustics/water.obj", tf, options);
    }

    // Lights
    {
        auto tf = Transform{ 0.5f, 0.995f, -0.5f, Quat(pi, x_axis), Vec3(.7f) };
        CreateRectXZ(scene, tf, white, mi_two_sided, AreaLightInfo{ .type = AreaLightType::directional, .emission = 5.f });
    }

    int32 width = 500;

    Point3 position{ 0.5f, 0.5f, 2.05f };
    Point3 target{ 0.5f, 0.5f, 0.0f };

    ri->integrator_info.type = IntegratorType::vol_sppm;
    ri->integrator_info.sample_direct_light = false;
    ri->integrator_info.max_bounces = 8;
    ri->camera_info.type = CameraType::perspective;
    // ri->camera_info.type = CameraType::orthographic;
    ri->camera_info.viewport_size = { 1.05, 1.05 };
    ri->camera_info.transform = Transform::LookAt(position, target, y_axis);
    ri->camera_info.fov = 28;
    ri->camera_info.aperture_radius = 0;
    ri->camera_info.focus_distance = Dist(position, target);
    ri->camera_info.film_info.filename = "";
    ri->camera_info.film_info.resolution = { width, width };
    ri->camera_info.sampler_info.type = SamplerType::stratified;
    ri->camera_info.sampler_info.spp = 64;
}

static int32 sample_index = Sample::Register("cornell-box-laser", CornellBoxLaser);
static int32 sample_index2 = Sample::Register("cornell-box-laser2", CornellBoxLaser2);
