#include "../samples.h"

void CornellBoxCaustics(RendererInfo* ri)
{
    Scene& scene = ri->scene;

    // Materials
    auto red = CreateDiffuseMaterial(scene, Spectrum(.65f, .05f, .05f));
    auto green = CreateDiffuseMaterial(scene, Spectrum(.12f, .45f, .15f));
    auto blue = CreateDiffuseMaterial(scene, Spectrum(.22f, .23f, .75f));
    auto white = CreateDiffuseMaterial(scene, Spectrum(.73f, .73f, .73f));
    auto box = CreateDiffuseMaterial(scene, Spectrum(.73f, .73f, .73f));
    auto mirror = CreateMirrorMaterial(scene, Spectrum(0.73f));
    auto mix = CreateMixtureMaterial(scene, red, blue, 0.5f);
    auto ss = CreateSubsurfaceRandomWalkMaterial(scene, Spectrum(1.0), Spectrum(0.5, 0.25, 0.125) * 0.03, 1.0f, 0.0f);
    auto glass = CreateDielectricMaterial(scene, 1.5f);
    auto water = CreateDielectricMaterial(scene, 1.333f);
    auto rough_glass = CreateDielectricMaterial(scene, 1.5f, 0.2f);
    auto gold = CreateConductorMaterial(scene, { 0.161, 0.492, 1.426 }, { 4.08769, 2.32625, 1.846 }, 0.1f);
    auto coated_gold = CreateLayeredMaterial(scene, glass, gold);

    auto left_box = white;
    auto right_box = white;

    // Caustics
    // https://benedikt-bitterli.me/resources/
    {
        ModelLoaderOptions options;
        options.use_fallback_material = true;
        auto tf = Transform{ .5f, 0.8f, -.5f, Quat::FromEuler({ 0, 0, 0 }), Vec3(1, 1, 1) };
        options.fallback_material = water;
        LoadModel(scene, "res/caustics/water.obj", tf, options);
    }

    // Cornell box
    {
        // front
        auto tf = Transform{ Vec3(0.5f, 0.5f, -1.0f), identity, Vec3(1.0f) };
        CreateRectXY(scene, tf, white);

        // left
        tf = Transform{ Vec3(0.0f, 0.5f, -0.5f), identity, Vec3(1.0f) };
        CreateRectYZ(scene, tf, red);

        // right
        tf = Transform{ Vec3(1.0f, 0.5f, -0.5f), Quat(pi, y_axis), Vec3(1.0f) };
        CreateRectYZ(scene, tf, green);

        // bottom
        tf = Transform{ Vec3(0.5f, 0.0f, -0.5f), identity, Vec3(1.0f) };
        CreateRectXZ(scene, tf, white);

        // top
        tf = Transform{ Vec3(0.5f, 1.0f, -0.5f), Quat(pi, x_axis), Vec3(1.0f) };
        CreateRectXZ(scene, tf, white);

        // back
        tf = Transform{ Vec3(0.5f, 0.5f, 0.0f), Quat(pi, y_axis), Vec3(1.0f) };
        CreateRectXY(scene, tf, glass);
    }

    // Left block
    {
        Float hx = 0.14f;
        Float hy = 0.28f;
        Float hz = 0.14f;

        auto tf = Transform{ 0.33f, hy, -0.66f, Quat(DegToRad(18.0f), y_axis), Vec3(hx * 2.0f, hy * 2.0f, hz * 2.0f) };
        // CreateBox(scene, tf, left_box);
    }

    // Right block
    {
        Float hx = 0.14f;
        Float hy = 0.14f;
        Float hz = 0.14f;

        auto tf = Transform{ 0.66f, hy, -0.33f, Quat(DegToRad(-18.0f), y_axis), Vec3(hx * 2.0f, hy * 2.0f, hz * 2.0f) };
        // CreateBox(scene, tf, right_box);
    }

    // Right sphere
    {
        // auto mat = CreateDielectricMaterial(scene, 1.5f);
        // auto mat = CreateThinDielectricMaterial(scene, 1.5f);
        // auto mat = CreateConductorMaterial(scene,
        //     CreateSpectrumConstantTexture(scene, 0.1, 0.2, 1.9), CreateSpectrumConstantTexture(scene, 3, 2.5, 2),
        //     CreateFloatConstantTexture(scene, 0.3f), CreateFloatConstantTexture(scene, 0.1f));
        // CreateSphere(scene, Transform(Vec3(0.65f, 0.15f, -0.3f), Quat(DegToRad(0), x_axis)), 0.15f, mirror);
    }

    // Lights
    {
        auto tf = Transform{ 0.5f, 0.995f, -0.5f, Quat(pi, x_axis), Vec3(0.002f) };
        CreateRectXZ(scene, tf, white, {}, AreaLightInfo{ .emission = 250000.0f });

        // CreatePointLight(scene, Point3(0.5f, 0.999f, -0.5f), Spectrum(0.25f));
    }

    int32 width = 500;

    Point3 position{ 0.5f, 0.5f, 2.05f };
    Point3 target{ 0.5f, 0.5f, 0.0f };

    ri->integrator_info.type = IntegratorType::sppm;
    ri->integrator_info.n_photons = 100000;
    ri->integrator_info.initial_radius = -1;
    ri->integrator_info.max_bounces = 64;
    ri->camera_info.type = CameraType::perspective;
    ri->camera_info.transform = Transform::LookAt(position, target, y_axis);
    ri->camera_info.fov = 28;
    ri->camera_info.aperture_radius = 0;
    ri->camera_info.focus_distance = Dist(position, target);
    ri->camera_info.film_info.filename = "";
    ri->camera_info.film_info.resolution = { width, width };
    ri->camera_info.sampler_info.type = SamplerType::independent;
    ri->camera_info.sampler_info.spp = 64;
}

static int32 sample_index = Sample::Register("cornell-box-caustics", CornellBoxCaustics);
