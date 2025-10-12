#include "../samples.h"

void CornellBoxFog(RendererInfo* ri)
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

    auto left = glass;
    auto right = white;

    Spectrum sigma_a(0);
    Spectrum sigma_s(1.0f);

    HomogeneousMedium* hm = scene.CreateMedium<HomogeneousMedium>(sigma_a, sigma_s, Spectrum(0.0), 0.0f);

    MediumInterface mi_outside(nullptr, hm);
    MediumInterface mi_inside(hm, nullptr);
    MediumInterface mi_two_sided(hm, hm);

    Point3 p_light(0.5f, 0.9f, -0.5f);
    Point3 p_corner(0, 0, -1);

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

    // Left Sphere
    {
        Float r = 1.0f / 3 / 2;

        Vec3 w = Normalize(p_corner - p_light);
        auto tf = Transform{ p_light + w * 2 * r };
        CreateSphere(scene, tf, r, left, mi_outside);
    }

    // Right block
    {
        Float hx = 0.14f;
        Float hy = 0.14f;
        Float hz = 0.14f;

        auto tf = Transform{ 0.66f, hy + Ray::epsilon * 2, -0.33f, Quat(DegToRad(-18.0f), y_axis),
                             Vec3(hx * 2.0f, hy * 2.0f, hz * 2.0f) };
        CreateBox(scene, tf, right, mi_outside);
    }

    // Lights
    {
        auto tf = Transform{ 0.5f, 0.995f, -0.5f, Quat(pi, x_axis), Vec3(0.25f) };
        CreateRectXZ(scene, tf, white, mi_two_sided, AreaLightInfo{ .emission = 15.0f });

        // CreatePointLight(scene, Point3(0.5f, 0.9f, -0.5f), Spectrum(0.25f));
    }

    int32 width = 500;

    Point3 position{ 0.5f, 0.5f, 2.05f };
    Point3 target{ 0.5f, 0.5f, 0.0f };

    ri->integrator_info.type = IntegratorType::vol_bdpt;
    ri->integrator_info.max_bounces = 8;
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

void CornellBoxFog2(RendererInfo* ri)
{
    Scene& scene = ri->scene;

    // Materials
    auto red = CreateDiffuseMaterial(scene, Spectrum(.65f, .05f, .05f));
    auto green = CreateDiffuseMaterial(scene, Spectrum(.12f, .45f, .15f));
    auto blue = CreateDiffuseMaterial(scene, Spectrum(.22f, .23f, .75f));
    auto white = CreateDiffuseMaterial(scene, Spectrum(.73f, .73f, .73f));
    auto glass = CreateDielectricMaterial(scene, 1.5f);
    auto rough_glass = CreateDielectricMaterial(scene, 1.5f, 0.1f);
    auto mirror = CreateMirrorMaterial(scene, Spectrum(1));
    auto mix = CreateMixtureMaterial(scene, red, blue, 0.5f);

    Spectrum sigma_a(0);
    Spectrum sigma_s(1.0f);

    HomogeneousMedium* hm = scene.CreateMedium<HomogeneousMedium>(sigma_a, sigma_s, Spectrum(0.0), 0.0f);

    MediumInterface mi_outside(nullptr, hm);
    MediumInterface mi_inside(hm, nullptr);
    MediumInterface mi_two_sided(hm, hm);

    Point3 p_light(0.5f, 0.995f, -0.5f);
    Point3 p_corner_1(0.1, 0, -0.25);
    Point3 p_corner_2(1, 0, -0.75);

    Vec3 w1 = p_corner_1 - p_light;
    Vec3 w2 = p_corner_2 - p_light;
    Float d1 = w1.Normalize();
    Float d2 = w2.Normalize();

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

    // Left sphere
    Float r1 = 0.15f;
    Point3 s1 = p_light + w1 * 0.4f * d1;
    CreateSphere(scene, s1, r1, glass, mi_outside);

    // Right sphere
    Float r2 = 0.1f;
    Point3 s2 = p_light + w2 * 0.2 * d2;
    CreateSphere(scene, s2, r2, glass, mi_outside);

    Float r3 = 0.05f;
    Point3 s3 = p_light + w2 * 0.5f * d2;
    CreateSphere(scene, s3, r3, glass, mi_outside);

    Point3 s4 = p_light + w2 * 0.7f * d2;
    Vec3 w3 = Normalize(s1 - s4);
    Vec3 n = Normalize(-w2 + w3);

    auto tf = Transform{ s4, Quat(n, y_axis), Point3(0.1f) };
    CreateRectXY(scene, tf, mirror, mi_two_sided);

    // Point3 s5 = p_light + w1 * 0.7f * d1;
    // Vec3 w4 = Normalize(s3 - s5);
    // n = Normalize(-w1 + w4);
    // tf = Transform{ s5, Quat(n, y_axis), Point3(0.1f) };
    // CreateRectXY(scene, tf, mirror, mi_two_sided);

    // Lights
    {
        auto tf = Transform{ p_light, Quat(pi, x_axis), Vec3(0.001f) };
        CreateRectXZ(scene, tf, white, mi_two_sided, AreaLightInfo{ .emission = 1000000.0f });

        // CreatePointLight(scene, Point3(0.5f, 0.9f, -0.5f), Spectrum(0.25f));
    }

    int32 width = 500;

    Point3 position{ 0.5f, 0.5f, 2.05f };
    Point3 target{ 0.5f, 0.5f, 0.0f };

    ri->integrator_info.type = IntegratorType::vol_bdpt;
    ri->integrator_info.max_bounces = 8;
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

void CornellBoxFog3(RendererInfo* ri)
{
    Scene& scene = ri->scene;

    // Materials
    auto red = CreateDiffuseMaterial(scene, Spectrum(.65f, .05f, .05f));
    auto green = CreateDiffuseMaterial(scene, Spectrum(.12f, .45f, .15f));
    auto blue = CreateDiffuseMaterial(scene, Spectrum(.22f, .23f, .75f));
    auto white = CreateDiffuseMaterial(scene, Spectrum(.73f, .73f, .73f));
    auto glass = CreateDielectricMaterial(scene, 1.5f);
    auto rough_glass = CreateDielectricMaterial(scene, 1.5f, 0.1f);
    auto mirror = CreateMirrorMaterial(scene, Spectrum(1));
    auto mix = CreateMixtureMaterial(scene, red, blue, 0.5f);

    Spectrum sigma_a(0);
    Spectrum sigma_s(.3f);

    HomogeneousMedium* hm = scene.CreateMedium<HomogeneousMedium>(sigma_a, sigma_s, Spectrum(0.0), 0.0f);

    MediumInterface mi_outside(nullptr, hm);
    MediumInterface mi_inside(hm, nullptr);
    MediumInterface mi_two_sided(hm, hm);

    Point3 p_light(0.5f, 0.995f, -0.5f);
    Point3 p_corner_1(0.1, 0, -0.25);
    Point3 p_corner_2(1, 0, -0.75);

    Vec3 w1 = p_corner_1 - p_light;
    Vec3 w2 = p_corner_2 - p_light;
    Float d1 = w1.Normalize();
    Float d2 = w2.Normalize();

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

    // Left sphere
    Float r1 = 0.15f;
    Point3 s1 = p_light + w1 * 0.4f * d1;
    CreateSphere(scene, s1, r1, glass, mi_outside);

    // Right sphere
    Float r2 = 0.1f;
    Point3 s2 = p_light + w2 * 0.2 * d2;
    CreateSphere(scene, s2, r2, glass, mi_outside);

    Float r3 = 0.05f;
    Point3 s3 = p_light + w2 * 0.5f * d2;
    CreateSphere(scene, s3, r3, glass, mi_outside);

    Point3 s4 = p_light + w2 * 0.7f * d2;
    Vec3 w3 = Normalize(s1 - s4);
    Vec3 n = Normalize(-w2 + w3);

    auto tf = Transform{ s4, Quat(n, y_axis), Point3(0.1f) };
    CreateRectXY(scene, tf, mirror, mi_two_sided);

    // Point3 s5 = p_light + w1 * 0.7f * d1;
    // Vec3 w4 = Normalize(s3 - s5);
    // n = Normalize(-w1 + w4);
    // tf = Transform{ s5, Quat(n, y_axis), Point3(0.1f) };
    // CreateRectXY(scene, tf, mirror, mi_two_sided);

    // Lights
    {
        auto tf = Transform{ p_light, Quat(pi, x_axis), Vec3(0.1f) };
        CreateRectXZ(scene, tf, white, mi_two_sided, AreaLightInfo{ .is_directional = true, .emission = 500.0f });

        // CreatePointLight(scene, Point3(0.5f, 0.9f, -0.5f), Spectrum(0.25f));
    }

    int32 width = 500;

    Point3 position{ 0.5f, 0.5f, 2.05f };
    Point3 target{ 0.5f, 0.5f, 0.0f };

    ri->integrator_info.type = IntegratorType::vol_bdpt;
    ri->integrator_info.max_bounces = 16;
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

static int32 sample_index1 = Sample::Register("cornell-box-fog", CornellBoxFog);
static int32 sample_index2 = Sample::Register("cornell-box-fog2", CornellBoxFog2);
static int32 sample_index3 = Sample::Register("cornell-box-fog3", CornellBoxFog3);
