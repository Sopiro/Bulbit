#include "../samples.h"

std::unique_ptr<Camera> CornellBox(Scene& scene)
{
    // auto normalmap = CreateSpectrumImageTexture(scene, "res/bistro/Concrete_Normal.png", true);

    // Materials
    auto red = CreateDiffuseMaterial(scene, Spectrum(.65f, .05f, .05f));
    auto green = CreateDiffuseMaterial(scene, Spectrum(.12f, .45f, .15f));
    auto blue = CreateDiffuseMaterial(scene, Spectrum(.22f, .23f, .75f));
    auto white = CreateDiffuseMaterial(scene, Spectrum(.73f, .73f, .73f));
    auto box = CreateDiffuseMaterial(scene, Spectrum(.73f, .73f, .73f));
    auto wakgood_mat = CreateDiffuseMaterial(scene, "res/wakdu.jpg");
    auto light = CreateDiffuseLightMaterial(scene, Spectrum(15.0f));
    // auto light = CreateDiffuseLightMaterial(scene, Spectrum(17.0f, 12.0f, 4.0f));
    auto mirror = CreateMirrorMaterial(scene, Spectrum(0.73f));
    auto mix = CreateMixtureMaterial(scene, red, blue, 0.5f);
    auto ss = CreateSubsurfaceRandomWalkMaterial(scene, Spectrum(1.0), Spectrum(0.5, 0.25, 0.125) * 0.03, 1.0f, 0.0f);
    auto glass = CreateDielectricMaterial(scene, 1.5f);
    auto rough_glass = CreateDielectricMaterial(scene, 1.5f, 0.1f);
    auto gold = CreateConductorMaterial(scene, { 0.161, 0.492, 1.426 }, { 4.08769, 2.32625, 1.846 }, 0.1f);
    auto coated_gold = CreateLayeredMaterial(scene, glass, gold);

    auto left_box = white;
    auto right_box = white;

    // Cornell box
    {
        // front
        auto tf = Transform{ Vec3(0.5f, 0.5f, -1.0f), identity, Vec3(1.0f) };
        CreateRectXY(scene, tf, wakgood_mat);

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
    }

    // Left block
    {
        Float hx = 0.14f;
        Float hy = 0.28f;
        Float hz = 0.14f;

        auto tf = Transform{ 0.33f, hy, -0.66f, Quat(DegToRad(18.0f), y_axis), Vec3(hx * 2.0f, hy * 2.0f, hz * 2.0f) };
        CreateBox(scene, tf, left_box);
    }

    // Right block
    {
        Float hx = 0.14f;
        Float hy = 0.14f;
        Float hz = 0.14f;

        // auto mat = CreateThinDielectricMaterial(scene, 1.5f);

        auto tf = Transform{ 0.66f, hy, -0.33f, Quat(DegToRad(-18.0f), y_axis), Vec3(hx * 2.0f, hy * 2.0f, hz * 2.0f) };
        CreateBox(scene, tf, right_box);
    }

    // Right sphere
    {
        // auto mat = CreateDielectricMaterial(scene, 1.5f);
        // auto mat = CreateThinDielectricMaterial(scene, 1.5f);
        // auto mat = CreateConductorMaterial(scene,
        //     CreateSpectrumConstantTexture(scene, 0.1, 0.2, 1.9), CreateSpectrumConstantTexture(scene, 3, 2.5, 2),
        //     CreateFloatConstantTexture(scene, 0.3f), CreateFloatConstantTexture(scene, 0.1f));
        // CreateSphere(scene, Transform(Vec3(0.65f, 0.15f, -0.3f), Quat(DegToRad(0), x_axis)), 0.15f, mat);
        // CreateSphere(scene, Transform(Vec3(0.65f, 0.15f, -0.3f), Quat(DegToRad(45), y_axis)), 0.15f, wakgood_mat);
    }

    // Lights
    {
        auto tf = Transform{ 0.5f, 0.995f, -0.5f, Quat(pi, x_axis), Vec3(0.25f) };
        CreateRectXZ(scene, tf, light);

        // CreateSphere(scene, Vec3(0.5f, 0.9f, -0.5f), 0.05f, light);
        // CreatePointLight(scene, Point3(0.5f, 0.9f, -0.5f), Spectrum(0.25f));
        // CreateDirectionalLight(scene,  Normalize(-Vec3(1, 1, 1)), Vec3(5.0f), 0.05f);
        // CreateImageInfiniteLight(scene, "res/HDR/quarry_04_puresky_1k.hdr");
        // CreateImageInfiniteLight(scene, "res/solitude_night_4k/solitude_night_4k.hdr");
        // CreateImageInfiniteLight(scene, "res/HDR/sunflowers_puresky_1k.hdr");
        // CreateImageInfiniteLight(scene, "res/HDR/san_giuseppe_bridge_4k.hdr", Transform(Quat(pi, y_axis)));
    }

    // std::cout << "Lights: " << scene.GetLights().size() << std::endl;

    int32 width = 500;

    Point3 lookfrom{ 0.5f, 0.5f, 2.05f };
    Point3 lookat{ 0.5f, 0.5f, 0.0f };

    Float dist_to_focus = Dist(lookfrom, lookat);
    Float aperture = 0.0f;
    Float vFov = 28.0f;

    // return std::make_unique<OrthographicCamera>(lookfrom, lookat, y_axis, Point2(1.1, 1.1), width);
    // return std::make_unique<SphericalCamera>(lookfrom, Point2i(width, width));
    return std::make_unique<PerspectiveCamera>(lookfrom, lookat, y_axis, vFov, aperture, dist_to_focus, Point2i(width, width));
}

static int32 sample_index = Sample::Register("cornell-box", CornellBox);
