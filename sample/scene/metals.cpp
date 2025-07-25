#include "../samples.h"

std::unique_ptr<Camera> Metals(Scene& scene)
{
    HomogeneousMedium* hm = scene.CreateMedium<HomogeneousMedium>(Spectrum(0, 0, 0), Spectrum(10), Spectrum(0.0), -0.9f);
    MediumInterface mi(hm, nullptr);

    SetLoaderUseForceFallbackMaterial(true);

    // Floor
    {
        auto checker = CreateSpectrumCheckerTexture(scene, 0.75, 0.3, Point2(60));
        auto tf = Transform{ Vec3(0, 0, 0), Quat::FromEuler({ 0, 0, 0 }), Vec3(10) };
        auto floor = scene.CreateMaterial<DiffuseMaterial>(checker);
        SetLoaderFallbackMaterial(floor);
        LoadModel(scene, "res/background.obj", tf);
    }

    int32 w = 5;
    int32 h = 1;

    Vec3 o(0, 0, 0.4);
    Vec3 x(0.8, 0, 0);
    Vec3 y(0, 0, 1);

    const int32 count = 5;

    const Material* outers[count];

    Float metallic = 0.0f;
    Float transmission = 1.0f;
    Float anisotrophy = 0.0f;
    Float ior = 1.5f;
    bool energy_compensation = true;

    Float roughness = 0.15f;

    // https://refractiveindex.info/
    const Material* iron = CreateConductorMaterial(scene, { 2.8653, 2.8889, 2.6260 }, { 3.1820, 2.9164, 2.7925 }, roughness);
    const Material* silver = CreateConductorMaterial(scene, { 0.26978, 0.24580, 0.23222 }, { 4.8078, 3.3897, 2.7380 }, roughness);
    const Material* gold = CreateConductorMaterial(scene, { 0.161, 0.492, 1.426 }, { 4.08769, 2.32625, 1.846 }, roughness);
    const Material* platinum =
        CreateConductorMaterial(scene, { 0.49745, 0.48267, 0.60399 }, { 6.9266, 4.8444, 3.8895 }, roughness);
    const Material* brass = CreateConductorMaterial(scene, { 0.446, 0.573, 0.994 }, { 4.1060, 2.5680, 1.883 }, roughness);
    const Material* glass = CreateDielectricMaterial(scene, 1.0f);
    const Material* air = CreateDielectricMaterial(scene, 1.0f);
    const Material* copper = CreateConductorMaterial(scene, { 0.21100, 1.1274, 1.2444 }, { 4.1592, 2.5978, 2.433 }, roughness);
    const Material* patina = CreateLayeredMaterial(scene, air, copper, true, { 0.25f, 0.95f, 0.45f }, 0.2, 0.1f);

    outers[3] = iron;
    outers[1] = silver;
    outers[0] = gold;
    outers[2] = copper;
    outers[4] = patina;

    const Material* inners[count];
    for (int32 i = 0; i < count; ++i)
    {
        inners[i] = outers[i];
    }

    for (int32 j = 0; j < h; ++j)
    {
        for (int32 i = 0; i < w; ++i)
        {
            int32 sign_i = std::pow<int32>(-1, i);
            int32 sign_j = std::pow<int32>(-1, j);
            Vec3 p = o + (sign_j * y * ((j + 1) / 2)) + (sign_i * x * ((i + 1) / 2));

            // SetLoaderFallbackMediumInterface(mi);

            SetLoaderFallbackMaterial(outers[std::min(i + j * w, count)]);

            // https://github.com/lighttransport/lighttransportequation-orb
            auto tf = Transform{ p, Quat::FromEuler({ 0, 0, 0 }), Vec3(2.0f) };
            LoadModel(scene, "res/mori_knob/base.obj", tf);

            LoadModel(scene, "res/mori_knob/outer.obj", tf);
            tf = Transform{ p, Quat::FromEuler({ 0, 0, 0 }), Vec3(2.1f) };
            LoadModel(scene, "res/mori_knob/inner.obj", tf);
            LoadModel(scene, "res/mori_knob/equation.obj", tf);
        }
    }

    // CreateUniformInfiniteLight(scene, Spectrum(1));
    CreateImageInfiniteLight(scene, "res/HDR/aerodynamics_workshop_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/peppermint_powerplant_4k.hdr", Transform(Quat(pi / 2, y_axis)));

    Float aspect_ratio = 4.f / 1.f;
    // Float aspect_ratio = 9.f / 16.f;
    // Float aspect_ratio = 3.f / 2.f;
    // Float aspect_ratio = 4.f / 3.f;
    // Float aspect_ratio = 1.f;
    int32 width = 1600;
    int32 height = int32(width / aspect_ratio);

    Point3 lookfrom = Point3{ 0, 1.0, 2.28 };
    Point3 lookat = Point3{ 0.0, 0.1, 0.0 };

    Float dist_to_focus = Dist(lookfrom, lookat);
    Float aperture = 0.0f;
    Float vFov = 30.0;

    return std::make_unique<PerspectiveCamera>(lookfrom, lookat, y_axis, vFov, aperture, dist_to_focus, Point2i(width, height));
}

static int32 index0 = Sample::Register("metals", Metals);
