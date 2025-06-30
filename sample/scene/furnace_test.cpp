#include "../samples.h"

std::unique_ptr<Camera> FurnacePrincipled(Scene& scene)
{
    HomogeneousMedium* hm = scene.CreateMedium<HomogeneousMedium>(Spectrum(0, 0, 0), Spectrum(10), Spectrum(0.0), -0.9f);
    MediumInterface mi(hm, nullptr);

    SetLoaderUseForceFallbackMaterial(true);

    // Floor
    {
        // auto checker = CreateSpectrumCheckerTexture(scene, 0.75, 0.3, Point2(60));
        // auto tf = Transform{ Vec3(0, 0, 0), Quat::FromEuler({ 0, 0, 0 }), Vec3(10) };
        // auto floor = scene.CreateMaterial<DiffuseMaterial>(checker);
        // SetLoaderFallbackMaterial(floor);
        // LoadModel(scene, "res/background.obj", tf);
    }

    int32 w = 5;
    int32 h = 1;

    Vec3 o(0, 0, 0.4);
    Vec3 x(0.8, 0, 0);
    Vec3 y(0, 0, 1);

    const int32 count = 5;

    const Material* outers[count];

    Spectrum color = { 1.0f, 1.0f, 1.0f };
    Float metallic = 1.0f;
    Float transmission = 1.0f;
    Float anisotrophy = 0.0f;
    Float ior = 1.5f;

    outers[3] = CreatePrincipledMaterial(scene, color, metallic, 0.0f, anisotrophy, ior, transmission);
    outers[1] = CreatePrincipledMaterial(scene, color, metallic, 0.25f, anisotrophy, ior, transmission);
    outers[0] = CreatePrincipledMaterial(scene, color, metallic, 0.5f, anisotrophy, ior, transmission);
    outers[2] = CreatePrincipledMaterial(scene, color, metallic, 0.75f, anisotrophy, ior, transmission);
    outers[4] = CreatePrincipledMaterial(scene, color, metallic, 1.0f, anisotrophy, ior, transmission);

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
            LoadModel(scene, "res/mori_knob/inner.obj", tf);
            // LoadModel(scene, "res/mori_knob/equation.obj", tf);
        }
    }

    CreateUniformInfiniteLight(scene, Spectrum(1));
    // CreateImageInfiniteLight(scene, "res/HDR/aerodynamics_workshop_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/peppermint_powerplant_4k.hdr", Transform(Quat(pi / 2, y_axis)));

    Float aspect_ratio = 4.f / 1.f;
    // Float aspect_ratio = 9.f / 16.f;
    // Float aspect_ratio = 3.f / 2.f;
    // Float aspect_ratio = 4.f / 3.f;
    // Float aspect_ratio = 1.f;
    int32 width = 1000;
    int32 height = int32(width / aspect_ratio);

    Point3 lookfrom = Point3{ 0, 1.0, 2.28 };
    Point3 lookat = Point3{ 0.0, 0.1, 0.0 };

    Float dist_to_focus = Dist(lookfrom, lookat);
    Float aperture = 0.01f;
    Float vFov = 30.0;

    return std::make_unique<PerspectiveCamera>(lookfrom, lookat, y_axis, vFov, aperture, dist_to_focus, Point2i(width, height));
}

std::unique_ptr<Camera> FurnaceDielectric(Scene& scene)
{
    HomogeneousMedium* hm = scene.CreateMedium<HomogeneousMedium>(Spectrum(0, 0, 0), Spectrum(10), Spectrum(0.0), -0.9f);
    MediumInterface mi(hm, nullptr);

    SetLoaderUseForceFallbackMaterial(true);

    // Floor
    {
        // auto checker = CreateSpectrumCheckerTexture(scene, 0.75, 0.3, Point2(60));
        // auto tf = Transform{ Vec3(0, 0, 0), Quat::FromEuler({ 0, 0, 0 }), Vec3(10) };
        // auto floor = scene.CreateMaterial<DiffuseMaterial>(checker);
        // SetLoaderFallbackMaterial(floor);
        // LoadModel(scene, "res/background.obj", tf);
    }

    int32 w = 5;
    int32 h = 1;

    Vec3 o(0, 0, 0.4);
    Vec3 x(0.8, 0, 0);
    Vec3 y(0, 0, 1);

    const int32 count = 5;

    const Material* outers[count];

    Spectrum color = { 1.0f, 1.0f, 1.0f };
    Float metallic = 0.0f;
    Float transmission = 1.0f;
    Float anisotrophy = 0.0f;
    Float ior = 1.5f;
    bool energy_compensation = true;

    outers[3] = CreateDielectricMaterial(scene, ior, color, 0.0f, energy_compensation);
    outers[1] = CreateDielectricMaterial(scene, ior, color, 0.25f, energy_compensation);
    outers[0] = CreateDielectricMaterial(scene, ior, color, 0.5f, energy_compensation);
    outers[2] = CreateDielectricMaterial(scene, ior, color, 0.75f, energy_compensation);
    outers[4] = CreateDielectricMaterial(scene, ior, color, 1.0f, energy_compensation);

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

            tf = Transform{ p, Quat::FromEuler({ 0, 0, 0 }), Vec3(2.2f) };
            // LoadModel(scene, "res/mori_knob/outer.obj", tf);
            LoadModel(scene, "res/mori_knob/inner.obj", tf);
            // LoadModel(scene, "res/mori_knob/equation.obj", tf);
        }
    }

    CreateUniformInfiniteLight(scene, Spectrum(1));
    // CreateImageInfiniteLight(scene, "res/HDR/aerodynamics_workshop_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/peppermint_powerplant_4k.hdr", Transform(Quat(pi / 2, y_axis)));

    Float aspect_ratio = 4.f / 1.f;
    // Float aspect_ratio = 9.f / 16.f;
    // Float aspect_ratio = 3.f / 2.f;
    // Float aspect_ratio = 4.f / 3.f;
    // Float aspect_ratio = 1.f;
    int32 width = 1000;
    int32 height = int32(width / aspect_ratio);

    Point3 lookfrom = Point3{ 0, 1.0, 2.28 };
    Point3 lookat = Point3{ 0.0, 0.1, 0.0 };

    Float dist_to_focus = Dist(lookfrom, lookat);
    Float aperture = 0.01f;
    Float vFov = 30.0;

    return std::make_unique<PerspectiveCamera>(lookfrom, lookat, y_axis, vFov, aperture, dist_to_focus, Point2i(width, height));
}

std::unique_ptr<Camera> FurnaceConductor(Scene& scene)
{
    HomogeneousMedium* hm = scene.CreateMedium<HomogeneousMedium>(Spectrum(0, 0, 0), Spectrum(10), Spectrum(0.0), -0.9f);
    MediumInterface mi(hm, nullptr);

    SetLoaderUseForceFallbackMaterial(true);

    // Floor
    {
        // auto checker = CreateSpectrumCheckerTexture(scene, 0.75, 0.3, Point2(60));
        // auto tf = Transform{ Vec3(0, 0, 0), Quat::FromEuler({ 0, 0, 0 }), Vec3(10) };
        // auto floor = scene.CreateMaterial<DiffuseMaterial>(checker);
        // SetLoaderFallbackMaterial(floor);
        // LoadModel(scene, "res/background.obj", tf);
    }

    int32 w = 5;
    int32 h = 1;

    Vec3 o(0, 0, 0.4);
    Vec3 x(0.8, 0, 0);
    Vec3 y(0, 0, 1);

    const int32 count = 5;

    const Material* outers[count];

    Spectrum color = { 1.0f, 1.0f, 1.0f };
    // Spectrum color = { 212 / 255.f, 175 / 255.f, 55 / 255.f };
    Float metallic = 0.0f;
    Float transmission = 1.0f;
    Float anisotrophy = 0.0f;
    Float ior = 1.5f;
    bool energy_compensation = true;

    outers[3] = CreateConductorMaterial(scene, color, 0.0f, energy_compensation);
    outers[1] = CreateConductorMaterial(scene, color, 0.25f, energy_compensation);
    outers[0] = CreateConductorMaterial(scene, color, 0.5f, energy_compensation);
    outers[2] = CreateConductorMaterial(scene, color, 0.75f, energy_compensation);
    outers[4] = CreateConductorMaterial(scene, color, 1.0f, energy_compensation);

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
            // LoadModel(scene, "res/mori_knob/equation.obj", tf);
        }
    }

    CreateUniformInfiniteLight(scene, Spectrum(1));
    // CreateImageInfiniteLight(scene, "res/HDR/aerodynamics_workshop_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/peppermint_powerplant_4k.hdr", Transform(Quat(pi / 2, y_axis)));

    Float aspect_ratio = 4.f / 1.f;
    // Float aspect_ratio = 9.f / 16.f;
    // Float aspect_ratio = 3.f / 2.f;
    // Float aspect_ratio = 4.f / 3.f;
    // Float aspect_ratio = 1.f;
    int32 width = 1000;
    int32 height = int32(width / aspect_ratio);

    Point3 lookfrom = Point3{ 0, 1.0, 2.28 };
    Point3 lookat = Point3{ 0.0, 0.1, 0.0 };

    Float dist_to_focus = Dist(lookfrom, lookat);
    Float aperture = 0.01f;
    Float vFov = 30.0;

    return std::make_unique<PerspectiveCamera>(lookfrom, lookat, y_axis, vFov, aperture, dist_to_focus, Point2i(width, height));
}

static int32 index0 = Sample::Register("furnace", FurnacePrincipled);
static int32 index1 = Sample::Register("furnace2", FurnaceDielectric);
static int32 index2 = Sample::Register("furnace3", FurnaceConductor);