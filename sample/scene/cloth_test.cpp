#include "../samples.h"

std::unique_ptr<Camera> ClothTest(Scene& scene)
{
    HomogeneousMedium* hm = scene.CreateMedium<HomogeneousMedium>(Spectrum(0, 0, 0), Spectrum(10), Spectrum(0.0), -0.9f);
    MediumInterface mi(hm, nullptr);

    ModelLoaderOptions options;
    options.use_fallback_material = true;

    // Floor
    {
        auto checker = CreateSpectrumCheckerTexture(scene, 0.75, 0.3, Point2(60));
        auto tf = Transform{ Vec3(0, 0, 0), Quat::FromEuler({ 0, 0, 0 }), Vec3(10) };
        auto floor = scene.CreateMaterial<DiffuseMaterial>(checker);

        options.fallback_material = floor;
        LoadModel(scene, "res/background.obj", tf, options);
    }

    int32 w = 5;
    int32 h = 1;

    Vec3 o(0, 0, 0.4);
    Vec3 x(0.8, 0, 0);
    Vec3 y(0, 0, 1);

    const int32 count = 5;
    const Material* outers[count];

    Spectrum base = { 0.3f, 0.0f, 0.01f };
    Spectrum sheen = { 1, 1, 1 };
    Float metallic = 0.0f;
    Float transmission = 1.0f;
    Float anisotrophy = 0.0f;
    Float ior = 1.5f;
    bool energy_compensation = true;

    outers[3] = CreateClothMaterial(scene, base, sheen, 0.0f);
    outers[1] = CreateClothMaterial(scene, base, sheen, 0.25f);
    outers[0] = CreateClothMaterial(scene, base, sheen, 0.5f);
    outers[2] = CreateClothMaterial(scene, base, sheen, 0.75f);
    outers[4] = CreateClothMaterial(scene, base, sheen, 1.0f);

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

            options.fallback_material = outers[std::min(i + j * w, count)];

            // https://github.com/lighttransport/lighttransportequation-orb
            auto tf = Transform{ p, Quat::FromEuler({ 0, 0, 0 }), Vec3(0.25f) };
            // LoadModel(scene, "res/mori_knob/base.obj", tf, options);

            // tf = Transform{ p, Quat::FromEuler({ 0, 0, 0 }), Vec3(2.0f) };
            // LoadModel(scene, "res/mori_knob/outer.obj", tf, options);
            // LoadModel(scene, "res/mori_knob/inner.obj", tf, options);
            // LoadModel(scene, "res/mori_knob/equation.obj", tf, options);
            LoadModel(scene, "res/cloth.glb", tf, options);
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
    int32 width = 1000;
    int32 height = int32(width / aspect_ratio);

    Point3 lookfrom = Point3{ 0, 1.0, 2.28 };
    Point3 lookat = Point3{ 0.0, 0.1, 0.0 };

    Float dist_to_focus = Dist(lookfrom, lookat);
    Float aperture = 0.01f;
    Float vFov = 30.0;

    return std::make_unique<PerspectiveCamera>(lookfrom, lookat, y_axis, vFov, aperture, dist_to_focus, Point2i(width, height));
}

std::unique_ptr<Camera> ClothTest2(Scene& scene)
{
    HomogeneousMedium* hm = scene.CreateMedium<HomogeneousMedium>(Spectrum(0, 0, 0), Spectrum(10), Spectrum(0.0), -0.9f);
    MediumInterface mi(hm, nullptr);

    ModelLoaderOptions options;
    options.use_fallback_material = true;

    // Floor
    {
        // auto checker = CreateSpectrumCheckerTexture(scene, 0.75, 0.3, Point2(60));
        // auto tf = Transform{ Vec3(0, 0, 0), Quat::FromEuler({ 0, 0, 0 }), Vec3(10) };
        // auto floor = scene.CreateMaterial<DiffuseMaterial>(checker);
        // options.fallback_material = floor;
        // LoadModel(scene, "res/background.obj", tf, options);
    }

    int32 w = 11;
    int32 h = 1;

    Vec3 o(0, 0, 0.4);
    Vec3 x(0.8, 0, 0);
    Vec3 y(0, 0, 1);

    const int32 count = 11;

    const Material* outers[count];

    Spectrum base = { 0.3f, 0.0f, 0.01f };
    Spectrum sheen = { 1, 1, 1 };
    Float metallic = 0.0f;
    Float transmission = 1.0f;
    Float anisotrophy = 0.0f;
    Float ior = 1.5f;
    bool energy_compensation = true;

    outers[9] = CreateClothMaterial(scene, base, sheen, 0.0f);
    outers[7] = CreateClothMaterial(scene, base, sheen, 0.1f);
    outers[5] = CreateClothMaterial(scene, base, sheen, 0.2f);
    outers[3] = CreateClothMaterial(scene, base, sheen, 0.3f);
    outers[1] = CreateClothMaterial(scene, base, sheen, 0.4f);
    outers[0] = CreateClothMaterial(scene, base, sheen, 0.5f);
    outers[2] = CreateClothMaterial(scene, base, sheen, 0.6f);
    outers[4] = CreateClothMaterial(scene, base, sheen, 0.7f);
    outers[6] = CreateClothMaterial(scene, base, sheen, 0.8f);
    outers[8] = CreateClothMaterial(scene, base, sheen, 0.9f);
    outers[10] = CreateClothMaterial(scene, base, sheen, 1.0f);

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

            // options.fallback_medium_interface = mi;

            options.fallback_material = outers[std::min(i + j * w, count)];

            // https://github.com/lighttransport/lighttransportequation-orb
            auto tf = Transform{ p, Quat::FromEuler({ 0, 0, 0 }), Vec3(0.25f) };
            // LoadModel(scene, "res/mori_knob/base.obj", tf, options);

            // tf = Transform{ p, Quat::FromEuler({ 0, 0, 0 }), Vec3(2.0f) };
            // LoadModel(scene, "res/mori_knob/outer.obj", tf, options);
            // LoadModel(scene, "res/mori_knob/inner.obj", tf, options);
            // LoadModel(scene, "res/mori_knob/equation.obj", tf, options);
            LoadModel(scene, "res/cloth.glb", tf, options);
        }
    }

    // CreateUniformInfiniteLight(scene, Spectrum(1));
    CreateImageInfiniteLight(scene, "res/HDR/aerodynamics_workshop_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/peppermint_powerplant_4k.hdr", Transform(Quat(pi / 2, y_axis)));

    Float aspect_ratio = 8.f / 1.f;
    int32 width = 1600;
    int32 height = int32(width / aspect_ratio);

    Point3 lookfrom = Point3{ 0, 0.25, 4.75 };
    Point3 lookat = Point3{ 0.0, 0.25, 0.0 };

    Float dist_to_focus = Dist(lookfrom, lookat);
    Float aperture = 0.0f;
    Float vFov = 15.0;

    return std::make_unique<PerspectiveCamera>(lookfrom, lookat, y_axis, vFov, aperture, dist_to_focus, Point2i(width, height));
}

static int32 index1 = Sample::Register("cloth", ClothTest);
static int32 index2 = Sample::Register("Cloth", ClothTest2);
