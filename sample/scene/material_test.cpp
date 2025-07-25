#include "../samples.h"

std::unique_ptr<Camera> MaterialTest(Scene& scene)
{
    HomogeneousMedium* hm = scene.CreateMedium<HomogeneousMedium>(Spectrum(0, 0, 0), Spectrum(10), Spectrum(0.0), -0.9f);
    MediumInterface mi(hm, nullptr);
    // SetLoaderFallbackMediumInterface(mi);

    SetLoaderUseForceFallbackMaterial(true);

    // Floor
    {
        auto checker = CreateSpectrumCheckerTexture(scene, 0.75, 0.3, Point2(20));
        auto tf = Transform{ Vec3(0, 0, 0), Quat::FromEuler({ 0, 0, 0 }), Vec3(3) };
        auto floor = scene.CreateMaterial<DiffuseMaterial>(checker);
        SetLoaderFallbackMaterial(floor);
        LoadModel(scene, "res/background.obj", tf);
    }

    auto normalmap = CreateSpectrumImageTexture(scene, "res/bistro/Concrete_Normal.png", true);

    const Material* white = CreateDiffuseMaterial(scene, 1.0f);
    const Material* air = CreateDielectricMaterial(scene, 1.0f);
    const Material* glass = CreateDielectricMaterial(scene, 1.5f);
    const Material* rough_glass = CreateDielectricMaterial(scene, 1.5f, 0.01f);
    const Material* copper = CreateConductorMaterial(scene, { 0.21100, 1.1274, 1.2444 }, { 4.1592, 2.5978, 2.433 }, 0.2f);
    const Material* patina = CreateLayeredMaterial(scene, air, copper, true, { 0.25f, 0.95f, 0.45f }, 0.2, 0.1f);
    const Material* gold = CreateConductorMaterial(scene, { 0.161, 0.492, 1.426 }, { 4.08769, 2.32625, 1.846 }, 0.1f);
    const Material* platinum = CreateConductorMaterial(scene, { 0.49745, 0.48267, 0.60399 }, { 6.9266, 4.8444, 3.8895 }, 0.1f);

    const Material* outer = CreateLayeredMaterial(scene, rough_glass, gold, true, Spectrum(0.7, 0, 0), 0.2f, -0.3f);
    const Material* inner = CreateDiffuseMaterial(scene, 0.8f, 0.0f);

    // auto tf = Transform{ o, Quat::FromEuler({ 0, pi / 6, 0 }), Vec3(2) };
    auto tf = Transform{ { 0, 0, 0.4 }, Quat::FromEuler({ 0, 0, 0 }), Vec3(2) };

    // https://github.com/lighttransport/lighttransportequation-orb
    SetLoaderFallbackMaterial(outer);
    LoadModel(scene, "res/mori_knob/base.obj", tf);
    LoadModel(scene, "res/mori_knob/outer.obj", tf);

    SetLoaderFallbackMaterial(inner);
    LoadModel(scene, "res/mori_knob/inner.obj", tf);
    // LoadModel(scene, "res/mori_knob/equation.obj", tf);

    // CreateImageInfiniteLight(scene, "res/HDR/photo_studio_loft_hall_1k.hdr", Transform(Quat(pi, y_axis)));
    CreateImageInfiniteLight(scene, "res/HDR/aerodynamics_workshop_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/photo_studio_01_1k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/scythian_tombs_2_4k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/material-test.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/peppermint_powerplant_4k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/quarry_04_puresky_1k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/sunflowers_puresky_1k.hdr");
    // CreateImageInfiniteLight(scene, "res/HDR/san_giuseppe_bridge_4k.hdr", Transform(Quat(-pi, y_axis)));
    // CreateUniformInfiniteLight(scene, Spectrum(1));

    Float aspect_ratio = 1.0f;
    int32 width = 1000;
    int32 height = int32(width / aspect_ratio);

    Point3 lookfrom = Point3{ 0, 1.0, 2.0 };
    Point3 lookat = Point3{ 0.0, 0.1, 0.0 };

    Float dist_to_focus = Dist(lookfrom, lookat);
    Float aperture = 0.01f;
    Float vFov = 30.0;

    return std::make_unique<PerspectiveCamera>(lookfrom, lookat, y_axis, vFov, aperture, dist_to_focus, Point2i(width, height));
}

std::unique_ptr<Camera> MetallicRoughness(Scene& scene)
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

    Float scale = 2.0f;

    int32 w = 3;
    int32 h = 1;

    Vec3 o(0, 0, 0.4);
    Vec3 x(0.8, 0, 0);
    Vec3 y(0, 0, 1);

    const int32 count = 3;

    auto normalmap = CreateSpectrumImageTexture(scene, "res/bistro/Concrete_Normal.png", true);

    const Material* outers[count];
    outers[0] = CreateDielectricMaterial(scene, 1.5f, 0.08f);
    outers[1] = CreateConductorMaterial(scene, { 0.1, 0.2, 1.9 }, { 3, 2.5, 2 }, 0.05f, 0.4f, true, normalmap);
    outers[2] = CreateMetallicRoughnessMaterial(scene, { 80 / 255.0, 1.0, 175 / 255.0 }, 0, 0);

    const Material* inners[count];
    inners[0] = CreateConductorMaterial(scene, Spectrum{ 0.7f }, (0.05f), (0.4f));
    inners[1] = CreateMirrorMaterial(scene, Spectrum(0.7f));
    inners[2] = CreateDiffuseMaterial(scene, Spectrum(0.7));

    for (int32 j = 0; j < h; ++j)
    {
        for (int32 i = 0; i < w; ++i)
        {
            int32 sign_i = std::pow<int32>(-1, i);
            int32 sign_j = std::pow<int32>(-1, j);
            Vec3 p = o + (sign_j * y * ((j + 1) / 2)) + (sign_i * x * ((i + 1) / 2));

            // SetLoaderFallbackMediumInterface(mi);

            auto tf = Transform{ p, Quat::FromEuler({ 0, pi / 6, 0 }), Vec3(scale) };

            // https://github.com/lighttransport/lighttransportequation-orb
            SetLoaderFallbackMaterial(outers[std::min(i + j * w, count)]);
            LoadModel(scene, "res/mori_knob/base.obj", tf);
            LoadModel(scene, "res/mori_knob/outer.obj", tf);

            SetLoaderFallbackMaterial(inners[std::min(i + j * w, count)]);
            LoadModel(scene, "res/mori_knob/inner.obj", tf);
            LoadModel(scene, "res/mori_knob/equation.obj", tf);
        }
    }

    // CreateImageInfiniteLight(scene,
    //     "res/material_test_ball/envmap.hdr", Transform(Quat::FromEuler(0, DegToRad(-67.26139831542969), 0))
    // );
    // CreateImageInfiniteLight(scene, "res/HDR/photo_studio_loft_hall_1k.hdr", Transform(Quat(pi, y_axis)));
    CreateImageInfiniteLight(scene, "res/HDR/aerodynamics_workshop_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/photo_studio_01_1k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/scythian_tombs_2_4k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/material-test.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/peppermint_powerplant_4k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/quarry_04_puresky_1k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/solitude_night_1k.hdr");
    // CreateImageInfiniteLight(scene, "res/HDR/sunflowers_puresky_1k.hdr");
    // CreateImageInfiniteLight(scene, "res/HDR/san_giuseppe_bridge_4k.hdr", Transform(Quat(-pi, y_axis)));
    // CreateUniformInfiniteLight(scene, Spectrum(1));

    Float aspect_ratio = 21.f / 9.f;
    // Float aspect_ratio = 9.f / 16.f;
    // Float aspect_ratio = 3.f / 2.f;
    // Float aspect_ratio = 4.f / 3.f;
    // Float aspect_ratio = 1.f;
    int32 width = 1600;
    int32 height = int32(width / aspect_ratio);

    Point3 lookfrom = Point3{ 0, 1.0, 2.28 };
    Point3 lookat = Point3{ 0.0, 0.1, 0.0 };

    Float dist_to_focus = Dist(lookfrom, lookat);
    Float aperture = 0.01f;
    Float vFov = 30.0;

    return std::make_unique<PerspectiveCamera>(lookfrom, lookat, y_axis, vFov, aperture, dist_to_focus, Point2i(width, height));
}

std::unique_ptr<Camera> Dielectrics(Scene& scene)
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

    Float scale = 2.0f;

    int32 w = 3;
    int32 h = 1;

    Vec3 o(0, 0, 0.4);
    Vec3 x(0.8, 0, 0);
    Vec3 y(0, 0, 1);

    const int32 count = 3;

    auto normalmap = CreateSpectrumImageTexture(scene, "res/bistro/Concrete_Normal.png", true);

    const Material* outers[count];
    outers[0] = CreateDielectricMaterial(scene, 1.5f, 0.02f);
    outers[1] = CreateDielectricMaterial(scene, 1.5f, 0.0f);
    outers[2] = CreateDielectricMaterial(scene, 1.5f, 0.05f);

    const Material* inners[count];
    inners[0] = CreateMetallicRoughnessMaterial(scene, Spectrum{ 0.66 }, (0), (0));
    inners[1] = inners[0];
    inners[2] = inners[0];

    for (int32 j = 0; j < h; ++j)
    {
        for (int32 i = 0; i < w; ++i)
        {
            int32 sign_i = std::pow<int32>(-1, i);
            int32 sign_j = std::pow<int32>(-1, j);
            Vec3 p = o + (sign_j * y * ((j + 1) / 2)) + (sign_i * x * ((i + 1) / 2));

            // SetLoaderFallbackMediumInterface(mi);

            auto tf = Transform{ p, Quat::FromEuler({ 0, pi / 6, 0 }), Vec3(scale) };

            // https://github.com/lighttransport/lighttransportequation-orb
            SetLoaderFallbackMaterial(outers[std::min(i + j * w, count)]);
            LoadModel(scene, "res/mori_knob/base.obj", tf);
            LoadModel(scene, "res/mori_knob/outer.obj", tf);

            SetLoaderFallbackMaterial(inners[std::min(i + j * w, count)]);
            LoadModel(scene, "res/mori_knob/inner.obj", tf);
            LoadModel(scene, "res/mori_knob/equation.obj", tf);
        }
    }

    // CreateImageInfiniteLight(scene,
    //     "res/material_test_ball/envmap.hdr", Transform(Quat::FromEuler(0, DegToRad(-67.26139831542969), 0))
    // );
    // CreateImageInfiniteLight(scene, "res/HDR/photo_studio_loft_hall_1k.hdr", Transform(Quat(pi, y_axis)));
    CreateImageInfiniteLight(scene, "res/HDR/aerodynamics_workshop_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/photo_studio_01_1k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/scythian_tombs_2_4k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/material-test.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/peppermint_powerplant_4k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/quarry_04_puresky_1k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/solitude_night_1k.hdr");
    // CreateImageInfiniteLight(scene, "res/HDR/sunflowers_puresky_1k.hdr");
    // CreateImageInfiniteLight(scene, "res/HDR/san_giuseppe_bridge_4k.hdr", Transform(Quat(-pi, y_axis)));
    // CreateUniformInfiniteLight(scene, Spectrum(1));

    Float aspect_ratio = 21.f / 9.f;
    // Float aspect_ratio = 9.f / 16.f;
    // Float aspect_ratio = 3.f / 2.f;
    // Float aspect_ratio = 4.f / 3.f;
    // Float aspect_ratio = 1.f;
    int32 width = 1600;
    int32 height = int32(width / aspect_ratio);

    Point3 lookfrom = Point3{ 0, 1.0, 2.28 };
    Point3 lookat = Point3{ 0.0, 0.1, 0.0 };

    Float dist_to_focus = Dist(lookfrom, lookat);
    Float aperture = 0.01f;
    Float vFov = 30.0;

    return std::make_unique<PerspectiveCamera>(lookfrom, lookat, y_axis, vFov, aperture, dist_to_focus, Point2i(width, height));
}

std::unique_ptr<Camera> Skins(Scene& scene)
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

    Float scale = 2.0f;

    int32 w = 3;
    int32 h = 1;

    Vec3 o(0, 0, 0.4);
    Vec3 x(0.8, 0, 0);
    Vec3 y(0, 0, 1);

    const int32 count = 3;

    auto normalmap = CreateSpectrumImageTexture(scene, "res/bistro/Concrete_Normal.png", true);

    const Material* skins[count];
    skins[1] = CreateSubsurfaceRandomWalkMaterial(
        scene, Spectrum(255 / 255.0, 195 / 255.0, 150 / 255.0) * 1.0, Spectrum(0.5, 0.25, 0.125) * 0.07, 1.38f, 0.1f
    );
    skins[0] = CreateSubsurfaceRandomWalkMaterial(
        scene, Spectrum(255 / 255.0, 195 / 255.0, 150 / 255.0) * 0.8, Spectrum(0.5, 0.25, 0.125) * 0.03, 1.38f, 0.1f
    );
    skins[2] = CreateSubsurfaceRandomWalkMaterial(
        scene, Spectrum(255 / 255.0, 195 / 255.0, 150 / 255.0) * 0.3, Spectrum(0.5, 0.25, 0.125) * 0.01, 1.38f, 0.1f
    );

    for (int32 j = 0; j < h; ++j)
    {
        for (int32 i = 0; i < w; ++i)
        {
            int32 sign_i = std::pow<int32>(-1, i);
            int32 sign_j = std::pow<int32>(-1, j);
            Vec3 p = o + (sign_j * y * ((j + 1) / 2)) + (sign_i * x * ((i + 1) / 2));

            // SetLoaderFallbackMediumInterface(mi);

            auto tf = Transform{ p, Quat::FromEuler({ 0, pi / 6, 0 }), Vec3(scale) };

            // https://github.com/lighttransport/lighttransportequation-orb
            SetLoaderFallbackMaterial(skins[std::min(i + j * w, count)]);
            LoadModel(scene, "res/mori_knob/base.obj", tf);
            LoadModel(scene, "res/mori_knob/outer.obj", tf);

            // SetLoaderFallbackMaterial(skins[std::min(i + j * w, count)]);
            // LoadModel(scene, "res/mori_knob/inner.obj", tf);
            // LoadModel(scene, "res/mori_knob/equation.obj", tf);
        }
    }

    // CreateImageInfiniteLight(scene,
    //     "res/material_test_ball/envmap.hdr", Transform(Quat::FromEuler(0, DegToRad(-67.26139831542969), 0))
    // );
    // CreateImageInfiniteLight(scene, "res/HDR/photo_studio_loft_hall_1k.hdr", Transform(Quat(pi, y_axis)));
    CreateImageInfiniteLight(scene, "res/HDR/aerodynamics_workshop_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/photo_studio_01_1k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/scythian_tombs_2_4k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/material-test.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/peppermint_powerplant_4k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/quarry_04_puresky_1k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/solitude_night_1k.hdr");
    // CreateImageInfiniteLight(scene, "res/HDR/sunflowers_puresky_1k.hdr");
    // CreateImageInfiniteLight(scene, "res/HDR/san_giuseppe_bridge_4k.hdr", Transform(Quat(-pi, y_axis)));
    // CreateUniformInfiniteLight(scene, Spectrum(1));

    auto light = CreateDiffuseLightMaterial(scene, Spectrum(5.0f));
    auto tf = Transform{ 0.0f, 0.8f, -0.5f, Quat::FromEuler({ 0, 0, 0 }), Vec3(3, 0.5, 1) };
    CreateRectXY(scene, tf, light);

    Float aspect_ratio = 21.f / 9.f;
    // Float aspect_ratio = 9.f / 16.f;
    // Float aspect_ratio = 3.f / 2.f;
    // Float aspect_ratio = 4.f / 3.f;
    // Float aspect_ratio = 1.f;
    int32 width = 1600;
    int32 height = int32(width / aspect_ratio);

    Point3 lookfrom = Point3{ 0, 1.0, 2.28 };
    Point3 lookat = Point3{ 0.0, 0.1, 0.0 };

    Float dist_to_focus = Dist(lookfrom, lookat);
    Float aperture = 0.01f;
    Float vFov = 30.0;

    return std::make_unique<PerspectiveCamera>(lookfrom, lookat, y_axis, vFov, aperture, dist_to_focus, Point2i(width, height));
}

std::unique_ptr<Camera> Mixtures(Scene& scene)
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

    Float scale = 2.0f;

    int32 w = 3;
    int32 h = 1;

    Vec3 o(0, 0, 0.4);
    Vec3 x(0.8, 0, 0);
    Vec3 y(0, 0, 1);

    const int32 count = 3;

    auto normalmap = CreateSpectrumImageTexture(scene, "res/bistro/Concrete_Normal.png", true);

    const Material* outers[count];
    auto a = CreateDielectricMaterial(scene, 1.5f, 0.0f);
    auto b = CreateConductorMaterial(scene, { 0.7, 0.3, 0.2 }, (0.1f));
    auto checker =
        CreateFloatCheckerTexture(scene, CreateFloatConstantTexture(scene, 0), CreateFloatConstantTexture(scene, 1), Point2(20));
    outers[1] = scene.CreateMaterial<MixtureMaterial>(a, b, checker);

    auto c = CreateConductorMaterial(scene, Spectrum{ 0.7f }, (0.05f), (0.4f));
    auto d = CreateConductorMaterial(scene, Spectrum{ 0.7f }, (0.4f), (0.05f));
    outers[0] = scene.CreateMaterial<MixtureMaterial>(c, d, checker);

    auto e = CreateDiffuseMaterial(scene, Spectrum(0.7, 0.9, 0.5));
    auto f = CreateMirrorMaterial(scene, Spectrum(0.6, 0.5, 0.4));
    outers[2] = scene.CreateMaterial<MixtureMaterial>(e, f, checker);

    const Material* inners[count];
    inners[0] = CreateMetallicRoughnessMaterial(scene, Spectrum{ 0.66 }, 0, 0);
    inners[1] = inners[0];
    inners[2] = inners[0];

    for (int32 j = 0; j < h; ++j)
    {
        for (int32 i = 0; i < w; ++i)
        {
            int32 sign_i = std::pow<int32>(-1, i);
            int32 sign_j = std::pow<int32>(-1, j);
            Vec3 p = o + (sign_j * y * ((j + 1) / 2)) + (sign_i * x * ((i + 1) / 2));

            // SetLoaderFallbackMediumInterface(mi);

            auto tf = Transform{ p, Quat::FromEuler({ 0, pi / 6, 0 }), Vec3(scale) };

            // https://github.com/lighttransport/lighttransportequation-orb
            SetLoaderFallbackMaterial(outers[std::min(i + j * w, count)]);
            LoadModel(scene, "res/mori_knob/base.obj", tf);
            LoadModel(scene, "res/mori_knob/outer.obj", tf);

            SetLoaderFallbackMaterial(inners[std::min(i + j * w, count)]);
            LoadModel(scene, "res/mori_knob/inner.obj", tf);
            LoadModel(scene, "res/mori_knob/equation.obj", tf);
        }
    }

    // CreateImageInfiniteLight(scene,
    //     "res/material_test_ball/envmap.hdr", Transform(Quat::FromEuler(0, DegToRad(-67.26139831542969), 0))
    // );
    // CreateImageInfiniteLight(scene, "res/HDR/photo_studio_loft_hall_1k.hdr", Transform(Quat(pi, y_axis)));
    CreateImageInfiniteLight(scene, "res/HDR/aerodynamics_workshop_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/photo_studio_01_1k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/scythian_tombs_2_4k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/material-test.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/peppermint_powerplant_4k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/quarry_04_puresky_1k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/solitude_night_1k.hdr");
    // CreateImageInfiniteLight(scene, "res/HDR/sunflowers_puresky_1k.hdr");
    // CreateImageInfiniteLight(scene, "res/HDR/san_giuseppe_bridge_4k.hdr", Transform(Quat(-pi, y_axis)));
    // CreateUniformInfiniteLight(scene, Spectrum(1));

    Float aspect_ratio = 21.f / 9.f;
    // Float aspect_ratio = 9.f / 16.f;
    // Float aspect_ratio = 3.f / 2.f;
    // Float aspect_ratio = 4.f / 3.f;
    // Float aspect_ratio = 1.f;
    int32 width = 1600;
    int32 height = int32(width / aspect_ratio);

    Point3 lookfrom = Point3{ 0, 1.0, 2.28 };
    Point3 lookat = Point3{ 0.0, 0.1, 0.0 };

    Float dist_to_focus = Dist(lookfrom, lookat);
    Float aperture = 0.01f;
    Float vFov = 30.0;

    return std::make_unique<PerspectiveCamera>(lookfrom, lookat, y_axis, vFov, aperture, dist_to_focus, Point2i(width, height));
}

std::unique_ptr<Camera> Alphas(Scene& scene)
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

    Float scale = 2.0f;

    int32 w = 3;
    int32 h = 1;

    Vec3 o(0, 0, 0.4);
    Vec3 x(0.8, 0, 0);
    Vec3 y(0, 0, 1);

    const int32 count = 3;

    auto normalmap = CreateSpectrumImageTexture(scene, "res/bistro/Concrete_Normal.png", true);

    const Material* outers[count];
    outers[0] = CreateDiffuseMaterial(scene, Spectrum(.65f, .05f, .05f), 0, nullptr, 0.4f);
    outers[1] = CreateDiffuseMaterial(scene, Spectrum(.12f, .45f, .15f), 0, nullptr, 0.2f);
    outers[2] = CreateDiffuseMaterial(scene, Spectrum(.22f, .23f, .75f), 0, nullptr, 0.6f);

    const Material* inners[count];
    inners[0] = CreateMetallicRoughnessMaterial(scene, Spectrum{ 0.66 }, (0), (0));
    inners[1] = inners[0];
    inners[2] = inners[0];

    for (int32 j = 0; j < h; ++j)
    {
        for (int32 i = 0; i < w; ++i)
        {
            int32 sign_i = std::pow<int32>(-1, i);
            int32 sign_j = std::pow<int32>(-1, j);
            Vec3 p = o + (sign_j * y * ((j + 1) / 2)) + (sign_i * x * ((i + 1) / 2));

            // SetLoaderFallbackMediumInterface(mi);

            auto tf = Transform{ p, Quat::FromEuler({ 0, pi / 6, 0 }), Vec3(scale) };

            // https://github.com/lighttransport/lighttransportequation-orb
            SetLoaderFallbackMaterial(outers[std::min(i + j * w, count)]);
            LoadModel(scene, "res/mori_knob/base.obj", tf);
            LoadModel(scene, "res/mori_knob/outer.obj", tf);

            SetLoaderFallbackMaterial(inners[std::min(i + j * w, count)]);
            LoadModel(scene, "res/mori_knob/inner.obj", tf);
            LoadModel(scene, "res/mori_knob/equation.obj", tf);
        }
    }

    // CreateImageInfiniteLight(scene,
    //     "res/material_test_ball/envmap.hdr", Transform(Quat::FromEuler(0, DegToRad(-67.26139831542969), 0))
    // );
    // CreateImageInfiniteLight(scene, "res/HDR/photo_studio_loft_hall_1k.hdr", Transform(Quat(pi, y_axis)));
    CreateImageInfiniteLight(scene, "res/HDR/aerodynamics_workshop_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/photo_studio_01_1k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/scythian_tombs_2_4k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/material-test.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/peppermint_powerplant_4k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/quarry_04_puresky_1k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/solitude_night_1k.hdr");
    // CreateImageInfiniteLight(scene, "res/HDR/sunflowers_puresky_1k.hdr");
    // CreateImageInfiniteLight(scene, "res/HDR/san_giuseppe_bridge_4k.hdr", Transform(Quat(-pi, y_axis)));
    // CreateUniformInfiniteLight(scene, Spectrum(1));

    Float aspect_ratio = 21.f / 9.f;
    // Float aspect_ratio = 9.f / 16.f;
    // Float aspect_ratio = 3.f / 2.f;
    // Float aspect_ratio = 4.f / 3.f;
    // Float aspect_ratio = 1.f;
    int32 width = 1600;
    int32 height = int32(width / aspect_ratio);

    Point3 lookfrom = Point3{ 0, 1.0, 2.28 };
    Point3 lookat = Point3{ 0.0, 0.1, 0.0 };

    Float dist_to_focus = Dist(lookfrom, lookat);
    Float aperture = 0.01f;
    Float vFov = 30.0;

    return std::make_unique<PerspectiveCamera>(lookfrom, lookat, y_axis, vFov, aperture, dist_to_focus, Point2i(width, height));
}

std::unique_ptr<Camera> ColoredDielectrics(Scene& scene)
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

    Float scale = 2.0f;

    int32 w = 3;
    int32 h = 1;

    Vec3 o(0, 0, 0.4);
    Vec3 x(0.8, 0, 0);
    Vec3 y(0, 0, 1);

    const int32 count = 3;

    auto normalmap = CreateSpectrumImageTexture(scene, "res/bistro/Concrete_Normal.png", true);

    const Material* outers[count];
    outers[0] = CreateDielectricMaterial(scene, 1.5f, 0.0f, Sqrt(Spectrum(1.0f, 0.5f, 0.8f)));
    outers[1] = CreateDielectricMaterial(scene, 1.5f, 0.0f, Spectrum(1.0f));
    outers[2] = CreateDielectricMaterial(scene, 1.5f, 0.0f, Spectrum(1.0f, 0.5f, 0.8f));

    const Material* inners[count];
    inners[0] = CreateMetallicRoughnessMaterial(scene, Spectrum{ 0.66 }, (0), (0));
    inners[1] = inners[0];
    inners[2] = inners[0];

    for (int32 j = 0; j < h; ++j)
    {
        for (int32 i = 0; i < w; ++i)
        {
            int32 sign_i = std::pow<int32>(-1, i);
            int32 sign_j = std::pow<int32>(-1, j);
            Vec3 p = o + (sign_j * y * ((j + 1) / 2)) + (sign_i * x * ((i + 1) / 2));

            // SetLoaderFallbackMediumInterface(mi);

            auto tf = Transform{ p, Quat::FromEuler({ 0, pi / 6, 0 }), Vec3(scale) };

            // https://github.com/lighttransport/lighttransportequation-orb
            SetLoaderFallbackMaterial(outers[std::min(i + j * w, count)]);
            LoadModel(scene, "res/mori_knob/base.obj", tf);
            LoadModel(scene, "res/mori_knob/outer.obj", tf);

            SetLoaderFallbackMaterial(inners[std::min(i + j * w, count)]);
            LoadModel(scene, "res/mori_knob/inner.obj", tf);
            LoadModel(scene, "res/mori_knob/equation.obj", tf);
        }
    }

    // CreateImageInfiniteLight(scene,
    //     "res/material_test_ball/envmap.hdr", Transform(Quat::FromEuler(0, DegToRad(-67.26139831542969), 0))
    // );
    // CreateImageInfiniteLight(scene, "res/HDR/photo_studio_loft_hall_1k.hdr", Transform(Quat(pi, y_axis)));
    CreateImageInfiniteLight(scene, "res/HDR/aerodynamics_workshop_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/photo_studio_01_1k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/scythian_tombs_2_4k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/material-test.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/peppermint_powerplant_4k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/quarry_04_puresky_1k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/solitude_night_1k.hdr");
    // CreateImageInfiniteLight(scene, "res/HDR/sunflowers_puresky_1k.hdr");
    // CreateImageInfiniteLight(scene, "res/HDR/san_giuseppe_bridge_4k.hdr", Transform(Quat(-pi, y_axis)));
    // CreateUniformInfiniteLight(scene, Spectrum(1));

    Float aspect_ratio = 21.f / 9.f;
    // Float aspect_ratio = 9.f / 16.f;
    // Float aspect_ratio = 3.f / 2.f;
    // Float aspect_ratio = 4.f / 3.f;
    // Float aspect_ratio = 1.f;
    int32 width = 1600;
    int32 height = int32(width / aspect_ratio);

    Point3 lookfrom = Point3{ 0, 1.0, 2.28 };
    Point3 lookat = Point3{ 0.0, 0.1, 0.0 };

    Float dist_to_focus = Dist(lookfrom, lookat);
    Float aperture = 0.01f;
    Float vFov = 30.0;

    return std::make_unique<PerspectiveCamera>(lookfrom, lookat, y_axis, vFov, aperture, dist_to_focus, Point2i(width, height));
}

static int32 index0 = Sample::Register("material", MaterialTest);
static int32 index1 = Sample::Register("material1", MetallicRoughness);
static int32 index2 = Sample::Register("material2", Dielectrics);
static int32 index3 = Sample::Register("material3", Skins);
static int32 index4 = Sample::Register("material4", Mixtures);
static int32 index5 = Sample::Register("material5", Alphas);
static int32 index8 = Sample::Register("material6", ColoredDielectrics);
