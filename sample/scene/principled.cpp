#include "../samples.h"

std::unique_ptr<Camera> Principled(Scene& scene, int32 lobe, int32 model)
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

    auto normalmap = CreateSpectrumImageTexture(scene, "res/bistro/Concrete_Normal.png", true);

    const Material* outers[count];
    switch (lobe)
    {
    case 0:
    {
        outers[3] = CreatePrincipledMaterial(scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 1, 0.1f);
        outers[1] = CreatePrincipledMaterial(scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 1, 0.3f);
        outers[0] = CreatePrincipledMaterial(scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 1, 0.5f);
        outers[2] = CreatePrincipledMaterial(scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 1, 0.7f);
        outers[4] = CreatePrincipledMaterial(scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 1, 0.9f);
    }
    break;

    case 1:
    {
        outers[3] = CreatePrincipledMaterial(scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 0, 0.1f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
        outers[1] = CreatePrincipledMaterial(scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 0, 0.1f, 0.0f, 1.25f, 0.0f, 0.0f, 0.0f);
        outers[0] = CreatePrincipledMaterial(scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 0, 0.1f, 0.0f, 1.5f, 0.0f, 0.0f, 0.0f);
        outers[2] = CreatePrincipledMaterial(scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 0, 0.1f, 0.0f, 1.75f, 0.0f, 0.0f, 0.0f);
        outers[4] = CreatePrincipledMaterial(scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 0, 0.1f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f);
    }
    break;

    case 2:
    {
        outers[3] = CreatePrincipledMaterial(scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 0, 0.1f, 0.0f, 1.5f, 0.0f, 0.0f, 0.0f);
        outers[1] = CreatePrincipledMaterial(scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 0, 0.1f, 0.0f, 1.5f, 0.25f, 0.0f, 0.0f);
        outers[0] = CreatePrincipledMaterial(scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 0, 0.1f, 0.0f, 1.5f, 0.5f, 0.0f, 0.0f);
        outers[2] = CreatePrincipledMaterial(scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 0, 0.1f, 0.0f, 1.5f, 0.75f, 0.0f, 0.0f);
        outers[4] = CreatePrincipledMaterial(scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 0, 0.1f, 0.0f, 1.5f, 1.0f, 0.0f, 0.0f);
    }
    break;

    case 3:
    {
        outers[3] = CreatePrincipledMaterial(scene, Spectrum{ 0.9f, 0.5f, 0.3f }, 1, 0.1f, 0.0f, 1.5f, 0.0f, 0.0f, 0.0f);
        outers[1] = CreatePrincipledMaterial(scene, Spectrum{ 0.9f, 0.5f, 0.3f }, 1, 0.1f, 0.25f, 1.5f, 0.0f, 0.0f, 0.0f);
        outers[0] = CreatePrincipledMaterial(scene, Spectrum{ 0.9f, 0.5f, 0.3f }, 1, 0.1f, 0.5f, 1.5f, 0.0f, 0.0f, 0.0f);
        outers[2] = CreatePrincipledMaterial(scene, Spectrum{ 0.9f, 0.5f, 0.3f }, 1, 0.1f, 0.74f, 1.5f, 0.0f, 0.0f, 0.0f);
        outers[4] = CreatePrincipledMaterial(scene, Spectrum{ 0.9f, 0.5f, 0.3f }, 1, 0.1f, 1.0f, 1.5f, 0.0f, 0.0f, 0.0f);
    }
    break;

    case 4:
    {
        Spectrum c0 = Spectrum(1);
        Spectrum c1 = Spectrum{ 1.0f, 0.5f, 0.8f };

        outers[3] = CreatePrincipledMaterial(scene, Lerp(c0, c1, 0.0f), 0.0f, 0.1f, 0.0f, 1.5f, 1.0f, 0.0f, 0.0f);
        outers[1] = CreatePrincipledMaterial(scene, Lerp(c0, c1, 0.25f), 0.0f, 0.1f, 0.0f, 1.5f, 1.0f, 0.0f, 0.0f);
        outers[0] = CreatePrincipledMaterial(scene, Lerp(c0, c1, 0.5f), 0.0f, 0.1f, 0.0f, 1.5f, 1.0f, 0.0f, 0.0f);
        outers[2] = CreatePrincipledMaterial(scene, Lerp(c0, c1, 0.75f), 0.0f, 0.1f, 0.0f, 1.5f, 1.0f, 0.0f, 0.0f);
        outers[4] = CreatePrincipledMaterial(scene, Lerp(c0, c1, 1.0f), 0.0f, 0.1f, 0.0f, 1.5f, 1.0f, 0.0f, 0.0f);
    }
    break;

    case 5:
    {
        outers[3] = CreatePrincipledMaterial(scene, { 0.6f, 0.0f, 0.0f }, 0.0f, 0.7f, 0.0f, 1.5f, 0.0f, 0.0f, 0.03f);
        outers[1] = CreatePrincipledMaterial(scene, { 0.6f, 0.0f, 0.0f }, 0.0f, 0.7f, 0.0f, 1.5f, 0.0f, 0.25f, 0.03f);
        outers[0] = CreatePrincipledMaterial(scene, { 0.6f, 0.0f, 0.0f }, 0.0f, 0.7f, 0.0f, 1.5f, 0.0f, 0.5f, 0.03f);
        outers[2] = CreatePrincipledMaterial(scene, { 0.6f, 0.0f, 0.0f }, 0.0f, 0.7f, 0.0f, 1.5f, 0.0f, 0.75f, 0.03f);
        outers[4] = CreatePrincipledMaterial(scene, { 0.6f, 0.0f, 0.0f }, 0.0f, 0.7f, 0.0f, 1.5f, 0.0f, 1.0f, 0.03f);
    }
    break;

    case 6:
    {
        outers[3] = CreatePrincipledMaterial(scene, { 0.6f, 0.0f, 0.0f }, 0.0f, 0.7f, 0.0f, 1.5f, 0.0f, 1.0f, 0.0f);
        outers[1] = CreatePrincipledMaterial(scene, { 0.6f, 0.0f, 0.0f }, 0.0f, 0.7f, 0.0f, 1.5f, 0.0f, 1.0f, 0.1f);
        outers[0] = CreatePrincipledMaterial(scene, { 0.6f, 0.0f, 0.0f }, 0.0f, 0.7f, 0.0f, 1.5f, 0.0f, 1.0f, 0.2f);
        outers[2] = CreatePrincipledMaterial(scene, { 0.6f, 0.0f, 0.0f }, 0.0f, 0.7f, 0.0f, 1.5f, 0.0f, 1.0f, 0.3f);
        outers[4] = CreatePrincipledMaterial(scene, { 0.6f, 0.0f, 0.0f }, 0.0f, 0.7f, 0.0f, 1.5f, 0.0f, 1.0f, 0.4f);
    }
    break;

    case 7:
    {
        outers[3] = CreatePrincipledMaterial(scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 0.0f, 0.2f);
        outers[1] = CreatePrincipledMaterial(scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 0.25f, 0.2f);
        outers[0] = CreatePrincipledMaterial(scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 0.5f, 0.2f);
        outers[2] = CreatePrincipledMaterial(scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 0.75f, 0.2f);
        outers[4] = CreatePrincipledMaterial(scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 1.0f, 0.2f);
    }
    break;

    case 8:
    {
        Spectrum color = { 0.3f, 0.0f, 0.01f };
        outers[3] = CreatePrincipledMaterial(scene, color, 0.0f, 0.3f, 0.0f, 1.5f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f);
        outers[1] = CreatePrincipledMaterial(scene, color, 0.0f, 0.3f, 0.0f, 1.5f, 0.0f, 0.0f, 0.0f, 1.0f, 0.25f);
        outers[0] = CreatePrincipledMaterial(scene, color, 0.0f, 0.3f, 0.0f, 1.5f, 0.0f, 0.0f, 0.0f, 1.0f, 0.5f);
        outers[2] = CreatePrincipledMaterial(scene, color, 0.0f, 0.3f, 0.0f, 1.5f, 0.0f, 0.0f, 0.0f, 1.0f, 0.75f);
        outers[4] = CreatePrincipledMaterial(scene, color, 0.0f, 0.3f, 0.0f, 1.5f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f);
    }
    break;

    default:
        break;
    }

    const Material* inners[count];
    inners[0] = outers[0];
    inners[1] = outers[1];
    inners[2] = outers[2];
    inners[3] = outers[3];
    inners[4] = outers[4];

    for (int32 j = 0; j < h; ++j)
    {
        for (int32 i = 0; i < w; ++i)
        {
            int32 sign_i = std::pow<int32>(-1, i);
            int32 sign_j = std::pow<int32>(-1, j);
            Vec3 p = o + (sign_j * y * ((j + 1) / 2)) + (sign_i * x * ((i + 1) / 2));

            // SetLoaderFallbackMediumInterface(mi);

            SetLoaderFallbackMaterial(outers[std::min(i + j * w, count)]);
            if (model == 0)
            {
                // https://github.com/lighttransport/lighttransportequation-orb
                auto tf = Transform{ p, Quat::FromEuler({ 0, 0, 0 }), Vec3(2.0f) };
                LoadModel(scene, "res/mori_knob/base.obj", tf);

                tf = Transform{ p, Quat::FromEuler({ 0, pi, 0 }), Vec3(2.0f) };
                LoadModel(scene, "res/mori_knob/outer.obj", tf);
            }
            else
            {
                auto tf = Transform{ p + Vec3(0, 0.0f, 0), Quat::FromEuler({ 0, pi, 0 }), Vec3(0.25f) };
                LoadModel(scene, "res/cloth.glb", tf);
            }

            // SetLoaderFallbackMaterial(inners[std::min(i + j * w, count)]);
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
    // CreateImageInfiniteLight(scene, "res/HDR/peppermint_powerplant_4k.hdr", Transform(Quat(pi / 2, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/quarry_04_puresky_1k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/solitude_night_1k.hdr");
    // CreateImageInfiniteLight(scene, "res/HDR/sunflowers_puresky_1k.hdr");
    // CreateImageInfiniteLight(scene, "res/HDR/san_giuseppe_bridge_4k.hdr", Transform(Quat(-pi, y_axis)));
    // CreateUniformInfiniteLight(scene, Spectrum(1));

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
    Float aperture = 0.01f;
    Float vFov = 30.0;

    return std::make_unique<PerspectiveCamera>(lookfrom, lookat, y_axis, vFov, aperture, dist_to_focus, Point2i(width, height));
}

enum
{
    roughness = 0,
    ior = 1,
    transmission = 2,
    anisotrophy = 3,
    colored_transmission = 4,
    clearcoat = 5,
    clearcoat_roughness = 6,
    metallic = 7,
    sheen = 8
};

enum
{
    knob = 0,
    cloth = 1
};

static int32 index0 = Sample::Register("principled0", std::bind(Principled, std::placeholders::_1, roughness, knob));
static int32 index1 = Sample::Register("principled1", std::bind(Principled, std::placeholders::_1, ior, knob));
static int32 index2 = Sample::Register("principled2", std::bind(Principled, std::placeholders::_1, transmission, knob));
static int32 index3 = Sample::Register("principled3", std::bind(Principled, std::placeholders::_1, anisotrophy, knob));
static int32 index4 = Sample::Register("principled4", std::bind(Principled, std::placeholders::_1, colored_transmission, knob));
static int32 index5 = Sample::Register("principled5", std::bind(Principled, std::placeholders::_1, clearcoat, cloth));
static int32 index6 = Sample::Register("principled6", std::bind(Principled, std::placeholders::_1, clearcoat_roughness, cloth));
static int32 index7 = Sample::Register("principled7", std::bind(Principled, std::placeholders::_1, metallic, knob));
static int32 index8 = Sample::Register("principled8", std::bind(Principled, std::placeholders::_1, sheen, cloth));