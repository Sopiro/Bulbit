#include "../samples.h"

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
    sheen = 8,
    colored_sheen = 9,
    colored_coat = 10,
    multi_scattering = 11,
    multi_scattering_2 = 12,
};

enum
{
    knob = 0,
    cloth = 1
};

RendererInfo Principled(int32 lobe, int32 model)
{
    auto scene = std::make_unique<Scene>();

    HomogeneousMedium* hm = scene->CreateMedium<HomogeneousMedium>(Spectrum(0, 0, 0), Spectrum(10), Spectrum(0.0), -0.9f);
    MediumInterface mi(hm, nullptr);

    ModelLoaderOptions options;
    options.use_fallback_material = true;

    // Floor
    {
        auto checker = CreateSpectrumCheckerTexture(*scene, 0.75, 0.3, Point2(60));
        auto tf = Transform{ Vec3(0, 0, 0), Quat::FromEuler({ 0, 0, 0 }), Vec3(10) };
        auto floor = scene->CreateMaterial<DiffuseMaterial>(checker);
        options.fallback_material = floor;
        LoadModel(*scene, "res/background.obj", tf, options);
    }

    int32 w = 5;
    int32 h = 1;

    Vec3 o(0, 0, 0.4);
    Vec3 x(0.8, 0, 0);
    Vec3 y(0, 0, 1);

    const int32 count = 5;

    auto normalmap = CreateSpectrumImageTexture(*scene, "res/bistro/Concrete_Normal.png", true);

    const Material* outers[count];
    switch (lobe)
    {
    case roughness:
    {
        outers[3] = CreatePrincipledMaterial(*scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 1, 0.1f);
        outers[1] = CreatePrincipledMaterial(*scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 1, 0.3f);
        outers[0] = CreatePrincipledMaterial(*scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 1, 0.5f);
        outers[2] = CreatePrincipledMaterial(*scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 1, 0.7f);
        outers[4] = CreatePrincipledMaterial(*scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 1, 0.9f);
    }
    break;

    case ior:
    {
        outers[3] = CreatePrincipledMaterial(*scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 0, 0.1f, 0.0f, 1.0f);
        outers[1] = CreatePrincipledMaterial(*scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 0, 0.1f, 0.0f, 1.25f);
        outers[0] = CreatePrincipledMaterial(*scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 0, 0.1f, 0.0f, 1.5f);
        outers[2] = CreatePrincipledMaterial(*scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 0, 0.1f, 0.0f, 1.75f);
        outers[4] = CreatePrincipledMaterial(*scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 0, 0.1f, 0.0f, 2.0f);
    }
    break;

    case transmission:
    {
        outers[3] = CreatePrincipledMaterial(*scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 0, 0.1f, 0.0f, 1.5f, 0.0f);
        outers[1] = CreatePrincipledMaterial(*scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 0, 0.1f, 0.0f, 1.5f, 0.25f);
        outers[0] = CreatePrincipledMaterial(*scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 0, 0.1f, 0.0f, 1.5f, 0.5f);
        outers[2] = CreatePrincipledMaterial(*scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 0, 0.1f, 0.0f, 1.5f, 0.75f);
        outers[4] = CreatePrincipledMaterial(*scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 0, 0.1f, 0.0f, 1.5f, 1.0f);
    }
    break;

    case anisotrophy:
    {
        outers[3] = CreatePrincipledMaterial(*scene, Spectrum{ 0.9f, 0.5f, 0.3f }, 1, 0.1f, 0.0f);
        outers[1] = CreatePrincipledMaterial(*scene, Spectrum{ 0.9f, 0.5f, 0.3f }, 1, 0.1f, 0.25f);
        outers[0] = CreatePrincipledMaterial(*scene, Spectrum{ 0.9f, 0.5f, 0.3f }, 1, 0.1f, 0.5f);
        outers[2] = CreatePrincipledMaterial(*scene, Spectrum{ 0.9f, 0.5f, 0.3f }, 1, 0.1f, 0.74f);
        outers[4] = CreatePrincipledMaterial(*scene, Spectrum{ 0.9f, 0.5f, 0.3f }, 1, 0.1f, 1.0f);
    }
    break;

    case colored_transmission:
    {
        Spectrum c0 = Spectrum(1);
        Spectrum c1 = Spectrum{ 1.0f, 0.5f, 0.8f };

        outers[3] = CreatePrincipledMaterial(*scene, Lerp(c0, c1, 0.0f), 0.0f, 0.1f, 0.0f, 1.5f, 1.0f, 0.0f, 0.0f);
        outers[1] = CreatePrincipledMaterial(*scene, Lerp(c0, c1, 0.25f), 0.0f, 0.1f, 0.0f, 1.5f, 1.0f, 0.0f, 0.0f);
        outers[0] = CreatePrincipledMaterial(*scene, Lerp(c0, c1, 0.5f), 0.0f, 0.1f, 0.0f, 1.5f, 1.0f, 0.0f, 0.0f);
        outers[2] = CreatePrincipledMaterial(*scene, Lerp(c0, c1, 0.75f), 0.0f, 0.1f, 0.0f, 1.5f, 1.0f, 0.0f, 0.0f);
        outers[4] = CreatePrincipledMaterial(*scene, Lerp(c0, c1, 1.0f), 0.0f, 0.1f, 0.0f, 1.5f, 1.0f, 0.0f, 0.0f);
    }
    break;

    case clearcoat:
    {
        outers[3] = CreatePrincipledMaterial(*scene, { 0.6f, 0.0f, 0.0f }, 0.0f, 0.7f, 0.0f, 1.5f, 0.0f, 0.0f, 0.03f);
        outers[1] = CreatePrincipledMaterial(*scene, { 0.6f, 0.0f, 0.0f }, 0.0f, 0.7f, 0.0f, 1.5f, 0.0f, 0.25f, 0.03f);
        outers[0] = CreatePrincipledMaterial(*scene, { 0.6f, 0.0f, 0.0f }, 0.0f, 0.7f, 0.0f, 1.5f, 0.0f, 0.5f, 0.03f);
        outers[2] = CreatePrincipledMaterial(*scene, { 0.6f, 0.0f, 0.0f }, 0.0f, 0.7f, 0.0f, 1.5f, 0.0f, 0.75f, 0.03f);
        outers[4] = CreatePrincipledMaterial(*scene, { 0.6f, 0.0f, 0.0f }, 0.0f, 0.7f, 0.0f, 1.5f, 0.0f, 1.0f, 0.03f);
    }
    break;

    case clearcoat_roughness:
    {
        outers[3] = CreatePrincipledMaterial(*scene, { 0.6f, 0.0f, 0.0f }, 0.0f, 0.7f, 0.0f, 1.5f, 0.0f, 1.0f, 0.0f);
        outers[1] = CreatePrincipledMaterial(*scene, { 0.6f, 0.0f, 0.0f }, 0.0f, 0.7f, 0.0f, 1.5f, 0.0f, 1.0f, 0.1f);
        outers[0] = CreatePrincipledMaterial(*scene, { 0.6f, 0.0f, 0.0f }, 0.0f, 0.7f, 0.0f, 1.5f, 0.0f, 1.0f, 0.2f);
        outers[2] = CreatePrincipledMaterial(*scene, { 0.6f, 0.0f, 0.0f }, 0.0f, 0.7f, 0.0f, 1.5f, 0.0f, 1.0f, 0.3f);
        outers[4] = CreatePrincipledMaterial(*scene, { 0.6f, 0.0f, 0.0f }, 0.0f, 0.7f, 0.0f, 1.5f, 0.0f, 1.0f, 0.4f);
    }
    break;

    case metallic:
    {
        outers[3] = CreatePrincipledMaterial(*scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 0.0f, 0.2f);
        outers[1] = CreatePrincipledMaterial(*scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 0.25f, 0.2f);
        outers[0] = CreatePrincipledMaterial(*scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 0.5f, 0.2f);
        outers[2] = CreatePrincipledMaterial(*scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 0.75f, 0.2f);
        outers[4] = CreatePrincipledMaterial(*scene, Spectrum{ 1.0f, 0.5f, 0.3f }, 1.0f, 0.2f);
    }
    break;

    case sheen:
    {
        Spectrum color = { 0.3f, 0.0f, 0.01f };
        outers[3] = CreatePrincipledMaterial(*scene, color, 0.0f, 0.3f, 0.0f, 1.5f, 0.0f, 0.0f, 0.0f, Spectrum(1), 1.0f, 0.0f);
        outers[1] = CreatePrincipledMaterial(*scene, color, 0.0f, 0.3f, 0.0f, 1.5f, 0.0f, 0.0f, 0.0f, Spectrum(1), 1.0f, 0.25f);
        outers[0] = CreatePrincipledMaterial(*scene, color, 0.0f, 0.3f, 0.0f, 1.5f, 0.0f, 0.0f, 0.0f, Spectrum(1), 1.0f, 0.5f);
        outers[2] = CreatePrincipledMaterial(*scene, color, 0.0f, 0.3f, 0.0f, 1.5f, 0.0f, 0.0f, 0.0f, Spectrum(1), 1.0f, 0.75f);
        outers[4] = CreatePrincipledMaterial(*scene, color, 0.0f, 0.3f, 0.0f, 1.5f, 0.0f, 0.0f, 0.0f, Spectrum(1), 1.0f, 1.0f);
    }
    break;

    case colored_sheen:
    {
        Spectrum color = { 0.7f, 0.7f, 0.7f };

#define default_val 0.0f, 0.3f, 0.0f, 1.5f, 0.0f, 0.0f, 0.0f, Spectrum(1), 1.0f, 0.65f
        outers[3] = CreatePrincipledMaterial(*scene, color, default_val, { 1, 0, 0 });
        outers[1] = CreatePrincipledMaterial(*scene, color, default_val, { 0, 1, 0 });
        outers[0] = CreatePrincipledMaterial(*scene, color, default_val, { 0, 0, 1 });
        outers[2] = CreatePrincipledMaterial(*scene, color, default_val, { 1, 1, 0 });
        outers[4] = CreatePrincipledMaterial(*scene, color, default_val, { 1, 0, 1 });
#undef default_val
    }
    break;

    case colored_coat:
    {
        Spectrum color = { 0.7f, 0.7f, 0.7f };

#define default_val 0.0f, 0.3f, 0.0f, 1.5f, 0.0f, 1.0f, 0.03f
        outers[3] = CreatePrincipledMaterial(*scene, color, default_val, { 1, 0, 0 });
        outers[1] = CreatePrincipledMaterial(*scene, color, default_val, { 0, 1, 0 });
        outers[0] = CreatePrincipledMaterial(*scene, color, default_val, { 0, 0, 1 });
        outers[2] = CreatePrincipledMaterial(*scene, color, default_val, { 1, 1, 0 });
        outers[4] = CreatePrincipledMaterial(*scene, color, default_val, { 1, 0, 1 });
#undef default_val
    }
    break;

    case multi_scattering:
    {
        Spectrum color = { 1.0f, 1.0f, 1.0f };
        Float metallic = 1.0f;
        Float transmission = 0.0f;

        outers[3] = CreatePrincipledMaterial(*scene, color, metallic, 0.0f, 0.0f, 1.5f, transmission);
        outers[1] = CreatePrincipledMaterial(*scene, color, metallic, 0.25f, 0.0f, 1.5f, transmission);
        outers[0] = CreatePrincipledMaterial(*scene, color, metallic, 0.5f, 0.0f, 1.5f, transmission);
        outers[2] = CreatePrincipledMaterial(*scene, color, metallic, 0.75f, 0.0f, 1.5f, transmission);
        outers[4] = CreatePrincipledMaterial(*scene, color, metallic, 1.0f, 0.0f, 1.5f, transmission);
    }
    break;

    case multi_scattering_2:
    {
        Spectrum color = { 1.0f, 1.0f, 1.0f };
        Float metallic = 0.0f;
        Float transmission = 1.0f;
        Float ior = 1.5f;

        outers[3] = CreatePrincipledMaterial(*scene, color, metallic, 0.0f, 0.0f, ior, transmission);
        outers[1] = CreatePrincipledMaterial(*scene, color, metallic, 0.25f, 0.0f, ior, transmission);
        outers[0] = CreatePrincipledMaterial(*scene, color, metallic, 0.5f, 0.0f, ior, transmission);
        outers[2] = CreatePrincipledMaterial(*scene, color, metallic, 0.75f, 0.0f, ior, transmission);
        outers[4] = CreatePrincipledMaterial(*scene, color, metallic, 1.0f, 0.0f, ior, transmission);
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

            // options.fallback_medium_interface = mi;

            options.fallback_material = outers[std::min(i + j * w, count)];
            if (model == 0)
            {
                // https://github.com/lighttransport/lighttransportequation-orb
                auto tf = Transform{ p, Quat::FromEuler({ 0, 0, 0 }), Vec3(2.0f) };
                LoadModel(*scene, "res/mori_knob/base.obj", tf, options);

                tf = Transform{ p, Quat::FromEuler({ 0, 0, 0 }), Vec3(2.0f) };
                LoadModel(*scene, "res/mori_knob/outer.obj", tf, options);

                LoadModel(*scene, "res/mori_knob/inner.obj", tf, options);
                LoadModel(*scene, "res/mori_knob/equation.obj", tf, options);
            }
            else
            {
                auto tf = Transform{ p + Vec3(0, 0.0f, 0), Quat::FromEuler({ 0, pi, 0 }), Vec3(0.25f) };
                LoadModel(*scene, "res/cloth.glb", tf);
            }

            // options.fallback_material = inners[std::min(i + j * w, count)];
            // LoadModel(*scene, "res/mori_knob/inner.obj", tf, options);
            // LoadModel(*scene, "res/mori_knob/equation.obj", tf, options);
        }
    }

    // CreateImageInfiniteLight(*scene,
    //     "res/material_test_ball/envmap.hdr", Transform(Quat::FromEuler(0, DegToRad(-67.26139831542969), 0))
    // );
    // CreateImageInfiniteLight(*scene, "res/HDR/photo_studio_loft_hall_1k.hdr", Transform(Quat(pi, y_axis)));
    CreateImageInfiniteLight(*scene, "res/HDR/aerodynamics_workshop_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(*scene, "res/HDR/photo_studio_01_1k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(*scene, "res/HDR/scythian_tombs_2_4k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(*scene, "res/HDR/material-test.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(*scene, "res/HDR/peppermint_powerplant_4k.hdr", Transform(Quat(pi / 2, y_axis)));
    // CreateImageInfiniteLight(*scene, "res/HDR/quarry_04_puresky_1k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(*scene, "res/HDR/abandoned_garage_4k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(*scene, "res/HDR/skylit_garage_4k.hdr", Transform(Quat(0, y_axis)), 0.5f);
    // CreateImageInfiniteLight(*scene, "res/HDR/solitude_night_1k.hdr");
    // CreateImageInfiniteLight(*scene, "res/HDR/sunflowers_puresky_1k.hdr");
    // CreateImageInfiniteLight(*scene, "res/HDR/san_giuseppe_bridge_4k.hdr", Transform(Quat(-pi, y_axis)));
    // CreateUniformInfiniteLight(*scene, Spectrum(1));

    Float aspect_ratio = 4.f / 1.f;
    int32 width = 1600;
    int32 height = int32(width / aspect_ratio);

    Point3 position = Point3{ 0, 1.0, 2.28 };
    Point3 target = Point3{ 0.0, 0.1, 0.0 };

    RendererInfo si;
    si.scene = std::move(scene);
    si.integrator_info.type = IntegratorType::path;
    si.integrator_info.max_bounces = 64;
    si.camera_info.type = CameraType::perspective;
    si.camera_info.transform = Transform::LookAt(position, target, y_axis);
    si.camera_info.fov = 30;
    si.camera_info.aperture_radius = 0.01;
    si.camera_info.focus_distance = Dist(position, target);
    si.camera_info.film_info.filename = "";
    si.camera_info.film_info.resolution = { width, height };
    si.camera_info.sampler_info.type = SamplerType::stratified;
    si.camera_info.sampler_info.spp = 64;

    return si;
}

static int32 index0 = Sample::Register("principled0", std::bind(Principled, roughness, knob));
static int32 index1 = Sample::Register("principled1", std::bind(Principled, ior, knob));
static int32 index2 = Sample::Register("principled2", std::bind(Principled, transmission, knob));
static int32 index3 = Sample::Register("principled3", std::bind(Principled, anisotrophy, knob));
static int32 index4 = Sample::Register("principled4", std::bind(Principled, colored_transmission, knob));
static int32 index5 = Sample::Register("principled5", std::bind(Principled, clearcoat, cloth));
static int32 index6 = Sample::Register("principled6", std::bind(Principled, clearcoat_roughness, cloth));
static int32 index7 = Sample::Register("principled7", std::bind(Principled, metallic, knob));
static int32 index8 = Sample::Register("principled8", std::bind(Principled, sheen, cloth));
static int32 index9 = Sample::Register("principled9", std::bind(Principled, colored_sheen, cloth));
static int32 index10 = Sample::Register("principled10", std::bind(Principled, colored_coat, knob));
static int32 index11 = Sample::Register("principled11", std::bind(Principled, multi_scattering, knob));
static int32 index12 = Sample::Register("principled12", std::bind(Principled, multi_scattering_2, knob));