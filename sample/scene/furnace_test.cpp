#include "../samples.h"

SceneInfo FurnacePrincipled()
{
    auto scene = std::make_unique<Scene>();

    HomogeneousMedium* hm = scene->CreateMedium<HomogeneousMedium>(Spectrum(0, 0, 0), Spectrum(10), Spectrum(0.0), -0.9f);
    MediumInterface mi(hm, nullptr);

    ModelLoaderOptions options;
    options.use_fallback_material = true;

    // Floor
    {
        // auto checker = CreateSpectrumCheckerTexture(*scene, 0.75, 0.3, Point2(60));
        // auto tf = Transform{ Vec3(0, 0, 0), Quat::FromEuler({ 0, 0, 0 }), Vec3(10) };
        // auto floor = scene->CreateMaterial<DiffuseMaterial>(checker);
        // options.fallback_material = floor;
        // LoadModel(*scene, "res/background.obj", tf, options);
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

    outers[3] = CreatePrincipledMaterial(*scene, color, metallic, 0.0f, anisotrophy, ior, transmission);
    outers[1] = CreatePrincipledMaterial(*scene, color, metallic, 0.25f, anisotrophy, ior, transmission);
    outers[0] = CreatePrincipledMaterial(*scene, color, metallic, 0.5f, anisotrophy, ior, transmission);
    outers[2] = CreatePrincipledMaterial(*scene, color, metallic, 0.75f, anisotrophy, ior, transmission);
    outers[4] = CreatePrincipledMaterial(*scene, color, metallic, 1.0f, anisotrophy, ior, transmission);

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
            auto tf = Transform{ p, Quat::FromEuler({ 0, 0, 0 }), Vec3(2.0f) };
            LoadModel(*scene, "res/mori_knob/base.obj", tf, options);

            LoadModel(*scene, "res/mori_knob/outer.obj", tf, options);
            LoadModel(*scene, "res/mori_knob/inner.obj", tf, options);
            // LoadModel(*scene, "res/mori_knob/equation.obj", tf, options);
        }
    }

    CreateUniformInfiniteLight(*scene, Spectrum(1));
    // CreateImageInfiniteLight(*scene, "res/HDR/aerodynamics_workshop_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(*scene, "res/HDR/peppermint_powerplant_4k.hdr", Transform(Quat(pi / 2, y_axis)));

    Float aspect_ratio = 4.f / 1.f;
    int32 width = 1600;
    int32 height = int32(width / aspect_ratio);

    Point3 position = Point3{ 0, 1.0, 2.28 };
    Point3 target = Point3{ 0.0, 0.1, 0.0 };

    Float aperture = 0.01f;
    Float fov = 30.0;

    SceneInfo si;
    si.scene = std::move(scene);
    si.renderer_info.type = IntegratorType::path;
    si.renderer_info.max_bounces = 64;
    si.camera_info.type = CameraType::perspective;
    si.camera_info.tf = Transform::LookAt(position, target, y_axis);
    si.camera_info.fov = fov;
    si.camera_info.aperture = aperture;
    si.camera_info.focus_distance = Dist(position, target);
    si.camera_info.film_info.filename = "";
    si.camera_info.film_info.resolution = { width, height };
    si.camera_info.sampler_info.type = SamplerType::stratified;
    si.camera_info.sampler_info.spp = 64;

    return si;
}

SceneInfo FurnaceDielectric()
{
    auto scene = std::make_unique<Scene>();

    HomogeneousMedium* hm = scene->CreateMedium<HomogeneousMedium>(Spectrum(0, 0, 0), Spectrum(10), Spectrum(0.0), -0.9f);
    MediumInterface mi(hm, nullptr);

    ModelLoaderOptions options;
    options.use_fallback_material = true;

    // Floor
    {
        // auto checker = CreateSpectrumCheckerTexture(*scene, 0.75, 0.3, Point2(60));
        // auto tf = Transform{ Vec3(0, 0, 0), Quat::FromEuler({ 0, 0, 0 }), Vec3(10) };
        // auto floor = scene->CreateMaterial<DiffuseMaterial>(checker);
        // options.fallback_material = floor;
        // LoadModel(*scene, "res/background.obj", tf, options);
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

    outers[3] = CreateDielectricMaterial(*scene, ior, 0.0f, color, energy_compensation);
    outers[1] = CreateDielectricMaterial(*scene, ior, 0.01f, color, energy_compensation);
    outers[0] = CreateDielectricMaterial(*scene, ior, 0.05f, color, energy_compensation);
    outers[2] = CreateDielectricMaterial(*scene, ior, 0.1f, color, energy_compensation);
    outers[4] = CreateDielectricMaterial(*scene, ior, 0.2f, color, energy_compensation);

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
            auto tf = Transform{ p, Quat::FromEuler({ 0, 0, 0 }), Vec3(2.0f) };
            LoadModel(*scene, "res/mori_knob/base.obj", tf, options);

            tf = Transform{ p, Quat::FromEuler({ 0, 0, 0 }), Vec3(2.0f) };
            LoadModel(*scene, "res/mori_knob/outer.obj", tf, options);
            LoadModel(*scene, "res/mori_knob/inner.obj", tf, options);
            // LoadModel(*scene, "res/mori_knob/equation.obj", tf, options);
        }
    }

    CreateUniformInfiniteLight(*scene, Spectrum(1));
    // CreateImageInfiniteLight(*scene, "res/HDR/aerodynamics_workshop_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(*scene, "res/HDR/peppermint_powerplant_4k.hdr", Transform(Quat(pi / 2, y_axis)));

    Float aspect_ratio = 4.f / 1.f;
    int32 width = 1600;
    int32 height = int32(width / aspect_ratio);

    Point3 position = Point3{ 0, 1.0, 2.28 };
    Point3 target = Point3{ 0.0, 0.1, 0.0 };

    Float aperture = 0.01f;
    Float fov = 30.0;

    SceneInfo si;
    si.scene = std::move(scene);
    si.renderer_info.type = IntegratorType::path;
    si.renderer_info.max_bounces = 64;
    si.camera_info.type = CameraType::perspective;
    si.camera_info.tf = Transform::LookAt(position, target, y_axis);
    si.camera_info.fov = fov;
    si.camera_info.aperture = aperture;
    si.camera_info.focus_distance = Dist(position, target);
    si.camera_info.film_info.filename = "";
    si.camera_info.film_info.resolution = { width, height };
    si.camera_info.sampler_info.type = SamplerType::stratified;
    si.camera_info.sampler_info.spp = 64;

    return si;
}

SceneInfo FurnaceConductor()
{
    auto scene = std::make_unique<Scene>();

    HomogeneousMedium* hm = scene->CreateMedium<HomogeneousMedium>(Spectrum(0, 0, 0), Spectrum(10), Spectrum(0.0), -0.9f);
    MediumInterface mi(hm, nullptr);

    ModelLoaderOptions options;
    options.use_fallback_material = true;

    // Floor
    {
        // auto checker = CreateSpectrumCheckerTexture(*scene, 0.75, 0.3, Point2(60));
        // auto tf = Transform{ Vec3(0, 0, 0), Quat::FromEuler({ 0, 0, 0 }), Vec3(10) };
        // auto floor = scene->CreateMaterial<DiffuseMaterial>(checker);
        // options.fallback_material = floor;
        // LoadModel(*scene, "res/background.obj", tf, options);
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

    outers[3] = CreateConductorMaterial(*scene, color, 0.0f, energy_compensation);
    outers[1] = CreateConductorMaterial(*scene, color, 0.25f, energy_compensation);
    outers[0] = CreateConductorMaterial(*scene, color, 0.5f, energy_compensation);
    outers[2] = CreateConductorMaterial(*scene, color, 0.75f, energy_compensation);
    outers[4] = CreateConductorMaterial(*scene, color, 1.0f, energy_compensation);

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
            auto tf = Transform{ p, Quat::FromEuler({ 0, 0, 0 }), Vec3(2.0f) };
            LoadModel(*scene, "res/mori_knob/base.obj", tf, options);

            LoadModel(*scene, "res/mori_knob/outer.obj", tf, options);
            tf = Transform{ p, Quat::FromEuler({ 0, 0, 0 }), Vec3(2.1f) };
            LoadModel(*scene, "res/mori_knob/inner.obj", tf, options);
            // LoadModel(*scene, "res/mori_knob/equation.obj", tf, options);
        }
    }

    CreateUniformInfiniteLight(*scene, Spectrum(1));
    // CreateImageInfiniteLight(*scene, "res/HDR/aerodynamics_workshop_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(*scene, "res/HDR/peppermint_powerplant_4k.hdr", Transform(Quat(pi / 2, y_axis)));

    Float aspect_ratio = 4.f / 1.f;
    int32 width = 1600;
    int32 height = int32(width / aspect_ratio);

    Point3 position = Point3{ 0, 1.0, 2.28 };
    Point3 target = Point3{ 0.0, 0.1, 0.0 };

    Float aperture = 0.01f;
    Float fov = 30.0;

    SceneInfo si;
    si.scene = std::move(scene);
    si.renderer_info.type = IntegratorType::path;
    si.renderer_info.max_bounces = 64;
    si.camera_info.type = CameraType::perspective;
    si.camera_info.tf = Transform::LookAt(position, target, y_axis);
    si.camera_info.fov = fov;
    si.camera_info.aperture = aperture;
    si.camera_info.focus_distance = Dist(position, target);
    si.camera_info.film_info.filename = "";
    si.camera_info.film_info.resolution = { width, height };
    si.camera_info.sampler_info.type = SamplerType::stratified;
    si.camera_info.sampler_info.spp = 64;

    return si;
}

static int32 index0 = Sample::Register("furnace", FurnacePrincipled);
static int32 index1 = Sample::Register("furnace2", FurnaceDielectric);
static int32 index2 = Sample::Register("furnace3", FurnaceConductor);