#include "../samples.h"

void FurnacePrincipled2(RendererInfo* ri)
{
    Scene& scene = ri->scene;

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

    Spectrum color = { 1.0f, 1.0f, 1.0f };
    Float metallic = 1.0f;
    Float transmission = 1.0f;
    Float anisotrophy = 0.0f;
    Float ior = 1.5f;

    outers[9] = CreatePrincipledMaterial(scene, color, metallic, 0.0f, anisotrophy, ior, transmission);
    outers[7] = CreatePrincipledMaterial(scene, color, metallic, 0.1f, anisotrophy, ior, transmission);
    outers[5] = CreatePrincipledMaterial(scene, color, metallic, 0.2f, anisotrophy, ior, transmission);
    outers[3] = CreatePrincipledMaterial(scene, color, metallic, 0.3f, anisotrophy, ior, transmission);
    outers[1] = CreatePrincipledMaterial(scene, color, metallic, 0.4f, anisotrophy, ior, transmission);
    outers[0] = CreatePrincipledMaterial(scene, color, metallic, 0.5f, anisotrophy, ior, transmission);
    outers[2] = CreatePrincipledMaterial(scene, color, metallic, 0.6f, anisotrophy, ior, transmission);
    outers[4] = CreatePrincipledMaterial(scene, color, metallic, 0.7f, anisotrophy, ior, transmission);
    outers[6] = CreatePrincipledMaterial(scene, color, metallic, 0.8f, anisotrophy, ior, transmission);
    outers[8] = CreatePrincipledMaterial(scene, color, metallic, 0.9f, anisotrophy, ior, transmission);
    outers[10] = CreatePrincipledMaterial(scene, color, metallic, 1.0f, anisotrophy, ior, transmission);

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
            auto tf = Transform{ p, Quat::FromEuler({ 0, 0, 0 }), Vec3(2.2f) };
            LoadModel(scene, "res/mori_knob/base.obj", tf, options);

            LoadModel(scene, "res/mori_knob/outer.obj", tf, options);
            LoadModel(scene, "res/mori_knob/inner.obj", tf, options);
            // LoadModel(scene, "res/mori_knob/equation.obj", tf, options);
        }
    }

    CreateUniformInfiniteLight(scene, Spectrum(1));
    // CreateImageInfiniteLight(scene, "res/HDR/aerodynamics_workshop_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/peppermint_powerplant_4k.hdr", Transform(Quat(pi / 2, y_axis)));

    Float aspect_ratio = 8.f / 1.f;
    int32 width = 1600;
    int32 height = int32(width / aspect_ratio);

    Point3 position = Point3{ 0, 0.25, 4.75 };
    Point3 target = Point3{ 0.0, 0.25, 0.0 };

    Float aperture = 0.0f;
    Float fov = 15.0;

    ri->integrator_info.type = IntegratorType::path;
    ri->integrator_info.max_bounces = 1024;
    ri->integrator_info.rr_min_bounces = 64;
    ri->camera_info.type = CameraType::perspective;
    ri->camera_info.transform = Transform::LookAt(position, target, y_axis);
    ri->camera_info.fov = fov;
    ri->camera_info.aperture_radius = aperture;
    ri->camera_info.focus_distance = Dist(position, target);
    ri->camera_info.film_info.filename = "";
    ri->camera_info.film_info.resolution = { width, height };
    ri->camera_info.sampler_info.type = SamplerType::stratified;
    ri->camera_info.sampler_info.spp = 64;
}

void FurnaceDielectric2(RendererInfo* ri)
{
    Scene& scene = ri->scene;

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

    Spectrum color = { 1.0f, 1.0f, 1.0f };
    Float metallic = 0.0f;
    Float transmission = 1.0f;
    Float anisotrophy = 0.0f;
    Float ior = 1.5f;
    bool energy_compensation = true;

    outers[9] = CreateDielectricMaterial(scene, ior, 0.0f, color, energy_compensation);
    outers[7] = CreateDielectricMaterial(scene, ior, 0.1f, color, energy_compensation);
    outers[5] = CreateDielectricMaterial(scene, ior, 0.2f, color, energy_compensation);
    outers[3] = CreateDielectricMaterial(scene, ior, 0.3f, color, energy_compensation);
    outers[1] = CreateDielectricMaterial(scene, ior, 0.4f, color, energy_compensation);
    outers[0] = CreateDielectricMaterial(scene, ior, 0.5f, color, energy_compensation);
    outers[2] = CreateDielectricMaterial(scene, ior, 0.6f, color, energy_compensation);
    outers[4] = CreateDielectricMaterial(scene, ior, 0.7f, color, energy_compensation);
    outers[6] = CreateDielectricMaterial(scene, ior, 0.8f, color, energy_compensation);
    outers[8] = CreateDielectricMaterial(scene, ior, 0.9f, color, energy_compensation);
    outers[10] = CreateDielectricMaterial(scene, ior, 1.0f, color, energy_compensation);

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
            auto tf = Transform{ p, Quat::FromEuler({ 0, 0, 0 }), Vec3(2.2f) };
            LoadModel(scene, "res/mori_knob/base.obj", tf, options);

            tf = Transform{ p, Quat::FromEuler({ 0, 0, 0 }), Vec3(2.2f) };
            // LoadModel(scene, "res/mori_knob/outer.obj", tf, options);
            LoadModel(scene, "res/mori_knob/inner.obj", tf, options);
            // LoadModel(scene, "res/mori_knob/equation.obj", tf, options);
        }
    }

    CreateUniformInfiniteLight(scene, Spectrum(1));
    // CreateImageInfiniteLight(scene, "res/HDR/aerodynamics_workshop_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/peppermint_powerplant_4k.hdr", Transform(Quat(pi / 2, y_axis)));

    Float aspect_ratio = 8.f / 1.f;
    int32 width = 1600;
    int32 height = int32(width / aspect_ratio);

    Point3 position = Point3{ 0, 0.25, 4.75 };
    Point3 target = Point3{ 0.0, 0.25, 0.0 };

    Float aperture = 0.0f;
    Float fov = 15.0;

    ri->integrator_info.type = IntegratorType::path;
    ri->integrator_info.max_bounces = 1024;
    ri->integrator_info.rr_min_bounces = 64;
    ri->camera_info.type = CameraType::perspective;
    ri->camera_info.transform = Transform::LookAt(position, target, y_axis);
    ri->camera_info.fov = fov;
    ri->camera_info.aperture_radius = aperture;
    ri->camera_info.focus_distance = Dist(position, target);
    ri->camera_info.film_info.filename = "";
    ri->camera_info.film_info.resolution = { width, height };
    ri->camera_info.sampler_info.type = SamplerType::stratified;
    ri->camera_info.sampler_info.spp = 64;
}

void FurnaceConductor2(RendererInfo* ri)
{
    Scene& scene = ri->scene;

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

    Spectrum color = { 1.0f, 1.0f, 1.0f };
    // Spectrum color = { 212 / 255.f, 175 / 255.f, 55 / 255.f };
    Float metallic = 0.0f;
    Float transmission = 1.0f;
    Float anisotrophy = 0.0f;
    Float ior = 1.5f;
    bool energy_compensation = true;
    Spectrum reflectance(1);

    outers[9] = CreateConductorMaterial(scene, color, 0.0, reflectance, energy_compensation);
    outers[7] = CreateConductorMaterial(scene, color, 0.1, reflectance, energy_compensation);
    outers[5] = CreateConductorMaterial(scene, color, 0.2, reflectance, energy_compensation);
    outers[3] = CreateConductorMaterial(scene, color, 0.3, reflectance, energy_compensation);
    outers[1] = CreateConductorMaterial(scene, color, 0.4, reflectance, energy_compensation);
    outers[0] = CreateConductorMaterial(scene, color, 0.5, reflectance, energy_compensation);
    outers[2] = CreateConductorMaterial(scene, color, 0.6, reflectance, energy_compensation);
    outers[4] = CreateConductorMaterial(scene, color, 0.7, reflectance, energy_compensation);
    outers[6] = CreateConductorMaterial(scene, color, 0.8, reflectance, energy_compensation);
    outers[8] = CreateConductorMaterial(scene, color, 0.9, reflectance, energy_compensation);
    outers[10] = CreateConductorMaterial(scene, color, 1.0f, reflectance, energy_compensation);

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
            auto tf = Transform{ p, Quat::FromEuler({ 0, 0, 0 }), Vec3(2.2f) };
            LoadModel(scene, "res/mori_knob/base.obj", tf, options);

            LoadModel(scene, "res/mori_knob/outer.obj", tf, options);
            tf = Transform{ p, Quat::FromEuler({ 0, 0, 0 }), Vec3(2.2f) };
            LoadModel(scene, "res/mori_knob/inner.obj", tf, options);
            // LoadModel(scene, "res/mori_knob/equation.obj", tf, options);
        }
    }

    CreateUniformInfiniteLight(scene, Spectrum(1));
    // CreateImageInfiniteLight(scene, "res/HDR/aerodynamics_workshop_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/peppermint_powerplant_4k.hdr", Transform(Quat(pi / 2, y_axis)));

    Float aspect_ratio = 8.f / 1.f;
    int32 width = 1600;
    int32 height = int32(width / aspect_ratio);

    Point3 position = Point3{ 0, 0.25, 4.75 };
    Point3 target = Point3{ 0.0, 0.25, 0.0 };

    Float aperture = 0.0f;
    Float fov = 15.0;

    ri->integrator_info.type = IntegratorType::path;
    ri->integrator_info.max_bounces = 1024;
    ri->integrator_info.rr_min_bounces = 64;
    ri->camera_info.type = CameraType::perspective;
    ri->camera_info.transform = Transform::LookAt(position, target, y_axis);
    ri->camera_info.fov = fov;
    ri->camera_info.aperture_radius = aperture;
    ri->camera_info.focus_distance = Dist(position, target);
    ri->camera_info.film_info.filename = "";
    ri->camera_info.film_info.resolution = { width, height };
    ri->camera_info.sampler_info.type = SamplerType::stratified;
    ri->camera_info.sampler_info.spp = 64;
}

static int32 index2 = Sample::Register("Furnace", FurnaceConductor2);
static int32 index1 = Sample::Register("Furnace2", FurnaceDielectric2);
static int32 index0 = Sample::Register("Furnace3", FurnacePrincipled2);