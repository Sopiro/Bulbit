#include "../samples.h"

RendererInfo Metals()
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

    const Material* outers[count];

    Float metallic = 0.0f;
    Float transmission = 1.0f;
    Float anisotrophy = 0.0f;
    Float ior = 1.5f;
    bool energy_compensation = true;

    Float roughness = 0.15f;

    // https://refractiveindex.info/
    const Material* iron = CreateConductorMaterial(*scene, { 2.8653, 2.8889, 2.6260 }, { 3.1820, 2.9164, 2.7925 }, roughness);
    const Material* silver =
        CreateConductorMaterial(*scene, { 0.26978, 0.24580, 0.23222 }, { 4.8078, 3.3897, 2.7380 }, roughness);
    const Material* gold = CreateConductorMaterial(*scene, { 0.161, 0.492, 1.426 }, { 4.08769, 2.32625, 1.846 }, roughness);
    const Material* platinum =
        CreateConductorMaterial(*scene, { 0.49745, 0.48267, 0.60399 }, { 6.9266, 4.8444, 3.8895 }, roughness);
    const Material* brass = CreateConductorMaterial(*scene, { 0.446, 0.573, 0.994 }, { 4.1060, 2.5680, 1.883 }, roughness);
    const Material* glass = CreateDielectricMaterial(*scene, 1.0f);
    const Material* air = CreateDielectricMaterial(*scene, 1.0f);
    const Material* copper = CreateConductorMaterial(*scene, { 0.21100, 1.1274, 1.2444 }, { 4.1592, 2.5978, 2.433 }, roughness);
    const Material* patina = CreateLayeredMaterial(*scene, air, copper, true, { 0.25f, 0.95f, 0.45f }, 0.2, 0.1f);

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

            // options.fallback_medium_interface = mi;

            options.fallback_material = outers[std::min(i + j * w, count)];

            // https://github.com/lighttransport/lighttransportequation-orb
            auto tf = Transform{ p, Quat::FromEuler({ 0, 0, 0 }), Vec3(2.0f) };
            LoadModel(*scene, "res/mori_knob/base.obj", tf, options);

            LoadModel(*scene, "res/mori_knob/outer.obj", tf, options);
            tf = Transform{ p, Quat::FromEuler({ 0, 0, 0 }), Vec3(2.1f) };
            LoadModel(*scene, "res/mori_knob/inner.obj", tf, options);
            LoadModel(*scene, "res/mori_knob/equation.obj", tf, options);
        }
    }

    // CreateUniformInfiniteLight(*scene, Spectrum(1));
    CreateImageInfiniteLight(*scene, "res/HDR/aerodynamics_workshop_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(*scene, "res/HDR/peppermint_powerplant_4k.hdr", Transform(Quat(pi / 2, y_axis)));

    Float aspect_ratio = 4.f / 1.f;
    int32 width = 1600;
    int32 height = int32(width / aspect_ratio);

    Point3 position = Point3{ 0, 1.0, 2.28 };
    Point3 target = Point3{ 0.0, 0.1, 0.0 };

    Float aperture = 0.0f;
    Float fov = 30.0;

    RendererInfo si;
    si.scene = std::move(scene);
    si.integrator_info.type = IntegratorType::path;
    si.integrator_info.max_bounces = 64;
    si.camera_info.type = CameraType::perspective;
    si.camera_info.transform = Transform::LookAt(position, target, y_axis);
    si.camera_info.fov = fov;
    si.camera_info.aperture_radius = aperture;
    si.camera_info.focus_distance = Dist(position, target);
    si.camera_info.film_info.filename = "";
    si.camera_info.film_info.resolution = { width, width };
    si.camera_info.sampler_info.type = SamplerType::stratified;
    si.camera_info.sampler_info.spp = 64;

    return si;
}

static int32 index0 = Sample::Register("metals", Metals);
