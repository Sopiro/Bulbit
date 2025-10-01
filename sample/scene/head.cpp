#include "../samples.h"

void Head(RendererInfo* ri)
{
    Scene& scene = ri->scene;

    ModelLoaderOptions options;
    options.use_fallback_material = true;

    // Head
    {
        auto head_albedo = CreateSpectrumImageTexture(scene, "res/head/lambertian.jpg");

        auto mat =
            CreateSubsurfaceRandomWalkMaterial(scene, head_albedo, Spectrum(0.0012953, 0.00095238, 0.00067114), 1.33f, 0.1f);

        // auto l0 = CreateDielectricMaterial(scene, 1.33, 0.3f);
        // auto l1 = scene.CreateMaterial<DiffuseMaterial>(head_albedo);
        // auto mat = CreateLayeredMaterial(scene, l0, l1);

        options.fallback_material = mat;

        auto tf = Transform{ Vec3(0.0f, 0.05f, 0.0f), Quat(DegToRad(0.0f), y_axis), Vec3(1.5f) };
        LoadModel(scene, "res/head/head.obj", tf, options);
    }

    // CreateImageInfiniteLight(scene, "res/HDR/small_rural_road_1k.hdr", Transform(Quat(0, y_axis)), 2);
    // CreateImageInfiniteLight(scene, "res/HDR/material-test.hdr", Transform(Quat(pi / 2, y_axis)));
    CreateImageInfiniteLight(scene, "res/HDR/photo_studio_loft_hall_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/sunset.hdr", Transform(Quat(-pi / 2, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/aerodynamics_workshop_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/scythian_tombs_2_4k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/quarry_04_puresky_1k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/photo_studio_01_1k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/peppermint_powerplant_4k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/white_cliff_top_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/sunflowers_puresky_1k.hdr");
    // CreateImageInfiniteLight(scene, "res/HDR/san_giuseppe_bridge_4k.hdr", Transform(Quat(pi / 2, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/Background_05.hdr", Transform(Quat(pi / 2, y_axis)));
    // CreateUniformInfiniteLight(scene, Spectrum(1));

    Float aspect_ratio = 4.f / 3.f;
    int32 width = 800;
    int32 height = int32(width / aspect_ratio);

    Point3 position = Point3{ -0.3, 0.1, 1 } * 0.8;
    Point3 target = Point3{ 0, 0, 0 };

    Float aperture = 0.01f;
    Float fov = 30;

    ri->integrator_info.type = IntegratorType::path;
    ri->integrator_info.max_bounces = 64;
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

static int32 sample_index = Sample::Register("head", Head);
