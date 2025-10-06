#include "../samples.h"

void NanoVDBTest(RendererInfo* ri)
{
    Scene& scene = ri->scene;

    ModelLoaderOptions options;
    options.use_fallback_material = true;

    nanovdb::GridHandle<nanovdb::HostBuffer> handle;
    try
    {
        handle = nanovdb::io::readGrid("C:/Users/sopir/Desktop/bunny_cloud.nvdb");
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }

    if (!handle)
    {
        std::cerr << "Failed to read NanoVDB file!\n";
        exit(1);
    }

    const nanovdb::NanoGrid<float>* grid = handle.grid<float>();

    if (!grid)
    {
        std::cerr << "Not a float grid!\n";
        exit(1);
    }

    std::cout << "Grid name: " << grid->gridName() << std::endl;
    auto bbox = grid->worldBBox();
    Vec3 min(bbox.min()[0], bbox.min()[1], bbox.min()[2]);
    Vec3 max(bbox.max()[0], bbox.max()[1], bbox.max()[2]);
    Vec3 center = (min + max) / 2;
    Vec3 extents = (max - min);

    std::cout << "BBox min: " << min.ToString() << std::endl;
    std::cout << "BBox max: " << max.ToString() << std::endl;
    std::cout << "BBox center: " << center.ToString() << std::endl;
    std::cout << "BBox extents: " << extents.ToString() << std::endl;
    std::cout << "Value range: " << grid->tree().root().minimum() << " ~ " << grid->tree().root().maximum() << std::endl;

    Transform to_origin(-center);
    Transform tf(0, 0, 0, Quat(0, y_axis), Vec3(1));

    Spectrum sigma_a(.1);
    Spectrum sigma_s(1);
    Float sigma_scale = 1;
    Float g = 0.0;

    NanoVDBMedium* medium =
        scene.CreateMedium<NanoVDBMedium>(tf * to_origin, sigma_a, sigma_s, sigma_scale, g, std::move(handle));
    MediumInterface mi_outside(nullptr, medium);
    MediumInterface mi_inside(medium, nullptr);
    MediumInterface mi_two_sided(medium, medium);

    CreateBox(scene, Transform(Point3::zero, tf.q, extents * tf.s), nullptr, mi_inside);

    // CreateImageInfiniteLight(scene, "res/HDR/small_rural_road_1k.hdr", Transform(Quat(0, y_axis)), 2);
    // CreateImageInfiniteLight(scene, "res/HDR/material-test.hdr", Transform(Quat(pi / 2, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/photo_studio_loft_hall_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/sunset.hdr", Transform(Quat(-pi / 2, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/aerodynamics_workshop_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/scythian_tombs_2_4k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/quarry_04_puresky_1k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/photo_studio_01_1k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/peppermint_powerplant_4k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/white_cliff_top_1k.hdr", Transform(Quat(pi, y_axis)));
    CreateImageInfiniteLight(scene, "res/HDR/sunflowers_puresky_1k.hdr", Transform(Quat(pi / 4, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/san_giuseppe_bridge_4k.hdr", Transform(Quat(pi / 2, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/Background_05.hdr", Transform(Quat(pi / 2, y_axis)));
    // CreateUniformInfiniteLight(scene, Spectrum(1));

    Float aspect_ratio = 4.f / 3.f;
    int32 width = 800;
    int32 height = int32(width / aspect_ratio);

    Point3 position = Point3{ 0, 0, 100 };
    Point3 target = Point3{ 0, 0, 0 };

    Float aperture = 0.0f;
    Float fov = 30;

    ri->integrator_info.type = IntegratorType::vol_path;
    ri->integrator_info.max_bounces = 64;
    ri->camera_info.type = CameraType::perspective;
    ri->camera_info.transform = Transform::LookAt(position, target, y_axis);
    ri->camera_info.fov = fov;
    ri->camera_info.aperture_radius = aperture;
    ri->camera_info.focus_distance = Dist(position, target);
    ri->camera_info.film_info.filename = "";
    ri->camera_info.film_info.resolution = { width, height };
    ri->camera_info.sampler_info.type = SamplerType::independent;
    ri->camera_info.sampler_info.spp = 8;
}

void NanoVDBTest2(RendererInfo* ri)
{
    Scene& scene = ri->scene;

    ModelLoaderOptions options;
    options.use_fallback_material = true;

    nanovdb::GridHandle<nanovdb::HostBuffer> handle;
    try
    {
        handle = nanovdb::io::readGrid("C:/Users/sopir/Desktop/wdas_cloud_quarter.nvdb");
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }

    if (!handle)
    {
        std::cerr << "Failed to read NanoVDB file!\n";
        exit(1);
    }

    const nanovdb::NanoGrid<float>* grid = handle.grid<float>();

    if (!grid)
    {
        std::cerr << "Not a float grid!\n";
        exit(1);
    }

    std::cout << "Grid name: " << grid->gridName() << std::endl;
    auto bbox = grid->worldBBox();
    Vec3 min(bbox.min()[0], bbox.min()[1], bbox.min()[2]);
    Vec3 max(bbox.max()[0], bbox.max()[1], bbox.max()[2]);
    Vec3 center = (min + max) / 2;
    Vec3 extents = (max - min);

    std::cout << "BBox min: " << min.ToString() << std::endl;
    std::cout << "BBox max: " << max.ToString() << std::endl;
    std::cout << "BBox center: " << center.ToString() << std::endl;
    std::cout << "BBox extents: " << extents.ToString() << std::endl;
    std::cout << "Value range: " << grid->tree().root().minimum() << " ~ " << grid->tree().root().maximum() << std::endl;

    Transform to_origin(-center);
    Transform tf(0, 0, 0, Quat(-pi / 2, y_axis), Vec3(1));

    Spectrum sigma_a(.01f);
    Spectrum sigma_s(1);
    Float sigma_scale = 1;
    Float g = 0.7;

    NanoVDBMedium* medium =
        scene.CreateMedium<NanoVDBMedium>(tf * to_origin, sigma_a, sigma_s, sigma_scale, g, std::move(handle));
    MediumInterface mi_outside(nullptr, medium);
    MediumInterface mi_inside(medium, nullptr);
    MediumInterface mi_two_sided(medium, medium);

    CreateBox(scene, Transform(Point3::zero, tf.q, extents * tf.s), nullptr, mi_inside);

    // CreateImageInfiniteLight(scene, "res/HDR/small_rural_road_1k.hdr", Transform(Quat(0, y_axis)), 2);
    // CreateImageInfiniteLight(scene, "res/HDR/material-test.hdr", Transform(Quat(pi / 2, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/photo_studio_loft_hall_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/sunset.hdr", Transform(Quat(-pi / 2, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/aerodynamics_workshop_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/scythian_tombs_2_4k.hdr", Transform(Quat(0, y_axis)));
    CreateImageInfiniteLight(scene, "res/HDR/quarry_04_puresky_1k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/photo_studio_01_1k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/peppermint_powerplant_4k.hdr", Transform(Quat(0, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/white_cliff_top_1k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/sunflowers_puresky_1k.hdr", Transform(Quat(pi / 4, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/san_giuseppe_bridge_4k.hdr", Transform(Quat(pi / 2, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/Background_05.hdr", Transform(Quat(pi / 2, y_axis)));
    // CreateUniformInfiniteLight(scene, Spectrum(1));

    Float aspect_ratio = 4.f / 3.f;
    int32 width = 800;
    int32 height = int32(width / aspect_ratio);

    Point3 position = Point3{ 0, 0, 1000 };
    Point3 target = Point3{ 0, 0, 0 };

    Float aperture = 0.0f;
    Float fov = 30;

    ri->integrator_info.type = IntegratorType::vol_path;
    ri->integrator_info.max_bounces = 1024;
    ri->camera_info.type = CameraType::perspective;
    ri->camera_info.transform = Transform::LookAt(position, target, y_axis);
    ri->camera_info.fov = fov;
    ri->camera_info.aperture_radius = aperture;
    ri->camera_info.focus_distance = Dist(position, target);
    ri->camera_info.film_info.filename = "";
    ri->camera_info.film_info.resolution = { width, height };
    ri->camera_info.sampler_info.type = SamplerType::independent;
    ri->camera_info.sampler_info.spp = 8;
}

static int32 sample_index1 = Sample::Register("nvdb", NanoVDBTest);
static int32 sample_index2 = Sample::Register("nvdb2", NanoVDBTest2);
