#include "../samples.h"

void CornellBoxCloud(RendererInfo* ri)
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
    Transform t(.5, .5, -.5, Quat(-pi / 2, y_axis), Vec3(0.001) * 1.3f);

    Spectrum sigma_a(.01f);
    Spectrum sigma_s(1);
    Float sigma_scale = 100;
    Float g = 0.5;

    NanoVDBMedium* medium = scene.CreateMedium<NanoVDBMedium>(t * to_origin, sigma_a, sigma_s, sigma_scale, g, std::move(handle));
    MediumInterface mi_outside(nullptr, medium);
    MediumInterface mi_inside(medium, nullptr);
    MediumInterface mi_two_sided(medium, medium);

    // CreateBox(scene, Transform(Point3::zero, tf.q, extents * tf.s), nullptr, mi_inside);

    // Cornell box
    {
        auto red = CreateDiffuseMaterial(scene, Spectrum(.65f, .05f, .05f));
        auto green = CreateDiffuseMaterial(scene, Spectrum(.12f, .45f, .15f));
        auto white = CreateDiffuseMaterial(scene, Spectrum(.73f, .73f, .73f));
        auto light = CreateDiffuseLightMaterial(scene, Spectrum(15.0f));

        // front
        auto tf = Transform{ Vec3(0.5f, 0.5f, -1.0f), identity, Vec3(1.0f) };
        CreateRectXY(scene, tf, white, mi_outside);

        // left
        tf = Transform{ Vec3(0.0f, 0.5f, -0.5f), identity, Vec3(1.0f) };
        CreateRectYZ(scene, tf, red, mi_outside);

        // right
        tf = Transform{ Vec3(1.0f, 0.5f, -0.5f), Quat(pi, y_axis), Vec3(1.0f) };
        CreateRectYZ(scene, tf, green, mi_outside);

        // bottom
        tf = Transform{ Vec3(0.5f, 0.0f, -0.5f), identity, Vec3(1.0f) };
        CreateRectXZ(scene, tf, white, mi_outside);

        // top
        tf = Transform{ Vec3(0.5f, 1.0f, -0.5f), Quat(pi, x_axis), Vec3(1.0f) };
        CreateRectXZ(scene, tf, white, mi_outside);

        // back
        tf = Transform{ Vec3(0.5f, 0.5f, 0.0f), Quat(pi, y_axis), Vec3(1.0f) };
        CreateRectXY(scene, tf, nullptr, mi_outside);

        tf = Transform{ 0.5f, 0.995f, -0.5f, Quat(pi, x_axis), Vec3(0.25f) };
        CreateRectXZ(scene, tf, light, mi_two_sided);
    }

    Float aspect_ratio = 1;
    int32 width = 1000;
    int32 height = int32(width / aspect_ratio);

    Point3 position{ 0.5f, 0.5f, 2.05f };
    Point3 target{ 0.5f, 0.5f, 0.0f };

    Float aperture = 0.0f;
    Float fov = 28;

    ri->integrator_info.type = IntegratorType::vol_bdpt;
    ri->integrator_info.max_bounces = 1024;
    ri->camera_info.type = CameraType::perspective;
    ri->camera_info.transform = Transform::LookAt(position, target, y_axis);
    ri->camera_info.fov = fov;
    ri->camera_info.aperture_radius = aperture;
    ri->camera_info.focus_distance = Dist(position, target);
    ri->camera_info.film_info.filename = "";
    ri->camera_info.film_info.resolution = { width, height };
    ri->camera_info.sampler_info.type = SamplerType::independent;
    ri->camera_info.sampler_info.spp = 64;
}

static int32 sample_index = Sample::Register("cornell-box-cloud", CornellBoxCloud);
