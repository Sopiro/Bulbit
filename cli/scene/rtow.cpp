#include "../samples.h"

// The final scene of Ray Tracing in One Weekend
// https://raytracing.github.io/books/RayTracingInOneWeekend.html#wherenext?/afinalrender
void RaytracigInOneWeekend(RendererInfo* ri)
{
    Scene& scene = ri->scene;

    Medium* hm = nullptr;
    // hm = scene.CreateMedium<HomogeneousMedium>(Spectrum(0), Spectrum(0.0f), Spectrum(0.0), 0.5f);
    MediumInterface mi_outside(nullptr, hm);
    MediumInterface mi_inside(hm, nullptr);
    MediumInterface mi_two_sided(hm, hm);

    // CreateSphere(scene, identity, 100, nullptr, mi_inside);

    auto ground_material = CreateDiffuseMaterial(scene, Spectrum(0.5f, 0.5f, 0.5f));

    Transform tf = identity;
    tf.s *= 30;
    CreateRectXZ(scene, tf, ground_material);

    Srand(7777);

    for (int32 a = -11; a < 11; a++)
    {
        for (int32 b = -11; b < 11; b++)
        {
            auto choose_mat = Rand();
            Vec3 center(a + 0.9f * Rand(), 0.2f, b + 0.9f * Rand());

            if (Dist(center, Vec3(4, 0.2f, 0)) > 0.9f)
            {
                if (choose_mat < 0.9f)
                {
                    auto mat = CreateRandomPrincipledMaterial(scene);
                    CreateSphere(
                        scene, Transform(center, Quat(DegToRad(Rand(0, 180)), SampleUniformSphere(RandVec2()))), 0.2f, mat,
                        mi_outside
                    );
                }
                else
                {
                    // glass
                    auto glass = CreateDielectricMaterial(scene, 1.5f, 0.0f);
                    CreateSphere(scene, center, 0.2f, glass, mi_outside);
                }
            }
        }
    }

    auto material1 = CreateDielectricMaterial(scene, 1.5f);
    CreateSphere(scene, Vec3(0, 1, 0), 1.0f, material1, mi_outside);

    auto material2 = CreateDiffuseMaterial(scene, Spectrum(0.4f, 0.2f, 0.1f));
    CreateSphere(scene, Vec3(-4, 1, 0), 1.0f, material2, mi_outside);

    auto material3 = CreateConductorMaterial(scene, { 0.1, 0.2, 1.9 }, { 3, 2.5, 2 }, 0.01f);
    CreateSphere(scene, Transform(Vec3(4, 1, 0), Quat(DegToRad(0), Normalize(Vec3(1, 0, 0)))), 1.0f, material3, mi_outside);

    // CreateDirectionalLight(scene, Normalize(Vec3(1, 1, 1)), 5.0f);
    // CreateImageInfiniteLight(scene, "res/HDR/kloppenheim_07_puresky_1k.hdr");
    // CreateImageInfiniteLight(scene, "res/HDR/quarry_04_puresky_1k.hdr");
    // CreateImageInfiniteLight(scene, "res/HDR/photo_studio_01_1k.hdr");
    // CreateImageInfiniteLight(scene, "res/HDR/pizzo_pernice_1k.hdr");
    CreateImageInfiniteLight(scene, "res/HDR/san_giuseppe_bridge_4k.hdr", Transform(Quat(pi, y_axis)));
    // CreateImageInfiniteLight(scene, "res/HDR/harties_4k.hdr");
    // CreateImageInfiniteLight(scene, "res/HDR/sunflowers_puresky_1k.hdr");

    Float aspect_ratio = 16.f / 9.f;
    int32 width = 1600;
    int32 height = int32(width / aspect_ratio);

    Point3 position{ 13, 2, 3 };
    Point3 target{ 0, 0, 0 };

    ri->integrator_info.type = IntegratorType::path;
    ri->integrator_info.max_bounces = 64;
    ri->camera_info.type = CameraType::perspective;
    ri->camera_info.transform = Transform::LookAt(position, target, y_axis);
    ri->camera_info.fov = 20;
    ri->camera_info.aperture_radius = 0.05f;
    ri->camera_info.focus_distance = 10;
    ri->camera_info.film_info.filename = "";
    ri->camera_info.film_info.resolution = { width, height };
    ri->camera_info.sampler_info.type = SamplerType::stratified;
    ri->camera_info.sampler_info.spp = 64;
}

static int32 sample_index = Sample::Register("rtow", RaytracigInOneWeekend);
