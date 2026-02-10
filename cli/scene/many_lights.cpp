#include "../samples.h"

void ManyLights(RendererInfo* ri)
{
    Scene& scene = ri->scene;

    Float r = 0.3f;
    int32 cx = 10;
    int32 cz = 7;
    Float xgap = 0.16f;
    Float zgap = 0.14f;
    Float xstep = 2.0f * r + xgap;
    Float zstep = 2.0f * r + zgap;

    for (int32 z = 0; z < cz; ++z)
    {
        for (int32 x = 0; x < cx; ++x)
        {
            Vec3 pos;
            pos.y = 0.0f;
            pos.x = x * xstep - ((cx - 1) * xstep * 0.5f);
            pos.z = z * zstep - ((cz - 1) * zstep * 0.5f);

            Quat rot(DegToRad(Rand(0.0f, 180.0f)), SampleUniformSphere(RandVec2()));
            auto mat = CreateRandomPrincipledMaterial(scene);
            CreateSphere(scene, Transform{ pos, rot }, r, mat);
        }
    }

    {
        auto checker = CreateSpectrumCheckerTexture(scene, 0.75, 0.3, Point2(100));
        auto mat = scene.CreateMaterial<DiffuseMaterial>(checker);
        auto tf = Transform{ Vec3(0.5f, -r, -0.5f), identity, Vec3(50.0f) };
        CreateRectXZ(scene, tf, mat);
    }

    {
        Point3 center(0.0f, 2.2f, 0.0f);
        Float x_half = 4.5f;
        Float z_half = 1.4f;
        int32 x_count = 50;
        int32 z_count = 30;
        Vec2 light_size(0.05f, 0.05f);

        Float min_x = center.x - x_half;
        Float max_x = center.x + x_half;
        Float min_z = center.z - z_half;
        Float max_z = center.z + z_half;
        auto light_mat = CreateDiffuseMaterial(scene, Spectrum(1.0f));

        for (int32 z = 0; z < z_count; ++z)
        {
            Float tz = (z + 0.5f) / Float(z_count);
            Float pz = Lerp(min_z, max_z, tz);

            for (int32 x = 0; x < x_count; ++x)
            {
                Float tx = (x + 0.5f) / Float(x_count);
                Float px = Lerp(min_x, max_x, tx);

                Vec3 pos;
                pos.y = center.y;
                pos.x = px;
                pos.z = pz;

                auto tf = Transform{ pos, Quat(pi, x_axis), Vec3(light_size.x, light_size.x, light_size.y) };
                CreateRectXZ(scene, tf, light_mat, {}, AreaLightInfo{ .emission = 10.0f });
            }
        }
    }

    Float aspect_ratio = 16.0f / 9.0f;
    int32 width = 1600;
    int32 height = int32(width / aspect_ratio);

    Point3 position{ 0.0f, 4.0f, 5.0f };
    Point3 target{ 0.0f, 0.0f, 0.0f };

    ri->integrator_info.type = IntegratorType::restir_di;
    ri->integrator_info.max_bounces = 1;
    ri->integrator_info.sample_direct_light = true;

    ri->camera_info.type = CameraType::perspective;
    ri->camera_info.transform = Transform::LookAt(position, target, y_axis);
    ri->camera_info.fov = 71.0f;
    ri->camera_info.aperture_radius = 0.0f;
    ri->camera_info.focus_distance = Dist(position, target);
    ri->camera_info.film_info.filename = "";
    ri->camera_info.film_info.resolution = { width, height };
    ri->camera_info.sampler_info.type = SamplerType::independent;
    ri->camera_info.sampler_info.spp = 1;
}

static int32 sample_index = Sample::Register("many-lights", ManyLights);
