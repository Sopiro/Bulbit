#include "../samples.h"

std::unique_ptr<Camera> CameraScene(Scene& scene)
{
    // Floor
    {
        auto mat = CreateMetallicRoughnessMaterial(scene, Spectrum(0.5f), 0.0f, 0.01f);

        auto tf = Transform{ Vec3::zero, identity, Vec3(8.0f) };
        CreateRectXZ(scene, tf, mat);
    }

    // Camera
    {
        auto tf = Transform{ Vec3::zero, Quat(DegToRad(0.0f), y_axis), Vec3(0.11f) };
        LoadModel(scene, "res/AntiqueCamera/glTF/AntiqueCamera.gltf", tf);
    }

    // Lights
    {
        auto light = CreateDiffuseLightMaterial(scene, Spectrum(1.0f, 0.9f, 0.8f) * 3);
        Float w = 0.4f;
        Float h = 1.2f;
        auto tf = Transform{ Point3(1.0f, h / 2.0f - 0.01f, 0.0f), Quat(pi, y_axis), Vec3(1.0f, h, w) };
        CreateRectYZ(scene, tf, light);

        tf = Transform{ Point3(0.0f, h / 2.0f - 0.01f, -1.0f), Quat(0.0f, y_axis), Vec3(w, h, 1.0f) };
        CreateRectXY(scene, tf, light);

        tf = Transform{ Point3(0.0f, h / 2.0f - 0.01f, 1.0f), Quat(pi, y_axis), Vec3(w, h, 1.0f) };
        CreateRectXY(scene, tf, light);

        tf = Transform{ Point3(-1.0f, h / 2.0f - 0.01f, 0.0f), Quat(0.0f, y_axis), Vec3(1.0f, h, w) };
        CreateRectYZ(scene, tf, light);
    }

    // scene.SetEnvironmentMap(ImageTexture::Create("res/HDR/sunflowers_puresky_1k.hdr", false, true));

    Float aspect_ratio = 16.f / 9.f;
    // Float aspect_ratio = 3.f / 2.f;
    // Float aspect_ratio = 4.f / 3.f;
    // Float aspect_ratio = 1.f;
    int32 width = 500;
    int32 height = int32(width / aspect_ratio);

    Point3 lookfrom{ -2, 1, 2 };
    Point3 lookat{ 0, 0.5f, 0 };

    Float dist_to_focus = Dist(lookfrom, lookat);
    Float aperture = 0;
    Float vFov = 30;

    return std::make_unique<PerspectiveCamera>(lookfrom, lookat, y_axis, vFov, aperture, dist_to_focus, Point2i(width, height));
}

static int32 sample_index = Sample::Register("camera", CameraScene);
