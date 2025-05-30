#include "../samples.h"

std::unique_ptr<Camera> StanfordScene(Scene& scene)
{
    // Floor
    {
        auto mat = scene.CreateMaterial<MetallicRoughnessMaterial>(
            CreateSpectrumImageTexture(scene, "res/dark_wooden_planks_4k/textures/dark_wooden_planks_diff_4k.jpg"),
            CreateFloatImageTexture(scene, "res/dark_wooden_planks_4k/textures/dark_wooden_planks_arm_4k.jpg", 2, true),
            CreateFloatImageTexture(scene, "res/dark_wooden_planks_4k/textures/dark_wooden_planks_arm_4k.jpg", 1, true),
            CreateFloatImageTexture(scene, "res/dark_wooden_planks_4k/textures/dark_wooden_planks_arm_4k.jpg", 1, true),
            CreateSpectrumConstantTexture(scene, Spectrum(0.0f)),
            CreateSpectrumImageTexture(scene, "res/dark_wooden_planks_4k/textures/dark_wooden_planks_nor_gl_4k.png", true)
        );

        auto tf = Transform{ Vec3::zero, identity, Vec3(8.0f) };
        CreateRectXZ(scene, tf, mat, {}, Point2(4.0f, 4.0f));
    }

    Float scale = 0.3f;
    Float gap = 0.2f;

    // Bunny
    {
        auto tf = Transform{ Vec3(gap * 3.0f, 0.0f, 0.0f), Quat(0.0f, y_axis), Vec3(scale) };
        auto mat = CreateRandomPrincipledMaterial(scene);

        SetLoaderFallbackMaterial(mat);
        LoadModel(scene, "res/stanford/bunny.obj", tf);
    }

    // Lucy
    {
        auto tf = Transform{ Vec3(gap, 0.0f, 0.0f), Quat(0.0f, y_axis), Vec3(scale) };
        auto mat = CreateRandomPrincipledMaterial(scene);

        SetLoaderFallbackMaterial(mat);
        LoadModel(scene, "res/stanford/lucy.obj", tf);
    }

    // Tyrannosaurus
    {
        auto tf = Transform{ Vec3(-gap, 0.0f, 0.0f), Quat(DegToRad(45.0f), y_axis), Vec3(scale) };
        auto mat = CreateRandomPrincipledMaterial(scene);

        SetLoaderFallbackMaterial(mat);
        SetLoaderFlipNormal(true);
        LoadModel(scene, "res/stanford/tyra.obj", tf);
        SetLoaderFlipNormal(false);
    }

    // Armadillo
    {
        auto tf = Transform{ Vec3(-gap * 3.0f, 0.0f, 0.0f), Quat(0.0f, y_axis), Vec3(scale) };
        auto mat = CreateMetallicRoughnessMaterial(
            scene, (Spectrum(Rand(0.0f, 1.0f), Rand(0.0f, 1.0f), Rand(0.0f, 1.0f)) * 0.7f), (1.0f), (0.2f)
        );

        SetLoaderFallbackMaterial(mat);
        LoadModel(scene, "res/stanford/arma.obj", tf);
    }

    {
        Float w = 0.04f;
        Float h = 0.6f;
        Float cx = 16.0f;
        Float xgap = 0.015f;
        Float xstep = 2.0f * w + xgap;

        auto light = CreateDiffuseLightMaterial(scene, Spectrum(3.0f), true);

        for (int32 x = 0; x < cx; ++x)
        {
            Vec3 pos;

            pos.y = 0.6f;
            pos.x = x * xstep - ((cx - 1) * xstep / 2.0f);
            pos.z = 0.0f;

            auto mat = CreateRandomPrincipledMaterial(scene);

            auto tf = Transform{ pos, Quat(pi, x_axis), Vec3(w, w, h) };
            CreateRectXZ(scene, tf, light);
        }
    }

    // CreateImageInfiniteLight(scene, "res/HDR/sunflowers_puresky_1k.hdr"));

    Float aspect_ratio = 16.f / 9.f;
    // Float aspect_ratio = 3.f / 2.f;
    // Float aspect_ratio = 4.f / 3.f;
    // Float aspect_ratio = 1.f;
    int32 width = 960;
    int32 height = int32(width / aspect_ratio);

    Point3 lookfrom{ 0, 0.5f, 2 };
    Point3 lookat{ 0, 0.2f, 0 };

    Float dist_to_focus = Dist(lookfrom, lookat);
    Float aperture = 0;
    Float vFov = 30;

    return std::make_unique<PerspectiveCamera>(lookfrom, lookat, y_axis, vFov, aperture, dist_to_focus, Point2i(width, height));
}

static int32 sample_index = Sample::Register("stanford", StanfordScene);
