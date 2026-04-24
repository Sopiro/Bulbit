#include "light_builder.h"
#include "texture_builder.h"

namespace bulbit
{

PointLight* CreatePointLight(Scene& scene, const Point3& position, const Spectrum& intensity, const Medium* medium)
{
    return scene.CreateLight<PointLight>(position, intensity, medium);
}

PointLight* CreatePointLight(Scene& scene, const Point3& position, Float intensity, const Medium* medium)
{
    return scene.CreateLight<PointLight>(position, Spectrum::FromIlluminantRGB(Vec3(intensity)), medium);
}

SpotLight* CreateSpotLight(
    Scene& scene,
    const Point3& position,
    const Vec3& direction,
    const Spectrum& intensity,
    Float angle_max,
    Float angle_falloff_start,
    const Medium* medium
)
{
    return scene.CreateLight<SpotLight>(position, direction, intensity, angle_max, angle_falloff_start, medium);
}

SpotLight* CreateSpotLight(
    Scene& scene,
    const Point3& position,
    const Vec3& direction,
    Float intensity,
    Float angle_max,
    Float angle_falloff_start,
    const Medium* medium
)
{
    return scene.CreateLight<SpotLight>(
        position, direction, Spectrum::FromIlluminantRGB(Vec3(intensity)), angle_max, angle_falloff_start, medium
    );
}

DirectionalLight* CreateDirectionalLight(Scene& scene, const Vec3& direction, const Spectrum& intensity)
{
    return scene.CreateLight<DirectionalLight>(direction, intensity);
}

DirectionalLight* CreateDirectionalLight(Scene& scene, const Vec3& direction, Float intensity)
{
    return scene.CreateLight<DirectionalLight>(direction, Spectrum::FromIlluminantRGB(Vec3(intensity)));
}

DiffuseAreaLight* CreateDiffuseAreaLight(Scene& scene, const Primitive* primitive, const Spectrum& emission, bool two_sided)
{
    SpectrumConstantTexture* texture = CreateSpectrumConstantTexture(scene, emission);
    return scene.CreateLight<DiffuseAreaLight>(primitive, texture, texture->MeanLuminance(), two_sided);
}

DiffuseAreaLight* CreateDiffuseAreaLight(Scene& scene, const Primitive* primitive, Float emission, bool two_sided)
{
    SpectrumConstantTexture* texture = CreateSpectrumConstantTexture(scene, Spectrum::FromIlluminantRGB(Vec3(emission)));
    return scene.CreateLight<DiffuseAreaLight>(primitive, texture, texture->MeanLuminance(), two_sided);
}

SpotAreaLight* CreateSpotAreaLight(
    Scene& scene, const Primitive* primitive, const Spectrum& emission, Float angle_max, Float angle_falloff_start, bool two_sided
)
{
    SpectrumConstantTexture* texture = CreateSpectrumConstantTexture(scene, emission);
    return scene.CreateLight<SpotAreaLight>(
        primitive, texture, texture->MeanLuminance(), angle_max, angle_falloff_start, two_sided
    );
}

SpotAreaLight* CreateSpotAreaLight(
    Scene& scene, const Primitive* primitive, Float emission, Float angle_max, Float angle_falloff_start, bool two_sided
)
{
    SpectrumConstantTexture* texture = CreateSpectrumConstantTexture(scene, Spectrum::FromIlluminantRGB(Vec3(emission)));
    return scene.CreateLight<SpotAreaLight>(
        primitive, texture, texture->MeanLuminance(), angle_max, angle_falloff_start, two_sided
    );
}

DirectionalAreaLight* CreateDirectionalAreaLight(
    Scene& scene, const Primitive* primitive, const Spectrum& emission, bool two_sided
)
{
    SpectrumConstantTexture* texture = CreateSpectrumConstantTexture(scene, emission);
    return scene.CreateLight<DirectionalAreaLight>(primitive, texture, texture->MeanLuminance(), two_sided);
}

DirectionalAreaLight* CreateDirectionalAreaLight(Scene& scene, const Primitive* primitive, Float emission, bool two_sided)
{
    SpectrumConstantTexture* texture = CreateSpectrumConstantTexture(scene, Spectrum::FromIlluminantRGB(Vec3(emission)));
    return scene.CreateLight<DirectionalAreaLight>(primitive, texture, texture->MeanLuminance(), two_sided);
}

UniformInfiniteLight* CreateUniformInfiniteLight(Scene& scene, const Spectrum& l, Float scale)
{
    return scene.CreateLight<UniformInfiniteLight>(l, scale);
}

UniformInfiniteLight* CreateUniformInfiniteLight(Scene& scene, Float l, Float scale)
{
    return scene.CreateLight<UniformInfiniteLight>(Spectrum::FromIlluminantRGB(Vec3(l)), scale);
}

ImageInfiniteLight* CreateImageInfiniteLight(Scene& scene, const std::string& filename, const Transform& tf, Float scale)
{
    return scene.CreateLight<ImageInfiniteLight>(CreateSpectrumIlluminantImageTexture(scene, filename, false), tf, scale);
}

} // namespace bulbit
