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
    return scene.CreateLight<PointLight>(position, Spectrum{ intensity }, medium);
}

DirectionalLight* CreateDirectionalLight(Scene& scene, const Vec3& direction, const Spectrum& intensity)
{
    return scene.CreateLight<DirectionalLight>(direction, intensity);
}

DirectionalLight* CreateDirectionalLight(Scene& scene, const Vec3& direction, Float intensity)
{
    return scene.CreateLight<DirectionalLight>(direction, Spectrum{ intensity });
}

UniformInfiniteLight* CreateUniformInfiniteLight(Scene& scene, const Spectrum& l, Float scale)
{
    return scene.CreateLight<UniformInfiniteLight>(l, scale);
}

UniformInfiniteLight* CreateUniformInfiniteLight(Scene& scene, Float l, Float scale)
{
    return scene.CreateLight<UniformInfiniteLight>(Spectrum{ l }, scale);
}

ImageInfiniteLight* CreateImageInfiniteLight(Scene& scene, const std::string& filename, const Transform& tf, Float scale)
{
    return scene.CreateLight<ImageInfiniteLight>(CreateSpectrumImageTexture(scene, filename, false), tf, scale);
}

} // namespace bulbit
