#pragma once

#include "bulbit/lights.h"
#include "bulbit/scene.h"

namespace bulbit
{

// clang-format off
PointLight* CreatePointLight(
    Scene& scene,
    const Point3& position,
    const Spectrum& intensity,
    const Medium* medium = nullptr
);
PointLight* CreatePointLight(
    Scene& scene,
    const Point3& position,
    Float intensity,
    const Medium* medium = nullptr
);

SpotLight* CreateSpotLight(
    Scene& scene,
    const Point3& position,
    const Vec3& direction,
    const Spectrum& intensity,
    Float angle_max,
    Float angle_falloff_start,
    const Medium* medium = nullptr
);
SpotLight* CreateSpotLight(
    Scene& scene,
    const Point3& position,
    const Vec3& direction,
    Float intensity,
    Float angle_max,
    Float angle_falloff_start,
    const Medium* medium = nullptr
);

DirectionalLight* CreateDirectionalLight(
    Scene& scene,
    const Vec3& direction,
    const Spectrum& intensity
);
DirectionalLight* CreateDirectionalLight(
    Scene& scene,
    const Vec3& direction,
    Float intensity
);

DiffuseAreaLight* CreateDiffuseAreaLight(
    Scene& scene,
    const Primitive* primitive,
    const Spectrum& emission,
    bool two_sided
);

DiffuseAreaLight* CreateDiffuseAreaLight(
    Scene& scene,
    const Primitive* primitive,
    Float emission,
    bool two_sided
);

DirectionalAreaLight* CreateDirectionalAreaLight(
    Scene& scene,
    const Primitive* primitive,
    const Spectrum& emission,
    bool two_sided
);

DirectionalAreaLight* CreateDirectionalAreaLight(
    Scene& scene,
    const Primitive* primitive,
    Float emission,
    bool two_sided
);

UniformInfiniteLight* CreateUniformInfiniteLight(
    Scene& scene,
    const Spectrum& l,
    Float scale = 1
);
UniformInfiniteLight* CreateUniformInfiniteLight(
    Scene& scene,
    Float l,
    Float scale = 1
);

ImageInfiniteLight* CreateImageInfiniteLight(
    Scene& scene,
    const std::string& filename,
    const Transform& tf = identity,
    Float scale = 1
);
// clang-format on

} // namespace bulbit
