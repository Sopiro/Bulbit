#pragma once

#include "bulbit/medium.h"
#include "bulbit/transform.h"

#include <variant>

namespace bulbit
{

class Mesh;
class Material;
class Scene;

enum class AreaLightType
{
    diffuse,
    directional,
    spot,
};

struct AreaLightInfo
{
    AreaLightType type = AreaLightType::diffuse;

    bool two_sided = false;

    // Used for spot light, degrees
    Float angle_max = 40;
    Float angle_falloff_start = 30;

    std::variant<Float, Spectrum, const SpectrumTexture*> emission = 0.0f;
};

void CreateSphere(
    Scene& scene,
    Transform tf,
    Float radius,
    const Material* material,
    const MediumInterface& medium_interface = {},
    std::optional<AreaLightInfo> area_light = {}
);
void CreateTriangles(
    Scene& scene,
    const Mesh* mesh,
    const Material* material,
    const MediumInterface& medium_interface = {},
    std::optional<AreaLightInfo> area_light = {}
);

void CreateRectXY(
    Scene& scene,
    const Transform& transform,
    const Material* material,
    const MediumInterface& medium_interface = {},
    std::optional<AreaLightInfo> area_light = {},
    const Point2& tex_coord = { 1, 1 }
);
void CreateRectXZ(
    Scene& scene,
    const Transform& transform,
    const Material* material,
    const MediumInterface& medium_interface = {},
    std::optional<AreaLightInfo> area_light = {},
    const Point2& tex_coord = { 1, 1 }
);
void CreateRectYZ(
    Scene& scene,
    const Transform& transform,
    const Material* material,
    const MediumInterface& medium_interface = {},
    std::optional<AreaLightInfo> area_light = {},
    const Point2& tex_coord = { 1, 1 }
);
void CreateBox(
    Scene& scene,
    const Transform& transform,
    const Material* material,
    const MediumInterface& medium_interface = {},
    std::optional<AreaLightInfo> area_light = {},
    const Point2& tex_coord = { 1, 1 }
);

} // namespace bulbit