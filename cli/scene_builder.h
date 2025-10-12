#pragma once

#include "bulbit/medium.h"
#include "bulbit/transform.h"

#include <variant>

namespace bulbit
{

class Mesh;
class Material;
class Scene;

struct AreaLightInfo
{
    bool is_directional = false;
    bool two_sided = false;
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