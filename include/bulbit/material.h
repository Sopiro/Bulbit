#pragma once

#include "bsdf.h"
#include "bssrdf.h"
#include "dynamic_dispatcher.h"
#include "intersectable.h"
#include "texture.h"

namespace bulbit
{

struct Intersection;

using Materials = TypePack<
    class DiffuseMaterial,
    class MirrorMaterial,
    class DielectricMaterial,
    class ThinDielectricMaterial,
    class ClothMaterial,
    class ConductorMaterial,
    class MetallicRoughnessMaterial,
    class PrincipledMaterial,
    class DiffuseLightMaterial,
    class MixtureMaterial,
    class SubsurfaceDiffusionMaterial,
    class SubsurfaceRandomWalkMaterial,
    class LayeredMaterial>;

class Material : public DynamicDispatcher<Materials>
{
public:
    using Types = Materials;

protected:
    Material(int32 type_index)
        : DynamicDispatcher(type_index)
    {
    }

public:
    Float GetAlpha(const Intersection& isect) const;
    const SpectrumTexture* GetNormalMap() const;

    Spectrum Le(const Intersection& isect, const Vec3& wo) const;
    bool GetBSDF(BSDF* bsdf, const Intersection& isect, const Vec3& wo, Allocator& alloc) const;
    bool GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, const Vec3& wo, Allocator& alloc) const;
};

} // namespace bulbit