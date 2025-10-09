#pragma once

#include "dynamic_dispatcher.h"
#include "texture.h"

namespace bulbit
{

struct Intersection;
class BSDF;
class BSSRDF;

using Materials = TypePack<
    class AreaLightMaterial,
    class DiffuseMaterial,
    class MirrorMaterial,
    class DielectricMaterial,
    class ThinDielectricMaterial,
    class ClothMaterial,
    class ConductorMaterial,
    class MetallicRoughnessMaterial,
    class PrincipledMaterial,
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
    Spectrum Le(const Intersection& isect, const Vec3& wo) const;
    bool GetBSDF(BSDF* bsdf, const Intersection& isect, const Vec3& wo, Allocator& alloc) const;
    bool GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, const Vec3& wo, Allocator& alloc) const;

    const FloatTexture* GetAlphaTexture() const;
    const SpectrumTexture* GetEmissionTexture() const;
    const SpectrumTexture* GetNormalTexture() const;
};

} // namespace bulbit