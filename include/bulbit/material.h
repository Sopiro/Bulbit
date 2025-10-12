#pragma once

#include "dynamic_dispatcher.h"
#include "texture.h"

namespace bulbit
{

struct Intersection;
class BSDF;
class BSSRDF;

using Materials = TypePack<
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

    bool GetBSDF(BSDF* bsdf, const Intersection& isect, Allocator& alloc) const;
    bool GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, Allocator& alloc) const;

    const FloatTexture* GetAlphaTexture() const;
    const SpectrumTexture* GetNormalTexture() const;

protected:
    Material(int32 type_index)
        : DynamicDispatcher(type_index)
    {
    }
};

} // namespace bulbit