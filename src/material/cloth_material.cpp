#include "bulbit/bxdfs.h"
#include "bulbit/materials.h"
#include "bulbit/textures.h"

namespace bulbit
{

ClothMaterial::ClothMaterial(
    const SpectrumTexture* basecolor,
    const SpectrumTexture* sheen_color,
    const FloatTexture* roughness,
    const SpectrumTexture* normalmap,
    const FloatTexture* alpha
)
    : Material(TypeIndexOf<ClothMaterial>())
    , basecolor{ basecolor }
    , sheen_color{ sheen_color }
    , roughness{ roughness }
    , normalmap{ normalmap }
    , alpha{ alpha }
{
}

Float ClothMaterial::GetAlpha(const Intersection& isect) const
{
    if (alpha)
    {
        return alpha->Evaluate(isect.uv);
    }
    else
    {
        return 1;
    }
}

const SpectrumTexture* ClothMaterial::GetNormalMap() const
{
    return normalmap;
}

Spectrum ClothMaterial::Le(const Intersection& isect, const Vec3& wo) const
{
    BulbitNotUsed(isect);
    BulbitNotUsed(wo);
    return Spectrum::black;
}

bool ClothMaterial::GetBSDF(BSDF* bsdf, const Intersection& isect, const Vec3& wo, Allocator& alloc) const
{
    BulbitNotUsed(wo);

    Spectrum base = basecolor->Evaluate(isect.uv);
    Spectrum sheen = sheen_color->Evaluate(isect.uv);
    Float alpha = std::fmax(TrowbridgeReitzDistribution::RoughnessToAlpha(roughness->Evaluate(isect.uv)), 1e-3f);

    *bsdf = BSDF(
        isect.shading.normal, isect.shading.tangent, alloc.new_object<SheenBxDF>(base, sheen, CharlieSheenDistribution(alpha))
    );
    return true;
}

bool ClothMaterial::GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, const Vec3& wo, Allocator& alloc) const
{
    BulbitNotUsed(bssrdf);
    BulbitNotUsed(isect);
    BulbitNotUsed(wo);
    BulbitNotUsed(alloc);
    return false;
}

} // namespace bulbit
