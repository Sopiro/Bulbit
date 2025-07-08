#include "bulbit/bxdfs.h"
#include "bulbit/materials.h"
#include "bulbit/textures.h"

namespace bulbit
{

ClothMaterial::ClothMaterial(Spectrum basecolor, Spectrum sheen_color, Float roughness)
    : Material(TypeIndexOf<ClothMaterial>())
    , basecolor{ basecolor }
    , sheen_color{ sheen_color }
    , roughness{ roughness }
{
}

Float ClothMaterial::GetAlpha(const Intersection& isect) const
{
    BulbitNotUsed(isect);
    return 1;
}

const SpectrumTexture* ClothMaterial::GetNormalMap() const
{
    return nullptr;
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
    Float alpha = std::fmax(TrowbridgeReitzDistribution::RoughnessToAlpha(roughness), 1e-3f);

    *bsdf = BSDF(
        isect.shading.normal, isect.shading.tangent,
        alloc.new_object<SheenBxDF>(basecolor, sheen_color, CharlieSheenDistribution(alpha))
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
