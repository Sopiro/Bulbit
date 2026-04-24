#include "bulbit/bsdf.h"
#include "bulbit/bxdfs.h"
#include "bulbit/intersectable.h"
#include "bulbit/materials.h"
#include "bulbit/textures.h"

namespace bulbit
{

ClothMaterial::ClothMaterial(
    const SpectrumTexture* basecolor,
    const SpectrumTexture* sheen_color,
    const FloatTexture* roughness,
    const Float3Texture* normal,
    const FloatTexture* alpha
)
    : Material(TypeIndexOf<ClothMaterial>())
    , basecolor{ basecolor }
    , sheen_color{ sheen_color }
    , roughness{ roughness }
    , normal{ normal }
    , alpha{ alpha }
{
}

bool ClothMaterial::GetBSDF(BSDF* bsdf, const Intersection& isect, WavelengthSample& lambda, Allocator& alloc) const
{
    SpectrumSample base = basecolor->Evaluate(isect.uv, lambda);
    SpectrumSample sheen = sheen_color->Evaluate(isect.uv, lambda);
    Float alpha = std::fmax(TrowbridgeReitzDistribution::RoughnessToAlpha(roughness->Evaluate(isect.uv)), 1e-3f);

    *bsdf = BSDF(
        isect.shading.normal, isect.shading.tangent, alloc.new_object<SheenBxDF>(base, sheen, CharlieSheenDistribution(alpha))
    );
    return true;
}

bool ClothMaterial::GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, const WavelengthSample& lambda, Allocator& alloc) const
{
    BulbitNotUsed(bssrdf);
    BulbitNotUsed(isect);
    BulbitNotUsed(lambda);
    BulbitNotUsed(alloc);
    return false;
}

const FloatTexture* ClothMaterial::GetAlphaTexture() const
{
    return alpha;
}

const Float3Texture* ClothMaterial::GetNormalTexture() const
{
    return normal;
}

} // namespace bulbit
