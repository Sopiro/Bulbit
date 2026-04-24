#include "bulbit/bsdf.h"
#include "bulbit/bxdfs.h"
#include "bulbit/intersectable.h"
#include "bulbit/materials.h"

namespace bulbit
{

MetallicRoughnessMaterial::MetallicRoughnessMaterial(
    const SpectrumTexture* basecolor,
    const FloatTexture* metallic,
    const FloatTexture* u_roughness,
    const FloatTexture* v_roughness,
    const Float3Texture* normal,
    const FloatTexture* alpha

)
    : Material(TypeIndexOf<MetallicRoughnessMaterial>())
    , basecolor{ basecolor }
    , metallic{ metallic }
    , u_roughness{ u_roughness }
    , v_roughness{ v_roughness }
    , normal{ normal }
    , alpha{ alpha }
{
}

bool MetallicRoughnessMaterial::GetBSDF(BSDF* bsdf, const Intersection& isect, WavelengthSample& lambda, Allocator& alloc) const
{
    SpectrumSample b = basecolor->Evaluate(isect.uv, lambda);
    Float m = metallic->Evaluate(isect.uv);
    Float alpha_x = MetallicRoughnessBxDF::RoughnessToAlpha(u_roughness->Evaluate(isect.uv));
    Float alpha_y = MetallicRoughnessBxDF::RoughnessToAlpha(v_roughness->Evaluate(isect.uv));

    *bsdf = BSDF(
        isect.shading.normal, isect.shading.tangent,
        alloc.new_object<MetallicRoughnessBxDF>(b, m, TrowbridgeReitzDistribution(alpha_x, alpha_y))
    );
    return true;
}

bool MetallicRoughnessMaterial::GetBSSRDF(
    BSSRDF** bssrdf, const Intersection& isect, const WavelengthSample& lambda, Allocator& alloc
) const
{
    BulbitNotUsed(bssrdf);
    BulbitNotUsed(isect);
    BulbitNotUsed(lambda);
    BulbitNotUsed(alloc);
    return false;
}

const FloatTexture* MetallicRoughnessMaterial::GetAlphaTexture() const
{
    return alpha;
}

const Float3Texture* MetallicRoughnessMaterial::GetNormalTexture() const
{
    return normal;
}

} // namespace bulbit
