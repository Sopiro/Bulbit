#include "bulbit/bsdf.h"
#include "bulbit/bxdfs.h"
#include "bulbit/intersectable.h"
#include "bulbit/materials.h"

namespace bulbit
{

SubstrateMaterial::SubstrateMaterial(
    const SpectrumTexture* reflectance,
    const FloatTexture* roughness,
    Float ior,
    const Spectrum& sigma_a,
    Float thickness,
    const SpectrumTexture* normal,
    const FloatTexture* alpha
)
    : Material(TypeIndexOf<SubstrateMaterial>())
    , reflectance{ reflectance }
    , roughness{ roughness }
    , ior{ std::max(ior, Float(1.0001f)) }
    , sigma_a{ Max(sigma_a, 0) }
    , thickness{ std::max<Float>(thickness, 0) }
    , normal{ normal }
    , alpha{ alpha }
{
}

bool SubstrateMaterial::GetBSDF(BSDF* bsdf, const Intersection& isect, Allocator& alloc) const
{
    Spectrum rho = reflectance->Evaluate(isect.uv);
    Float rough = roughness ? roughness->Evaluate(isect.uv) : 0.02f;
    Float mf_alpha = SubstrateBxDF::RoughnessToAlpha(rough);

    *bsdf = BSDF(
        isect.shading.normal, isect.shading.tangent,
        alloc.new_object<SubstrateBxDF>(rho, ior, TrowbridgeReitzDistribution(mf_alpha), sigma_a, thickness)
    );

    return true;
}

bool SubstrateMaterial::GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, Allocator& alloc) const
{
    BulbitNotUsed(bssrdf);
    BulbitNotUsed(isect);
    BulbitNotUsed(alloc);
    return false;
}

const FloatTexture* SubstrateMaterial::GetAlphaTexture() const
{
    return alpha;
}

const SpectrumTexture* SubstrateMaterial::GetNormalTexture() const
{
    return normal;
}

} // namespace bulbit
