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
    const Float3Texture* normal,
    const FloatTexture* alpha
)
    : Material(TypeIndexOf<SubstrateMaterial>())
    , reflectance{ reflectance }
    , roughness{ roughness }
    , ior{ std::max(ior, Float(1.0001f)) }
    , sigma_a{ sigma_a }
    , thickness{ std::max<Float>(thickness, 0) }
    , normal{ normal }
    , alpha{ alpha }
{
}

bool SubstrateMaterial::GetBSDF(BSDF* bsdf, const Intersection& isect, WavelengthSample& lambda, Allocator& alloc) const
{
    SpectrumSample rho = reflectance->Evaluate(isect.uv, lambda);
    Float rough = roughness ? roughness->Evaluate(isect.uv) : 0.02f;
    Float mf_alpha = SubstrateBxDF::RoughnessToAlpha(rough);
    SpectrumSample sigma_a_sample = Max(sigma_a.Sample(lambda), 0.0f);

    *bsdf = BSDF(
        isect.shading.normal, isect.shading.tangent,
        alloc.new_object<SubstrateBxDF>(rho, ior, TrowbridgeReitzDistribution(mf_alpha), sigma_a_sample, thickness)
    );

    return true;
}

bool SubstrateMaterial::GetBSSRDF(
    BSSRDF** bssrdf, const Intersection& isect, const WavelengthSample& lambda, Allocator& alloc
) const
{
    BulbitNotUsed(bssrdf);
    BulbitNotUsed(isect);
    BulbitNotUsed(lambda);
    BulbitNotUsed(alloc);
    return false;
}

const FloatTexture* SubstrateMaterial::GetAlphaTexture() const
{
    return alpha;
}

const Float3Texture* SubstrateMaterial::GetNormalTexture() const
{
    return normal;
}

} // namespace bulbit
