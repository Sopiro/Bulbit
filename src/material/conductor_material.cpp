#include "bulbit/bsdf.h"
#include "bulbit/bxdfs.h"
#include "bulbit/intersectable.h"
#include "bulbit/materials.h"

namespace bulbit
{

ConductorMaterial::ConductorMaterial(
    const SpectrumTexture* eta,
    const SpectrumTexture* k,
    const FloatTexture* u_roughness,
    const FloatTexture* v_roughness,
    const SpectrumTexture* reflectance,
    bool energy_compensation,
    const Float3Texture* normal,
    const FloatTexture* alpha
)
    : Material(TypeIndexOf<ConductorMaterial>())
    , eta{ eta }
    , k{ k }
    , u_roughness{ u_roughness }
    , v_roughness{ v_roughness }
    , reflectance{ reflectance }
    , energy_compensation{ energy_compensation }
    , normal{ normal }
    , alpha{ alpha }
{
}

ConductorMaterial::ConductorMaterial(
    const SpectrumTexture* R,
    const FloatTexture* u_roughness,
    const FloatTexture* v_roughness,
    const SpectrumTexture* reflectance,
    bool energy_compensation,
    const Float3Texture* normal,
    const FloatTexture* alpha
)
    : Material(TypeIndexOf<ConductorMaterial>())
    , eta{ nullptr }
    , k{ R }
    , u_roughness{ u_roughness }
    , v_roughness{ v_roughness }
    , reflectance{ reflectance }
    , energy_compensation{ energy_compensation }
    , normal{ normal }
    , alpha{ alpha }
{
}

bool ConductorMaterial::GetBSDF(BSDF* bsdf, const Intersection& isect, WavelengthSample& lambda, Allocator& alloc) const
{
    Float alpha_x = TrowbridgeReitzDistribution::RoughnessToAlpha(u_roughness->Evaluate(isect.uv));
    Float alpha_y = TrowbridgeReitzDistribution::RoughnessToAlpha(v_roughness->Evaluate(isect.uv));
    SpectrumSample r = reflectance ? reflectance->Evaluate(isect.uv, lambda) : SpectrumSample(1);

    if (eta)
    {
        SpectrumSample k_s = k->Evaluate(isect.uv, lambda);
        SpectrumSample eta_s = eta->Evaluate(isect.uv, lambda);

        if (energy_compensation)
        {
            *bsdf = BSDF(
                isect.shading.normal, isect.shading.tangent,
                alloc.new_object<ConductorMultiScatteringBxDF>(eta_s, k_s, TrowbridgeReitzDistribution(alpha_x, alpha_y), r)
            );
        }
        else
        {
            *bsdf = BSDF(
                isect.shading.normal, isect.shading.tangent,
                alloc.new_object<ConductorBxDF>(eta_s, k_s, TrowbridgeReitzDistribution(alpha_x, alpha_y), r)
            );
        }
    }
    else
    {
        SpectrumSample R = k->Evaluate(isect.uv, lambda);

        if (energy_compensation)
        {
            *bsdf = BSDF(
                isect.shading.normal, isect.shading.tangent,
                alloc.new_object<ConductorMultiScatteringBxDF>(R, TrowbridgeReitzDistribution(alpha_x, alpha_y), r)
            );
        }
        else
        {
            *bsdf = BSDF(
                isect.shading.normal, isect.shading.tangent,
                alloc.new_object<ConductorBxDF>(R, TrowbridgeReitzDistribution(alpha_x, alpha_y), r)
            );
        }
    }

    return true;
}

bool ConductorMaterial::GetBSSRDF(
    BSSRDF** bssrdf, const Intersection& isect, const WavelengthSample& lambda, Allocator& alloc
) const
{
    BulbitNotUsed(bssrdf);
    BulbitNotUsed(isect);
    BulbitNotUsed(lambda);
    BulbitNotUsed(alloc);
    return false;
}

const FloatTexture* ConductorMaterial::GetAlphaTexture() const
{
    return alpha;
}

const Float3Texture* ConductorMaterial::GetNormalTexture() const
{
    return normal;
}

} // namespace bulbit
