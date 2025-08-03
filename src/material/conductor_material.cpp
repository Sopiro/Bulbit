#include "bulbit/bxdfs.h"
#include "bulbit/materials.h"

namespace bulbit
{

ConductorMaterial::ConductorMaterial(
    const SpectrumTexture* eta,
    const SpectrumTexture* k,
    const FloatTexture* u_roughness,
    const FloatTexture* v_roughness,
    bool energy_compensation,
    const SpectrumTexture* normal,
    const FloatTexture* alpha
)
    : Material(TypeIndexOf<ConductorMaterial>())
    , eta{ eta }
    , k{ k }
    , u_roughness{ u_roughness }
    , v_roughness{ v_roughness }
    , energy_compensation{ energy_compensation }
    , normal{ normal }
    , alpha{ alpha }
{
}

ConductorMaterial::ConductorMaterial(
    const SpectrumTexture* reflectance,
    const FloatTexture* u_roughness,
    const FloatTexture* v_roughness,
    bool energy_compensation,
    const SpectrumTexture* normal,
    const FloatTexture* alpha
)
    : Material(TypeIndexOf<ConductorMaterial>())
    , eta{ nullptr }
    , k{ reflectance }
    , u_roughness{ u_roughness }
    , v_roughness{ v_roughness }
    , energy_compensation{ energy_compensation }
    , normal{ normal }
    , alpha{ alpha }
{
}

Spectrum ConductorMaterial::Le(const Intersection& isect, const Vec3& wo) const
{
    BulbitNotUsed(isect);
    BulbitNotUsed(wo);
    return Spectrum::black;
}

bool ConductorMaterial::GetBSDF(BSDF* bsdf, const Intersection& isect, const Vec3& wo, Allocator& alloc) const
{
    BulbitNotUsed(wo);

    Float alpha_x = u_roughness->Evaluate(isect.uv);
    Float alpha_y = v_roughness->Evaluate(isect.uv);

    if (eta)
    {
        Spectrum k_s = k->Evaluate(isect.uv);
        Spectrum eta_s = eta->Evaluate(isect.uv);

        if (energy_compensation)
        {
            *bsdf = BSDF(
                isect.shading.normal, isect.shading.tangent,
                alloc.new_object<ConductorMultiScatteringBxDF>(eta_s, k_s, TrowbridgeReitzDistribution(alpha_x, alpha_y))
            );
        }
        else
        {
            *bsdf = BSDF(
                isect.shading.normal, isect.shading.tangent,
                alloc.new_object<ConductorBxDF>(eta_s, k_s, TrowbridgeReitzDistribution(alpha_x, alpha_y))
            );
        }
    }
    else
    {
        Spectrum r = k->Evaluate(isect.uv);

        if (energy_compensation)
        {
            *bsdf = BSDF(
                isect.shading.normal, isect.shading.tangent,
                alloc.new_object<ConductorMultiScatteringBxDF>(r, TrowbridgeReitzDistribution(alpha_x, alpha_y))
            );
        }
        else
        {
            *bsdf = BSDF(
                isect.shading.normal, isect.shading.tangent,
                alloc.new_object<ConductorBxDF>(r, TrowbridgeReitzDistribution(alpha_x, alpha_y))
            );
        }
    }

    return true;
}

bool ConductorMaterial::GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, const Vec3& wo, Allocator& alloc) const
{
    BulbitNotUsed(bssrdf);
    BulbitNotUsed(isect);
    BulbitNotUsed(wo);
    BulbitNotUsed(alloc);
    return false;
}

const FloatTexture* ConductorMaterial::GetAlphaTexture() const
{
    return alpha;
}

const SpectrumTexture* ConductorMaterial::GetEmissionTexture() const
{
    return nullptr;
}

const SpectrumTexture* ConductorMaterial::GetNormalTexture() const
{
    return normal;
}

} // namespace bulbit
