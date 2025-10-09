#include "bulbit/bsdf.h"
#include "bulbit/bxdfs.h"
#include "bulbit/intersectable.h"
#include "bulbit/materials.h"
#include "bulbit/textures.h"

namespace bulbit
{

DielectricMaterial::DielectricMaterial(
    Float eta,
    const FloatTexture* u_roughness,
    const FloatTexture* v_roughness,
    const SpectrumTexture* reflectance,
    bool energy_compensation,
    const SpectrumTexture* normal
)
    : Material(TypeIndexOf<DielectricMaterial>())
    , eta{ eta }
    , u_roughness{ u_roughness }
    , v_roughness{ v_roughness }
    , reflectance{ reflectance }
    , energy_compensation{ energy_compensation }
    , normal{ normal }
{
}

Spectrum DielectricMaterial::Le(const Intersection& isect, const Vec3& wo) const
{
    BulbitNotUsed(isect);
    BulbitNotUsed(wo);
    return Spectrum::black;
}

bool DielectricMaterial::GetBSDF(BSDF* bsdf, const Intersection& isect, Allocator& alloc) const
{
    Float eta_p = isect.front_face ? eta : 1 / eta;

    Spectrum r = reflectance->Evaluate(isect.uv);
    Float alpha_x = TrowbridgeReitzDistribution::RoughnessToAlpha(u_roughness->Evaluate(isect.uv));
    Float alpha_y = TrowbridgeReitzDistribution::RoughnessToAlpha(v_roughness->Evaluate(isect.uv));

    if (energy_compensation)
    {
        *bsdf = BSDF(
            isect.shading.normal, isect.shading.tangent,
            alloc.new_object<DielectricMultiScatteringBxDF>(eta_p, TrowbridgeReitzDistribution(alpha_x, alpha_y), r)
        );
    }
    else
    {
        *bsdf = BSDF(
            isect.shading.normal, isect.shading.tangent,
            alloc.new_object<DielectricBxDF>(eta_p, TrowbridgeReitzDistribution(alpha_x, alpha_y), r)
        );
    }

    return true;
}

bool DielectricMaterial::GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, Allocator& alloc) const
{
    BulbitNotUsed(bssrdf);
    BulbitNotUsed(isect);
    BulbitNotUsed(alloc);
    return false;
}

const FloatTexture* DielectricMaterial::GetAlphaTexture() const
{
    return nullptr;
}

const SpectrumTexture* DielectricMaterial::GetEmissionTexture() const
{
    return nullptr;
}

const SpectrumTexture* DielectricMaterial::GetNormalTexture() const
{
    return normal;
}

} // namespace bulbit
