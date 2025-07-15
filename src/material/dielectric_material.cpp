#include "bulbit/bxdfs.h"
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
    const SpectrumTexture* normalmap
)
    : Material(TypeIndexOf<DielectricMaterial>())
    , eta{ eta }
    , u_roughness{ u_roughness }
    , v_roughness{ v_roughness }
    , reflectance{ reflectance }
    , energy_compensation{ energy_compensation }
    , normalmap{ normalmap }
{
}

Float DielectricMaterial::GetAlpha(const Intersection& isect) const
{
    BulbitNotUsed(isect);
    return 1;
}

const SpectrumTexture* DielectricMaterial::GetNormalMap() const
{
    return normalmap;
}

Spectrum DielectricMaterial::Le(const Intersection& isect, const Vec3& wo) const
{
    BulbitNotUsed(isect);
    BulbitNotUsed(wo);
    return Spectrum::black;
}

bool DielectricMaterial::GetBSDF(BSDF* bsdf, const Intersection& isect, const Vec3& wo, Allocator& alloc) const
{
    BulbitNotUsed(wo);

    Float eta_p = isect.front_face ? eta : 1 / eta;

    Spectrum r = reflectance->Evaluate(isect.uv);
    Float alpha_x = u_roughness->Evaluate(isect.uv);
    Float alpha_y = v_roughness->Evaluate(isect.uv);

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

bool DielectricMaterial::GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, const Vec3& wo, Allocator& alloc) const
{
    BulbitNotUsed(bssrdf);
    BulbitNotUsed(isect);
    BulbitNotUsed(wo);
    BulbitNotUsed(alloc);
    return false;
}

} // namespace bulbit
