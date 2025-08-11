#include "bulbit/bsdf.h"
#include "bulbit/bssrdfs.h"
#include "bulbit/bxdfs.h"
#include "bulbit/materials.h"
#include "bulbit/textures.h"

namespace bulbit
{

SubsurfaceDiffusionMaterial::SubsurfaceDiffusionMaterial(
    const SpectrumTexture* reflectance,
    const Spectrum& mfp,
    Float eta,
    const FloatTexture* u_roughness,
    const FloatTexture* v_roughness,
    const SpectrumTexture* normal
)
    : Material(TypeIndexOf<SubsurfaceDiffusionMaterial>())
    , reflectance{ reflectance }
    , l{ mfp }
    , eta{ eta }
    , u_roughness{ u_roughness }
    , v_roughness{ v_roughness }
    , normal{ normal }
{
}

Spectrum SubsurfaceDiffusionMaterial::Le(const Intersection& isect, const Vec3& wo) const
{
    BulbitNotUsed(isect);
    BulbitNotUsed(wo);
    return Spectrum::black;
}

bool SubsurfaceDiffusionMaterial::GetBSDF(BSDF* bsdf, const Intersection& isect, const Vec3& wo, Allocator& alloc) const
{
    BulbitNotUsed(wo);

    Float alpha_x = u_roughness->Evaluate(isect.uv);
    Float alpha_y = v_roughness->Evaluate(isect.uv);

    Float eta_p = isect.front_face ? eta : 1 / eta;

    *bsdf = BSDF(
        isect.shading.normal, isect.shading.tangent,
        alloc.new_object<DielectricBxDF>(eta_p, TrowbridgeReitzDistribution(alpha_x, alpha_y), Spectrum(1))
    );

    return true;
}

bool SubsurfaceDiffusionMaterial::GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, const Vec3& wo, Allocator& alloc) const
{
    Spectrum R = reflectance->Evaluate(isect.uv);
    Spectrum s = Spectrum(1.9f) - R + 3.5f * Sqr(R - Spectrum(0.8f)); // Eq. 6
    Spectrum d = l / s;

    *bssrdf = alloc.new_object<DisneyBSSRDF>(R, d, isect, wo, eta);
    return true;
}

const FloatTexture* SubsurfaceDiffusionMaterial::GetAlphaTexture() const
{
    return nullptr;
}

const SpectrumTexture* SubsurfaceDiffusionMaterial::GetEmissionTexture() const
{
    return nullptr;
}

const SpectrumTexture* SubsurfaceDiffusionMaterial::GetNormalTexture() const
{
    return normal;
}

} // namespace bulbit
