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

bool SubsurfaceDiffusionMaterial::GetBSDF(BSDF* bsdf, const Intersection& isect, Allocator& alloc) const
{
    Float alpha_x = TrowbridgeReitzDistribution::RoughnessToAlpha(u_roughness->Evaluate(isect.uv));
    Float alpha_y = TrowbridgeReitzDistribution::RoughnessToAlpha(v_roughness->Evaluate(isect.uv));

    Float eta_p = isect.front_face ? eta : 1 / eta;

    *bsdf = BSDF(
        isect.shading.normal, isect.shading.tangent,
        alloc.new_object<DielectricMultiScatteringBxDF>(eta_p, TrowbridgeReitzDistribution(alpha_x, alpha_y), Spectrum(1))
    );

    return true;
}

bool SubsurfaceDiffusionMaterial::GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, Allocator& alloc) const
{
    Spectrum R = reflectance->Evaluate(isect.uv);
    Spectrum s = Spectrum(1.9f) - R + 3.5f * Sqr(R - Spectrum(0.8f)); // Eq. 6
    Spectrum d = l / s;

    *bssrdf = alloc.new_object<DisneyBSSRDF>(R, d, isect, eta);
    return true;
}

const FloatTexture* SubsurfaceDiffusionMaterial::GetAlphaTexture() const
{
    return nullptr;
}

const SpectrumTexture* SubsurfaceDiffusionMaterial::GetNormalTexture() const
{
    return normal;
}

} // namespace bulbit
