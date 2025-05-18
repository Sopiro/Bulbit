#include "bulbit/bxdfs.h"
#include "bulbit/materials.h"

namespace bulbit
{

PrincipledMaterial::PrincipledMaterial(
    const SpectrumTexture* basecolor,
    const FloatTexture* metallic,
    const FloatTexture* roughness,
    const FloatTexture* anisotropy,
    Float ior,
    Float transmission,
    const SpectrumTexture* emissive,
    const SpectrumTexture* normalmap,
    const FloatTexture* alpha

)
    : Material{ TypeIndexOf<PrincipledMaterial>() }
    , basecolor{ basecolor }
    , metallic{ metallic }
    , roughness{ roughness }
    , anisotropy{ anisotropy }
    , emissive{ emissive }
    , ior{ ior }
    , transmission{ transmission }
    , normalmap{ normalmap }
    , alpha{ alpha }
{
}

Float PrincipledMaterial::GetAlpha(const Intersection& isect) const
{
    if (alpha)
    {
        return alpha->Evaluate(isect.uv);
    }
    else
    {
        return 1;
    }
}

const SpectrumTexture* PrincipledMaterial::GetNormalMap() const
{
    return normalmap;
}

Spectrum PrincipledMaterial::Le(const Intersection& isect, const Vec3& wo) const
{
    BulbitNotUsed(wo);
    return emissive ? emissive->Evaluate(isect.uv) : Spectrum::black;
}

bool PrincipledMaterial::GetBSDF(BSDF* bsdf, const Intersection& isect, const Vec3& wo, Allocator& alloc) const
{
    Vec3 n = isect.shading.normal;
    if (Dot(n, wo) < 0)
    {
        // Resolve back facing normal by flipping method
        n = Reflect(n, isect.normal);
    }

    Spectrum b = basecolor->Evaluate(isect.uv);
    Float m = metallic->Evaluate(isect.uv);
    Float ratio = std::sqrt(1 - 0.9f * anisotropy->Evaluate(isect.uv));
    Float alpha = PrincipledBxDF::RoughnessToAlpha(roughness->Evaluate(isect.uv));

    Float eta = isect.front_face ? ior : 1 / ior;

    *bsdf = BSDF(
        n, isect.shading.tangent,
        alloc.new_object<PrincipledBxDF>(b, m, TrowbridgeReitzDistribution(alpha / ratio, alpha * ratio), eta, transmission)
    );
    return true;
}

bool PrincipledMaterial::GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, const Vec3& wo, Allocator& alloc) const
{
    BulbitNotUsed(bssrdf);
    BulbitNotUsed(isect);
    BulbitNotUsed(wo);
    BulbitNotUsed(alloc);
    return false;
}

} // namespace bulbit
