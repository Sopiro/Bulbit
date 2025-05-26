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
    const FloatTexture* transmission,
    const FloatTexture* clearcoat,
    const FloatTexture* clearcoat_roughness,
    const SpectrumTexture* emissive,
    const SpectrumTexture* normalmap,
    const FloatTexture* alpha

)
    : Material(TypeIndexOf<PrincipledMaterial>())
    , basecolor{ basecolor }
    , metallic{ metallic }
    , roughness{ roughness }
    , anisotropy{ anisotropy }
    , ior{ std::max(ior, 1.01f) }
    , transmission{ transmission }
    , clearcoat{ clearcoat }
    , clearcoat_roughness{ clearcoat_roughness }
    , emissive{ emissive }
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

    Float eta = isect.front_face ? ior : 1 / ior;

    Spectrum color = basecolor->Evaluate(isect.uv);
    Float metal = metallic->Evaluate(isect.uv);
    Float rough = roughness->Evaluate(isect.uv);
    Float aniso = anisotropy->Evaluate(isect.uv);
    Float trans = transmission->Evaluate(isect.uv);
    Float cc = clearcoat->Evaluate(isect.uv);
    Float cc_rough = clearcoat_roughness->Evaluate(isect.uv);

    Point2 alpha = PrincipledBxDF::RoughnessToAlpha(rough, aniso);
    Float cc_alpha = PrincipledBxDF::RoughnessToAlpha(cc_rough);

    *bsdf = BSDF(
        n, isect.shading.tangent,
        alloc.new_object<PrincipledBxDF>(
            color, metal, TrowbridgeReitzDistribution(alpha.x, alpha.y), eta, trans, cc,
            TrowbridgeReitzDistribution(cc_alpha, cc_alpha)
        )
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
