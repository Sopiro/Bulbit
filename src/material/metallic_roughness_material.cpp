#include "bulbit/bxdfs.h"
#include "bulbit/materials.h"

namespace bulbit
{

MetallicRoughnessMaterial::MetallicRoughnessMaterial(
    const SpectrumTexture* basecolor,
    const FloatTexture* metallic,
    const FloatTexture* u_roughness,
    const FloatTexture* v_roughness,
    const SpectrumTexture* emissive,
    const SpectrumTexture* normalmap,
    const FloatTexture* alpha

)
    : Material(TypeIndexOf<MetallicRoughnessMaterial>())
    , basecolor{ basecolor }
    , metallic{ metallic }
    , u_roughness{ u_roughness }
    , v_roughness{ v_roughness }
    , emissive{ emissive }
    , normalmap{ normalmap }
    , alpha{ alpha }
{
}

Float MetallicRoughnessMaterial::GetAlpha(const Intersection& isect) const
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

const SpectrumTexture* MetallicRoughnessMaterial::GetNormalMap() const
{
    return normalmap;
}

Spectrum MetallicRoughnessMaterial::Le(const Intersection& isect, const Vec3& wo) const
{
    BulbitNotUsed(wo);
    return emissive ? emissive->Evaluate(isect.uv) : Spectrum::black;
}

bool MetallicRoughnessMaterial::GetBSDF(BSDF* bsdf, const Intersection& isect, const Vec3& wo, Allocator& alloc) const
{
    Vec3 n = isect.shading.normal;
    if (Dot(n, wo) < 0)
    {
        // Resolve back facing normal by flipping method
        n = Reflect(n, isect.normal);
    }

    Spectrum b = basecolor->Evaluate(isect.uv);
    Float m = metallic->Evaluate(isect.uv);
    Float alpha_x = MetallicRoughnessBxDF::RoughnessToAlpha(u_roughness->Evaluate(isect.uv));
    Float alpha_y = MetallicRoughnessBxDF::RoughnessToAlpha(v_roughness->Evaluate(isect.uv));

    *bsdf = BSDF(
        n, isect.shading.tangent, alloc.new_object<MetallicRoughnessBxDF>(b, m, TrowbridgeReitzDistribution(alpha_x, alpha_y))
    );
    return true;
}

bool MetallicRoughnessMaterial::GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, const Vec3& wo, Allocator& alloc) const
{
    BulbitNotUsed(bssrdf);
    BulbitNotUsed(isect);
    BulbitNotUsed(wo);
    BulbitNotUsed(alloc);
    return false;
}

} // namespace bulbit
