#include "bulbit/bxdfs.h"
#include "bulbit/materials.h"

namespace bulbit
{

PrincipledMaterial::PrincipledMaterial(
    const SpectrumTexture* basecolor,
    const FloatTexture* metallic,
    const FloatTexture* u_roughness,
    const FloatTexture* v_roughness,
    const SpectrumTexture* emissive,
    const SpectrumTexture* normalmap,
    const FloatTexture* alpha

)
    : Material{ TypeIndexOf<PrincipledMaterial>() }
    , basecolor{ basecolor }
    , metallic{ metallic }
    , u_roughness{ u_roughness }
    , v_roughness{ v_roughness }
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

    Spectrum b = basecolor->Evaluate(isect.uv);
    Float m = metallic->Evaluate(isect.uv);
    Float alpha_x = PrincipledBxDF::RoughnessToAlpha(u_roughness->Evaluate(isect.uv));
    Float alpha_y = PrincipledBxDF::RoughnessToAlpha(v_roughness->Evaluate(isect.uv));

    Spectrum f0 = PrincipledBxDF::F0(b, m);
    Spectrum F = PrincipledBxDF::F_Schlick(f0, Dot(wo, n));
    Float diff_weight = (1 - m);
    Float spec_weight = F.Luminance();
    // Float spec_weight = std::fmax(F.x, std::fmax(F.y, F.z));
    Float t = Clamp(spec_weight / (diff_weight + spec_weight), 0.15f, 0.9f);

    *bsdf =
        BSDF(n, isect.shading.tangent, alloc.new_object<PrincipledBxDF>(b, m, TrowbridgeReitzDistribution(alpha_x, alpha_y), t));
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
