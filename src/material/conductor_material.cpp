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
    const SpectrumTexture* normalmap,
    const FloatTexture* alpha
)
    : Material(TypeIndexOf<ConductorMaterial>())
    , eta{ eta }
    , k{ k }
    , u_roughness{ u_roughness }
    , v_roughness{ v_roughness }
    , energy_compensation{ energy_compensation }
    , normalmap{ normalmap }
    , alpha{ alpha }
{
}

ConductorMaterial::ConductorMaterial(
    const SpectrumTexture* reflectance,
    const FloatTexture* u_roughness,
    const FloatTexture* v_roughness,
    bool energy_compensation,
    const SpectrumTexture* normalmap,
    const FloatTexture* alpha
)
    : Material(TypeIndexOf<ConductorMaterial>())
    , eta{ nullptr }
    , k{ reflectance }
    , u_roughness{ u_roughness }
    , v_roughness{ v_roughness }
    , energy_compensation{ energy_compensation }
    , normalmap{ normalmap }
    , alpha{ alpha }
{
}

Float ConductorMaterial::GetAlpha(const Intersection& isect) const
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

const SpectrumTexture* ConductorMaterial::GetNormalMap() const
{
    return normalmap;
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

    Spectrum eta_s, k_s;
    if (eta)
    {
        eta_s = eta->Evaluate(isect.uv);
        k_s = k->Evaluate(isect.uv);
    }
    else
    {
        // The reflectance R for a conductor is:
        // R = \frac{(\eta - 1)^2 + k^2}{(\eta + 1)^2 + k^2}
        // Assume \eta = 1 and solve for k

        eta_s = Spectrum(1.0f);
        Spectrum r = Clamp(k->Evaluate(isect.uv), 0, 0.9999);
        k_s = 2 * Sqrt(r) / Sqrt(Max(Spectrum(1) - r, 0));
    }

    Float alpha_x = u_roughness->Evaluate(isect.uv);
    Float alpha_y = v_roughness->Evaluate(isect.uv);

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

} // namespace bulbit
