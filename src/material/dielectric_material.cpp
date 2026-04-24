#include "bulbit/bsdf.h"
#include "bulbit/bxdfs.h"
#include "bulbit/intersectable.h"
#include "bulbit/materials.h"
#include "bulbit/textures.h"

namespace bulbit
{

namespace
{

bool HasSpectralDispersion(const SpectrumSample& sample)
{
    Float hero_value = sample[WavelengthSample::hero_lane];
    for (int32 i = 1; i < SpectrumSample::num_lanes; ++i)
    {
        if (std::abs(sample[i] - hero_value) > 1e-6f)
        {
            return true;
        }
    }

    return false;
}

} // namespace

DielectricMaterial::DielectricMaterial(
    Float eta,
    const FloatTexture* u_roughness,
    const FloatTexture* v_roughness,
    const SpectrumTexture* reflectance,
    bool energy_compensation,
    const Float3Texture* normal
)
    : DielectricMaterial(Spectrum::Constant(eta), u_roughness, v_roughness, reflectance, energy_compensation, normal)
{
}

DielectricMaterial::DielectricMaterial(
    const Spectrum& eta,
    const FloatTexture* u_roughness,
    const FloatTexture* v_roughness,
    const SpectrumTexture* reflectance,
    bool energy_compensation,
    const Float3Texture* normal
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

bool DielectricMaterial::GetBSDF(BSDF* bsdf, const Intersection& isect, WavelengthSample& lambda, Allocator& alloc) const
{
    SpectrumSample eta_sample = eta.Sample(lambda);
    if (HasSpectralDispersion(eta_sample))
    {
        lambda.CollapseToPrimary();
    }

    Float eta_hero = eta_sample[WavelengthSample::hero_lane];
    if (!isect.front_face)
    {
        eta_hero = 1.0f / eta_hero;
    }
    SpectrumSample r = reflectance->Evaluate(isect.uv, lambda);
    Float alpha_x = TrowbridgeReitzDistribution::RoughnessToAlpha(u_roughness->Evaluate(isect.uv));
    Float alpha_y = TrowbridgeReitzDistribution::RoughnessToAlpha(v_roughness->Evaluate(isect.uv));

    if (energy_compensation)
    {
        *bsdf = BSDF(
            isect.shading.normal, isect.shading.tangent,
            alloc.new_object<DielectricMultiScatteringBxDF>(eta_hero, TrowbridgeReitzDistribution(alpha_x, alpha_y), r)
        );
    }
    else
    {
        *bsdf = BSDF(
            isect.shading.normal, isect.shading.tangent,
            alloc.new_object<DielectricBxDF>(eta_hero, TrowbridgeReitzDistribution(alpha_x, alpha_y), r)
        );
    }

    return true;
}

bool DielectricMaterial::GetBSSRDF(
    BSSRDF** bssrdf, const Intersection& isect, const WavelengthSample& lambda, Allocator& alloc
) const
{
    BulbitNotUsed(bssrdf);
    BulbitNotUsed(isect);
    BulbitNotUsed(lambda);
    BulbitNotUsed(alloc);
    return false;
}

const FloatTexture* DielectricMaterial::GetAlphaTexture() const
{
    return nullptr;
}

const Float3Texture* DielectricMaterial::GetNormalTexture() const
{
    return normal;
}

} // namespace bulbit
