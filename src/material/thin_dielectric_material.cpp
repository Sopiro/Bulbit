#include "bulbit/bsdf.h"
#include "bulbit/bxdfs.h"
#include "bulbit/intersectable.h"
#include "bulbit/materials.h"

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

ThinDielectricMaterial::ThinDielectricMaterial(Float eta, const SpectrumTexture* reflectance)
    : ThinDielectricMaterial(Spectrum::Constant(eta), reflectance)
{
}

ThinDielectricMaterial::ThinDielectricMaterial(const Spectrum& eta, const SpectrumTexture* reflectance)
    : Material(TypeIndexOf<ThinDielectricMaterial>())
    , eta{ eta }
    , reflectance{ reflectance }
{
}

bool ThinDielectricMaterial::GetBSDF(BSDF* bsdf, const Intersection& isect, WavelengthSample& lambda, Allocator& alloc) const
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
    *bsdf = BSDF(isect.shading.normal, isect.shading.tangent, alloc.new_object<ThinDielectricBxDF>(eta_hero, r));
    return true;
}

bool ThinDielectricMaterial::GetBSSRDF(
    BSSRDF** bssrdf, const Intersection& isect, const WavelengthSample& lambda, Allocator& alloc
) const
{
    BulbitNotUsed(bssrdf);
    BulbitNotUsed(isect);
    BulbitNotUsed(lambda);
    BulbitNotUsed(alloc);
    return false;
}

const FloatTexture* ThinDielectricMaterial::GetAlphaTexture() const
{
    return nullptr;
}

const Float3Texture* ThinDielectricMaterial::GetNormalTexture() const
{
    return nullptr;
}

} // namespace bulbit
