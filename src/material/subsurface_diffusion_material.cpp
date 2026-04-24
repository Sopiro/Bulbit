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
    const Float3Texture* normal
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

bool SubsurfaceDiffusionMaterial::GetBSDF(BSDF* bsdf, const Intersection& isect, WavelengthSample& lambda, Allocator& alloc) const
{
    Float alpha_x = TrowbridgeReitzDistribution::RoughnessToAlpha(u_roughness->Evaluate(isect.uv));
    Float alpha_y = TrowbridgeReitzDistribution::RoughnessToAlpha(v_roughness->Evaluate(isect.uv));

    Float eta_p = isect.front_face ? eta : 1 / eta;

    *bsdf = BSDF(
        isect.shading.normal, isect.shading.tangent,
        alloc.new_object<DielectricMultiScatteringBxDF>(
            SpectrumSample(eta_p), TrowbridgeReitzDistribution(alpha_x, alpha_y), SpectrumSample(1)
        )
    );

    BulbitNotUsed(lambda);

    return true;
}

bool SubsurfaceDiffusionMaterial::GetBSSRDF(
    BSSRDF** bssrdf, const Intersection& isect, const WavelengthSample& lambda, Allocator& alloc
) const
{
    SpectrumSample R = reflectance->Evaluate(isect.uv, lambda);
    SpectrumSample l_sample = l.Sample(lambda);
    SpectrumSample diff = R - SpectrumSample(0.8f);
    SpectrumSample s = SpectrumSample(1.9f) - R + 3.5f * diff * diff;
    SpectrumSample d = SafeDiv(l_sample, s);

    *bssrdf = alloc.new_object<DisneyBSSRDF>(R, d, isect, eta);
    return true;
}

const FloatTexture* SubsurfaceDiffusionMaterial::GetAlphaTexture() const
{
    return nullptr;
}

const Float3Texture* SubsurfaceDiffusionMaterial::GetNormalTexture() const
{
    return normal;
}

} // namespace bulbit
