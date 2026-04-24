#include "bulbit/bsdf.h"
#include "bulbit/bssrdfs.h"
#include "bulbit/bxdfs.h"
#include "bulbit/materials.h"
#include "bulbit/textures.h"

namespace bulbit
{

SubsurfaceRandomWalkMaterial::SubsurfaceRandomWalkMaterial(
    const SpectrumTexture* reflectance,
    const Spectrum& mfp,
    Float eta,
    const FloatTexture* u_roughness,
    const FloatTexture* v_roughness,
    Float g,
    const Float3Texture* normal
)
    : Material(TypeIndexOf<SubsurfaceRandomWalkMaterial>())
    , reflectance{ reflectance }
    , l{ mfp }
    , eta{ eta }
    , u_roughness{ u_roughness }
    , v_roughness{ v_roughness }
    , g{ g }
    , normal{ normal }
{
}

bool SubsurfaceRandomWalkMaterial::GetBSDF(
    BSDF* bsdf, const Intersection& isect, WavelengthSample& lambda, Allocator& alloc
) const
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

bool SubsurfaceRandomWalkMaterial::GetBSSRDF(
    BSSRDF** bssrdf, const Intersection& isect, const WavelengthSample& lambda, Allocator& alloc
) const
{
    SpectrumSample R = reflectance->Evaluate(isect.uv, lambda);
    SpectrumSample sigma_s = SafeDiv(SpectrumSample(1.0f), l.Sample(lambda));

    *bssrdf = alloc.new_object<RandomWalkBSSRDF>(R, SpectrumSample(0), sigma_s, isect, eta, g);
    return true;
}

const FloatTexture* SubsurfaceRandomWalkMaterial::GetAlphaTexture() const
{
    return nullptr;
}

const Float3Texture* SubsurfaceRandomWalkMaterial::GetNormalTexture() const
{
    return normal;
}

} // namespace bulbit
