#include "bulbit/bsdf.h"
#include "bulbit/bxdfs.h"
#include "bulbit/intersectable.h"
#include "bulbit/materials.h"
#include "bulbit/textures.h"

namespace bulbit
{

DiffuseMaterial::DiffuseMaterial(
    const SpectrumTexture* reflectance, const FloatTexture* roughness, const Float3Texture* normal, const FloatTexture* alpha
)
    : Material(TypeIndexOf<DiffuseMaterial>())
    , reflectance{ reflectance }
    , roughness{ roughness }
    , normal{ normal }
    , alpha{ alpha }
{
}

bool DiffuseMaterial::GetBSDF(BSDF* bsdf, const Intersection& isect, WavelengthSample& lambda, Allocator& alloc) const
{
    Float r = roughness ? roughness->Evaluate(isect.uv) : 0;
    SpectrumSample rho = reflectance->Evaluate(isect.uv, lambda);

    if (r > 0)
    {
        *bsdf = BSDF(isect.shading.normal, isect.shading.tangent, alloc.new_object<EONBxDF>(rho, r));
    }
    else
    {
        *bsdf = BSDF(isect.shading.normal, isect.shading.tangent, alloc.new_object<LambertianBxDF>(rho));
    }

    return true;
}

bool DiffuseMaterial::GetBSSRDF(
    BSSRDF** bssrdf, const Intersection& isect, const WavelengthSample& lambda, Allocator& alloc
) const
{
    BulbitNotUsed(bssrdf);
    BulbitNotUsed(isect);
    BulbitNotUsed(lambda);
    BulbitNotUsed(alloc);
    return false;
}

const FloatTexture* DiffuseMaterial::GetAlphaTexture() const
{
    return alpha;
}

const Float3Texture* DiffuseMaterial::GetNormalTexture() const
{
    return normal;
}

} // namespace bulbit
