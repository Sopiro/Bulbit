#include "bulbit/bsdf.h"
#include "bulbit/bxdfs.h"
#include "bulbit/intersectable.h"
#include "bulbit/materials.h"
#include "bulbit/textures.h"

namespace bulbit
{

MirrorMaterial::MirrorMaterial(const SpectrumTexture* reflectance, const Float3Texture* normal, const FloatTexture* alpha)
    : Material(TypeIndexOf<MirrorMaterial>())
    , reflectance{ reflectance }
    , normal{ normal }
    , alpha{ alpha }
{
}

bool MirrorMaterial::GetBSDF(BSDF* bsdf, const Intersection& isect, WavelengthSample& lambda, Allocator& alloc) const
{
    SpectrumSample r = reflectance->Evaluate(isect.uv, lambda);
    *bsdf = BSDF(isect.shading.normal, isect.shading.tangent, alloc.new_object<SpecularReflectionBxDF>(r));
    return true;
}

bool MirrorMaterial::GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, const WavelengthSample& lambda, Allocator& alloc) const
{
    BulbitNotUsed(bssrdf);
    BulbitNotUsed(isect);
    BulbitNotUsed(lambda);
    BulbitNotUsed(alloc);
    return false;
}

const FloatTexture* MirrorMaterial::GetAlphaTexture() const
{
    return alpha;
}

const Float3Texture* MirrorMaterial::GetNormalTexture() const
{
    return normal;
}

} // namespace bulbit
