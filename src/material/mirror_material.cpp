#include "bulbit/bsdf.h"
#include "bulbit/bxdfs.h"
#include "bulbit/intersectable.h"
#include "bulbit/materials.h"
#include "bulbit/textures.h"

namespace bulbit
{

MirrorMaterial::MirrorMaterial(const SpectrumTexture* reflectance, const SpectrumTexture* normal, const FloatTexture* alpha)
    : Material(TypeIndexOf<MirrorMaterial>())
    , reflectance{ reflectance }
    , normal{ normal }
    , alpha{ alpha }
{
}

bool MirrorMaterial::GetBSDF(BSDF* bsdf, const Intersection& isect, Allocator& alloc) const
{
    Spectrum r = reflectance->Evaluate(isect.uv);
    *bsdf = BSDF(isect.shading.normal, isect.shading.tangent, alloc.new_object<SpecularReflectionBxDF>(r));
    return true;
}

bool MirrorMaterial::GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, Allocator& alloc) const
{
    BulbitNotUsed(bssrdf);
    BulbitNotUsed(isect);
    BulbitNotUsed(alloc);
    return false;
}

const FloatTexture* MirrorMaterial::GetAlphaTexture() const
{
    return alpha;
}

const SpectrumTexture* MirrorMaterial::GetNormalTexture() const
{
    return normal;
}

} // namespace bulbit
