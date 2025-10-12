#include "bulbit/bsdf.h"
#include "bulbit/bxdfs.h"
#include "bulbit/intersectable.h"
#include "bulbit/materials.h"

namespace bulbit
{

ThinDielectricMaterial::ThinDielectricMaterial(Float eta, const SpectrumTexture* reflectance)
    : Material(TypeIndexOf<ThinDielectricMaterial>())
    , eta{ eta }
    , reflectance{ reflectance }
{
}

bool ThinDielectricMaterial::GetBSDF(BSDF* bsdf, const Intersection& isect, Allocator& alloc) const
{
    Spectrum r = reflectance->Evaluate(isect.uv);
    *bsdf = BSDF(isect.shading.normal, isect.shading.tangent, alloc.new_object<ThinDielectricBxDF>(eta, r));
    return true;
}

bool ThinDielectricMaterial::GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, Allocator& alloc) const
{
    BulbitNotUsed(bssrdf);
    BulbitNotUsed(isect);
    BulbitNotUsed(alloc);
    return false;
}

const FloatTexture* ThinDielectricMaterial::GetAlphaTexture() const
{
    return nullptr;
}

const SpectrumTexture* ThinDielectricMaterial::GetNormalTexture() const
{
    return nullptr;
}

} // namespace bulbit
