#include "bulbit/bxdfs.h"
#include "bulbit/materials.h"

namespace bulbit
{

ThinDielectricMaterial::ThinDielectricMaterial(Float eta, const SpectrumTexture* reflectance)
    : Material(TypeIndexOf<ThinDielectricMaterial>())
    , eta{ eta }
    , reflectance{ reflectance }
{
}

Float ThinDielectricMaterial::GetAlpha(const Intersection& isect) const
{
    BulbitNotUsed(isect);
    return 1;
}

const SpectrumTexture* ThinDielectricMaterial::GetNormalMap() const
{
    return nullptr;
}

Spectrum ThinDielectricMaterial::Le(const Intersection& isect, const Vec3& wo) const
{
    BulbitNotUsed(isect);
    BulbitNotUsed(wo);
    return Spectrum::black;
}

bool ThinDielectricMaterial::GetBSDF(BSDF* bsdf, const Intersection& isect, const Vec3& wo, Allocator& alloc) const
{
    BulbitNotUsed(wo);

    Spectrum r = reflectance->Evaluate(isect.uv);
    *bsdf = BSDF(isect.shading.normal, isect.shading.tangent, alloc.new_object<ThinDielectricBxDF>(eta, r));
    return true;
}

bool ThinDielectricMaterial::GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, const Vec3& wo, Allocator& alloc) const
{
    BulbitNotUsed(bssrdf);
    BulbitNotUsed(isect);
    BulbitNotUsed(wo);
    BulbitNotUsed(alloc);
    return false;
}

} // namespace bulbit
