#include "bulbit/bxdfs.h"
#include "bulbit/materials.h"
#include "bulbit/textures.h"

namespace bulbit
{

DiffuseLightMaterial::DiffuseLightMaterial(const SpectrumTexture* emission, bool two_sided, const FloatTexture* alpha)
    : Material(TypeIndexOf<DiffuseLightMaterial>())
    , emission{ emission }
    , alpha{ alpha }
    , two_sided{ two_sided }
{
}

Float DiffuseLightMaterial::GetAlpha(const Intersection& isect) const
{
    return alpha ? alpha->Evaluate(isect.uv) : 1;
}

Spectrum DiffuseLightMaterial::Le(const Intersection& isect, const Vec3& wo) const
{
    BulbitNotUsed(wo);

    Float alpha = GetAlpha(isect);
    if (alpha < 1 && HashFloat(isect, wo) > alpha)
    {
        return Spectrum::black;
    }

    if (isect.front_face || two_sided)
    {
        return emission->Evaluate(isect.uv);
    }
    else
    {
        return Spectrum::black;
    }
}

bool DiffuseLightMaterial::GetBSDF(BSDF* bsdf, const Intersection& isect, const Vec3& wo, Allocator& alloc) const
{
    BulbitNotUsed(wo);

    *bsdf = BSDF(isect.shading.normal, isect.shading.tangent, alloc.new_object<LambertianBxDF>(Spectrum::black));
    return true;
}

bool DiffuseLightMaterial::GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, const Vec3& wo, Allocator& alloc) const
{
    BulbitNotUsed(bssrdf);
    BulbitNotUsed(isect);
    BulbitNotUsed(wo);
    BulbitNotUsed(alloc);
    return false;
}

bool DiffuseLightMaterial::IsTwoSided() const
{
    return two_sided;
}

const SpectrumTexture* DiffuseLightMaterial::GetNormalTexture() const
{
    return nullptr;
}

} // namespace bulbit
