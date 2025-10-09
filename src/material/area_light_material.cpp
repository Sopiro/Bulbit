#include "bulbit/bsdf.h"
#include "bulbit/bxdfs.h"
#include "bulbit/intersectable.h"
#include "bulbit/materials.h"
#include "bulbit/textures.h"

namespace bulbit
{

AreaLightMaterial::AreaLightMaterial(const SpectrumTexture* emission, bool two_sided, bool directional, const FloatTexture* alpha)
    : Material(TypeIndexOf<AreaLightMaterial>())
    , emission{ emission }
    , alpha{ alpha }
    , two_sided{ two_sided }
    , directional{ directional }
{
}

Spectrum AreaLightMaterial::Le(const Intersection& isect, const Vec3& wo) const
{
    if (directional && wo != Vec3::zero)
    {
        return Spectrum::black;
    }

    Float a = alpha ? alpha->Evaluate(isect.uv) : 1;
    if (a < 1 && HashFloat(isect, wo) > a)
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

bool AreaLightMaterial::GetBSDF(BSDF* bsdf, const Intersection& isect, Allocator& alloc) const
{
    *bsdf = BSDF(isect.shading.normal, isect.shading.tangent, alloc.new_object<LambertianBxDF>(Spectrum(1)));
    return true;
}

bool AreaLightMaterial::GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, Allocator& alloc) const
{
    BulbitNotUsed(bssrdf);
    BulbitNotUsed(isect);
    BulbitNotUsed(alloc);
    return false;
}

const FloatTexture* AreaLightMaterial::GetAlphaTexture() const
{
    return alpha;
}

const SpectrumTexture* AreaLightMaterial::GetEmissionTexture() const
{
    return emission;
}

const SpectrumTexture* AreaLightMaterial::GetNormalTexture() const
{
    return nullptr;
}

bool AreaLightMaterial::IsTwoSided() const
{
    return two_sided;
}

bool AreaLightMaterial::IsDirectional() const
{
    return directional;
}

} // namespace bulbit
