#include "bulbit/bxdfs.h"
#include "bulbit/materials.h"
#include "bulbit/textures.h"

namespace bulbit
{

DiffuseMaterial::DiffuseMaterial(
    const SpectrumTexture* reflectance, const FloatTexture* roughness, const SpectrumTexture* normalmap, const FloatTexture* alpha
)
    : Material(TypeIndexOf<DiffuseMaterial>())
    , reflectance{ reflectance }
    , roughness{ roughness }
    , normalmap{ normalmap }
    , alpha{ alpha }
{
}

Float DiffuseMaterial::GetAlpha(const Intersection& isect) const
{
    return alpha ? alpha->Evaluate(isect.uv) : 1;
}

const SpectrumTexture* DiffuseMaterial::GetNormalMap() const
{
    return normalmap;
}

Spectrum DiffuseMaterial::Le(const Intersection& isect, const Vec3& wo) const
{
    BulbitNotUsed(isect);
    BulbitNotUsed(wo);
    return Spectrum::black;
}

bool DiffuseMaterial::GetBSDF(BSDF* bsdf, const Intersection& isect, const Vec3& wo, Allocator& alloc) const
{
    BulbitNotUsed(wo);

    Float r = roughness ? roughness->Evaluate(isect.uv) : 0;
    Spectrum rho = reflectance->Evaluate(isect.uv);

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

bool DiffuseMaterial::GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, const Vec3& wo, Allocator& alloc) const
{
    BulbitNotUsed(bssrdf);
    BulbitNotUsed(isect);
    BulbitNotUsed(wo);
    BulbitNotUsed(alloc);
    return false;
}

} // namespace bulbit
