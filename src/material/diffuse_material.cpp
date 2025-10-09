#include "bulbit/bsdf.h"
#include "bulbit/bxdfs.h"
#include "bulbit/intersectable.h"
#include "bulbit/materials.h"
#include "bulbit/textures.h"

namespace bulbit
{

DiffuseMaterial::DiffuseMaterial(
    const SpectrumTexture* reflectance, const FloatTexture* roughness, const SpectrumTexture* normal, const FloatTexture* alpha
)
    : Material(TypeIndexOf<DiffuseMaterial>())
    , reflectance{ reflectance }
    , roughness{ roughness }
    , normal{ normal }
    , alpha{ alpha }
{
}

Spectrum DiffuseMaterial::Le(const Intersection& isect, const Vec3& wo) const
{
    BulbitNotUsed(isect);
    BulbitNotUsed(wo);
    return Spectrum::black;
}

bool DiffuseMaterial::GetBSDF(BSDF* bsdf, const Intersection& isect, Allocator& alloc) const
{
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

bool DiffuseMaterial::GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, Allocator& alloc) const
{
    BulbitNotUsed(bssrdf);
    BulbitNotUsed(isect);
    BulbitNotUsed(alloc);
    return false;
}

const FloatTexture* DiffuseMaterial::GetAlphaTexture() const
{
    return alpha;
}

const SpectrumTexture* DiffuseMaterial::GetEmissionTexture() const
{
    return nullptr;
}

const SpectrumTexture* DiffuseMaterial::GetNormalTexture() const
{
    return normal;
}

} // namespace bulbit
