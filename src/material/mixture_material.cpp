#include "bulbit/bsdf.h"
#include "bulbit/hash.h"
#include "bulbit/intersectable.h"
#include "bulbit/materials.h"
#include "bulbit/textures.h"

namespace bulbit
{

MixtureMaterial::MixtureMaterial(
    const Material* material1, const Material* material2, const FloatTexture* amount, const FloatTexture* alpha
)
    : Material(TypeIndexOf<MixtureMaterial>())
    , mixture_amount{ amount }
    , alpha{ alpha }
{
    BulbitAssert(material1 != nullptr);
    BulbitAssert(material2 != nullptr);
    materials[0] = material1;
    materials[1] = material2;
}

bool MixtureMaterial::GetBSDF(BSDF* bsdf, const Intersection& isect, Allocator& alloc) const
{
    BulbitAssert(false);
    BulbitNotUsed(bsdf);
    BulbitNotUsed(isect);
    BulbitNotUsed(alloc);
    return false;
}

const Material* MixtureMaterial::ChooseMaterial(const Intersection& isect, const Vec3& wo) const
{
    Float m = mixture_amount->Evaluate(isect.uv);
    if (m <= 0)
    {
        return materials[0];
    }
    else if (m >= 1)
    {
        return materials[1];
    }

    Float u = HashFloat(isect.point, wo, materials[0], materials[1]);
    if (m < u)
    {
        return materials[0];
    }
    else
    {
        return materials[1];
    }
}

bool MixtureMaterial::GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, Allocator& alloc) const
{
    BulbitNotUsed(bssrdf);
    BulbitNotUsed(isect);
    BulbitNotUsed(alloc);
    return false;
}

const FloatTexture* MixtureMaterial::GetAlphaTexture() const
{
    return alpha;
}

const SpectrumTexture* MixtureMaterial::GetNormalTexture() const
{
    BulbitAssert(false);
    return nullptr;
}

} // namespace bulbit
