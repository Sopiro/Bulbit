#include "bulbit/bsdf.h"
#include "bulbit/bxdfs.h"
#include "bulbit/hash.h"
#include "bulbit/intersectable.h"
#include "bulbit/materials.h"
#include "bulbit/textures.h"

namespace bulbit
{

LayeredMaterial::LayeredMaterial(
    const Material* top,
    const Material* bottom,
    bool two_sided,
    const Spectrum& albedo,
    Float thickness,
    Float g,
    int32 max_bounces,
    int32 samples,
    const SpectrumTexture* normal,
    const FloatTexture* alpha
)
    : Material(TypeIndexOf<LayeredMaterial>())
    , top{ top }
    , bottom{ bottom }
    , two_sided{ two_sided }
    , albedo{ albedo }
    , thickness{ thickness }
    , g{ g }
    , max_bounces{ max_bounces }
    , samples{ samples }
    , normal{ normal }
    , alpha{ alpha }
{
    BulbitAssert(top != nullptr);
    BulbitAssert(bottom != nullptr);
}

bool LayeredMaterial::GetBSDF(BSDF* bsdf, const Intersection& isect, Allocator& alloc) const
{
    BSDF top_bsdf, bottom_bsdf;
    if (!top->GetBSDF(&top_bsdf, isect, alloc) || !bottom->GetBSDF(&bottom_bsdf, isect, alloc))
    {
        return false;
    }

    *bsdf = BSDF(
        isect.shading.normal, isect.shading.tangent,
        alloc.new_object<LayeredBxDF>(
            top_bsdf.GetBxDF(), bottom_bsdf.GetBxDF(), two_sided, albedo, thickness, g, max_bounces, samples
        )
    );

    return true;
}

bool LayeredMaterial::GetBSSRDF(BSSRDF** bssrdf, const Intersection& isect, Allocator& alloc) const
{
    BulbitNotUsed(bssrdf);
    BulbitNotUsed(isect);
    BulbitNotUsed(alloc);
    return false;
}

const FloatTexture* LayeredMaterial::GetAlphaTexture() const
{
    return alpha;
}

const SpectrumTexture* LayeredMaterial::GetNormalTexture() const
{
    return normal;
}

} // namespace bulbit
