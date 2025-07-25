#include "bulbit/bsdf.h"
#include "bulbit/materials.h"
#include "bulbit/primitive.h"

namespace bulbit
{

static void NormalMapping(Intersection* isect, const SpectrumTexture* normalmap)
{
    BulbitAssert(normalmap != nullptr);

    Spectrum sp = normalmap->Evaluate(isect->uv);
    Vec3 sn(sp.r * 2 - 1, sp.g * 2 - 1, sp.b * 2 - 1);
    sn.Normalize();

    Frame tbn = Frame::FromXZ(isect->shading.tangent, isect->shading.normal);

    Vec3 n = tbn.FromLocal(sn);
    Vec3 t = isect->shading.tangent - Dot(isect->shading.tangent, n) * n;

    isect->shading.normal = n;
    isect->shading.tangent = t;
}

Spectrum Intersection::Le(const Vec3& wo) const
{
    const Material* mat = primitive->GetMaterial();
    if (!mat)
    {
        return Spectrum::black;
    }

    while (mat->Is<MixtureMaterial>())
    {
        mat = ((MixtureMaterial*)mat)->ChooseMaterial(*this, wo);
    }

    return mat->Le(*this, wo);
}

bool Intersection::GetBSDF(BSDF* bsdf, const Vec3& wo, Allocator& alloc)
{
    const Material* mat = primitive->GetMaterial();
    if (!mat)
    {
        return false;
    }

    while (mat->Is<MixtureMaterial>())
    {
        mat = ((MixtureMaterial*)mat)->ChooseMaterial(*this, wo);
    }

    const SpectrumTexture* normalmap = mat->GetNormalMap();
    if (normalmap)
    {
        NormalMapping(this, normalmap);
    }

    return mat->GetBSDF(bsdf, *this, wo, alloc);
}

bool Intersection::GetBSSRDF(BSSRDF** bssrdf, const Vec3& wo, Allocator& alloc)
{
    const Material* mat = primitive->GetMaterial();
    while (mat->Is<MixtureMaterial>())
    {
        mat = ((MixtureMaterial*)mat)->ChooseMaterial(*this, wo);
    }

    if (!mat)
    {
        return false;
    }

    const SpectrumTexture* normalmap = mat->GetNormalMap();
    if (normalmap)
    {
        NormalMapping(this, normalmap);
    }

    return mat->GetBSSRDF(bssrdf, *this, wo, alloc);
}

const Medium* Intersection::GetMedium(const Vec3& w) const
{
    const MediumInterface* medium_interface = primitive->GetMediumInterface();
    if (front_face == (Dot(w, normal) > 0))
    {
        return medium_interface->outside;
    }
    else
    {
        return medium_interface->inside;
    }
}

} // namespace  bulbit
