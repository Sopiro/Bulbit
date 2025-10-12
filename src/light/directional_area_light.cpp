#include "bulbit/lights.h"
#include "bulbit/materials.h"
#include "bulbit/medium.h"
#include "bulbit/primitive.h"
#include "bulbit/sampling.h"

namespace bulbit
{

DirectionalAreaLight::DirectionalAreaLight(const Primitive* primitive, const SpectrumTexture* emission, bool two_sided)
    : Light(TypeIndexOf<DirectionalAreaLight>())
    , primitive{ primitive }
    , emission{ emission }
    , two_sided{ two_sided }
{
}

void DirectionalAreaLight::Preprocess(const AABB& world_bounds)
{
    BulbitNotUsed(world_bounds);
}

Spectrum DirectionalAreaLight::Le(const Intersection& isect, const Vec3& wo) const
{
    if (wo == Vec3::zero)
    {
        return emission->Evaluate(isect.uv);
    }
    else
    {
        return Spectrum::black;
    }
}

Spectrum DirectionalAreaLight::Le(const Ray& ray) const
{
    BulbitAssert(false);
    BulbitNotUsed(ray);
    return Spectrum::black;
}

bool DirectionalAreaLight::Sample_Li(LightSampleLi* sample, const Intersection& ref, Point2 u) const
{
    // This sampling scheme only works for planar shapes
    const Shape* shape = primitive->GetShape();
    ShapeSample shape_sample = shape->Sample(ref.point, u);

    Vec3 wi = shape_sample.point - ref.point;
    bool front_face = Dot(shape_sample.normal, wi) < 0;

    Vec3 normal = shape_sample.normal;
    if (!front_face)
    {
        normal.Negate();
    }

    Intersection isect;
    if (!shape->Intersect(&isect, Ray(ref.point, -normal), 0, infinity))
    {
        return false;
    }

    if (!two_sided && !isect.front_face)
    {
        return false;
    }

    sample->point = isect.point;
    sample->normal = isect.normal;

    sample->wi = -isect.normal;
    sample->visibility = isect.t - Ray::epsilon;

    sample->Li = emission->Evaluate(isect.uv);
    sample->pdf = 1;
    return true;
}

Float DirectionalAreaLight::EvaluatePDF_Li(const Ray& ray) const
{
    BulbitAssert(false);
    BulbitNotUsed(ray);
    return 0;
}

bool DirectionalAreaLight::Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1) const
{
    ShapeSample shape_sample = primitive->GetShape()->Sample(u0);
    sample->pdf_p = shape_sample.pdf;
    sample->normal = shape_sample.normal;

    Vec3 w = shape_sample.normal;
    bool front_face = true;

    if (two_sided)
    {
        if (u1[0] < 0.5f)
        {
            front_face = true;
        }
        else
        {
            w = -w;
            front_face = false;
        }

        sample->pdf_w = 0.5f;
    }
    else
    {
        front_face = true;
        sample->pdf_w = 1;
    }

    sample->ray = Ray(shape_sample.point, w);

    Intersection isect;
    isect.uv = shape_sample.uv;
    isect.front_face = front_face;
    sample->Le = Le(isect, Vec3::zero);

    MediumInterface medium_interface = primitive->GetMediumInterface();
    sample->medium = front_face ? medium_interface.outside : medium_interface.inside;

    return true;
}

void DirectionalAreaLight::PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const
{
    BulbitNotUsed(w);
    *pdf_p = primitive->GetShape()->PDF(isect);
    *pdf_w = 0;
}

Spectrum DirectionalAreaLight::Phi() const
{
    const Shape* shape = primitive->GetShape();
    return emission->Average() * shape->Area() * (two_sided ? 2 : 1);
}

} // namespace bulbit
