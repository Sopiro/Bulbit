#include "bulbit/lights.h"
#include "bulbit/materials.h"

namespace bulbit
{

AreaLight::AreaLight(const Primitive* primitive, bool two_sided)
    : Light(TypeIndexOf<AreaLight>())
    , primitive{ primitive }
    , two_sided{ two_sided }
{
}

Spectrum AreaLight::Le(const Ray& ray) const
{
    BulbitAssert(false);

    Intersection isect;
    if (!primitive->Intersect(&isect, ray, epsilon, infinity))
    {
        return Spectrum::black;
    }

    return primitive->GetMaterial()->Le(isect, -ray.d);
}

bool AreaLight::Sample_Li(LightSampleLi* sample, const Intersection& ref, Point2 u) const
{
    ShapeSample shape_sample = primitive->GetShape()->Sample(ref.point, u);
    Vec3 ref2p = shape_sample.point - ref.point;

    sample->visibility = ref2p.Normalize() - Ray::epsilon;
    sample->wi = ref2p;
    sample->pdf = shape_sample.pdf;

    Intersection isect;
    isect.point = shape_sample.point;
    isect.front_face = Dot(shape_sample.normal, ref2p) < 0;
    sample->Li = primitive->GetMaterial()->Le(isect, ref2p);

    return true;
}

Float AreaLight::EvaluatePDF_Li(const Ray& ray) const
{
    return primitive->GetShape()->EvaluatePDF(ray);
}

bool AreaLight::Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1) const
{
    ShapeSample shape_sample = primitive->GetShape()->Sample(u0);
    sample->pdf_p = shape_sample.pdf;
    sample->normal = shape_sample.normal;

    Vec3 w;
    Float pdf_w;
    bool front_face;
    if (two_sided)
    {
        if (u1[0] < 0.5f)
        {
            u1[0] = std::min(2 * u1[0], 1 - epsilon);
            front_face = true;
        }
        else
        {
            u1[0] = std::min(2 * (u1[0] - 0.5f), 1 - epsilon);
            sample->normal.Negate();
            front_face = false;
        }

        w = SampleCosineHemisphere(u1);
        pdf_w = 0.5f * CosineHemispherePDF(w.z);
    }
    else
    {
        front_face = true;
        w = SampleCosineHemisphere(u1);
        pdf_w = CosineHemispherePDF(w.z);
    }

    if (pdf_w == 0)
    {
        return false;
    }

    sample->pdf_w = pdf_w;

    Frame f(sample->normal);
    w = f.FromLocal(w);

    Intersection isect;
    isect.point = shape_sample.point;
    isect.front_face = front_face;
    sample->Le = primitive->GetMaterial()->Le(isect, -w);

    const MediumInterface* medium_interface = primitive->GetMediumInterface();
    sample->medium = front_face ? medium_interface->outside : medium_interface->inside;

    sample->ray = Ray(shape_sample.point, w);

    return true;
}

void AreaLight::EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const
{
    Intersection isect;
    if (!primitive->Intersect(&isect, ray, epsilon, infinity))
    {
        *pdf_p = 0;
        *pdf_w = 0;
    }
    else
    {
        PDF_Le(pdf_p, pdf_w, isect, -ray.d);
    }
}

void AreaLight::PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const
{
    *pdf_p = primitive->GetShape()->PDF(isect);
    *pdf_w = two_sided ? (0.5f * CosineHemispherePDF(AbsDot(isect.normal, w))) : CosineHemispherePDF(Dot(isect.normal, w));
}

} // namespace bulbit
