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

LightSampleLi AreaLight::Sample_Li(const Intersection& ref, const Point2& u) const
{
    ShapeSample shape_sample = primitive->GetShape()->Sample(ref.point, u);
    Vec3 ref2p = shape_sample.point - ref.point;

    LightSampleLi light_sample;
    light_sample.visibility = ref2p.Normalize() - Ray::epsilon;
    light_sample.wi = ref2p;
    light_sample.pdf = shape_sample.pdf;

    Intersection isect;
    isect.point = shape_sample.point;
    isect.front_face = Dot(shape_sample.normal, ref2p) < 0;
    light_sample.Li = primitive->GetMaterial()->Le(isect, ref2p);

    return light_sample;
}

Float AreaLight::EvaluatePDF_Li(const Ray& ray) const
{
    return primitive->GetShape()->EvaluatePDF(ray);
}

LightSampleLe AreaLight::Sample_Le(const Point2& u0, const Point2& u1) const
{
    LightSampleLe light_sample;

    ShapeSample shape_sample = primitive->GetShape()->Sample(u0);
    light_sample.pdf_p = shape_sample.pdf;
    light_sample.normal = shape_sample.normal;

    Vec3 w;
    Point2 u = u1;
    if (two_sided)
    {
        if (u[0] < 0.5f)
        {
            u[0] = std::min(2 * u[0], 1 - epsilon);
        }
        else
        {
            u[0] = std::min(2 * (u[0] - 0.5f), 1 - epsilon);
            light_sample.normal.Negate();
        }

        w = SampleCosineHemisphere(u);
        light_sample.pdf_w = 0.5f * CosineHemispherePDF(w.z);
    }
    else
    {
        w = SampleCosineHemisphere(u);
        light_sample.pdf_w = CosineHemispherePDF(w.z);
    }

    Frame f(light_sample.normal);
    w = f.FromLocal(w);

    Intersection isect;
    isect.point = shape_sample.point;
    isect.front_face = true;
    light_sample.Le = primitive->GetMaterial()->Le(isect, -w);

    light_sample.ray = Ray(shape_sample.point, w);

    return light_sample;
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
