#include "bulbit/lights.h"

namespace bulbit
{

PointLight::PointLight(const Point3& position, const Spectrum& intensity)
    : Light(TypeIndexOf<PointLight>())
    , position{ position }
    , intensity{ intensity }
{
}

Spectrum PointLight::Le(const Ray& ray) const
{
    BulbitAssert(false);
    BulbitNotUsed(ray);
    return Spectrum::black;
}

LightSampleLi PointLight::Sample_Li(const Intersection& ref, const Point2& u) const
{
    BulbitNotUsed(u);

    Vec3 d = position - ref.point;
    Float distance = d.Normalize();

    LightSampleLi light_sample;
    light_sample.wi = d;
    light_sample.visibility = distance;
    light_sample.pdf = 1;
    light_sample.Li = intensity / (distance * distance);

    return light_sample;
}

Float PointLight::EvaluatePDF_Li(const Ray& ray) const
{
    BulbitAssert(false);
    BulbitNotUsed(ray);
    return 0;
}

LightSampleLe PointLight::Sample_Le(const Point2& u0, const Point2& u1) const
{
    BulbitNotUsed(u0);

    LightSampleLe sample;
    Vec3 w = SampleUniformSphere(u1);

    sample.ray = Ray(position, w);
    sample.normal = w;
    sample.pdf_p = 1;
    sample.pdf_w = UniformSpherePDF();

    return sample;
}

void PointLight::EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const
{
    BulbitNotUsed(ray);
    *pdf_p = 0;
    *pdf_w = UniformSpherePDF();
}

void PointLight::PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const
{
    // This functions should be called by AreaLight only
    BulbitNotUsed(pdf_p);
    BulbitNotUsed(pdf_w);
    BulbitNotUsed(isect);
    BulbitNotUsed(w);
    BulbitAssert(false);
}

} // namespace bulbit
