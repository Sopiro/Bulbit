#include "bulbit/lights.h"

namespace bulbit
{

PointLight::PointLight(const Point3& position, const Spectrum& intensity)
    : Light(TypeIndexOf<PointLight>())
    , position{ position }
    , intensity{ intensity }
{
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
    BulbitNotUsed(u1);
    return {};
}

void PointLight::EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const
{
    BulbitNotUsed(pdf_p);
    BulbitNotUsed(pdf_w);
    BulbitNotUsed(ray);
}

} // namespace bulbit
