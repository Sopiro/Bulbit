#include "bulbit/lights.h"

namespace bulbit
{

PointLight::PointLight(const Point3& position, const Spectrum& intensity)
    : Light(TypeIndexOf<PointLight>())
    , position{ position }
    , intensity{ intensity }
{
}

LightSample PointLight::Sample_Li(const Intersection& ref, const Point2& u) const
{
    BulbitNotUsed(u);

    Vec3 d = position - ref.point;
    Float distance = d.Normalize();

    LightSample light_sample;
    light_sample.wi = d;
    light_sample.visibility = distance;
    light_sample.pdf = 1;
    light_sample.Li = intensity / (distance * distance);

    return light_sample;
}

Float PointLight::EvaluatePDF(const Ray& ray) const
{
    BulbitAssert(false);
    BulbitNotUsed(ray);
    return 0;
}

} // namespace bulbit
