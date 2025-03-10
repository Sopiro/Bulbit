#include "bulbit/lights.h"
#include "bulbit/sampling.h"

namespace bulbit
{

DirectionalLight::DirectionalLight(const Vec3& direction, const Spectrum& intensity, Float visible_radius)
    : Light(TypeIndexOf<DirectionalLight>())
    , dir{ Normalize(direction) }
    , intensity{ intensity }
    , radius{ visible_radius }
{
}

Spectrum DirectionalLight::Le(const Ray& ray) const
{
    BulbitAssert(false);
    BulbitNotUsed(ray);
    return Spectrum::black;
}

LightSampleLi DirectionalLight::Sample_Li(const Intersection& ref, Point2 u) const
{
    BulbitNotUsed(ref);

    LightSampleLi light_sample;
    light_sample.wi = -dir + SampleInsideUnitSphere(u) * radius;
    light_sample.pdf = 1;
    light_sample.visibility = infinity;
    light_sample.Li = intensity;

    return light_sample;
}

Float DirectionalLight::EvaluatePDF_Li(const Ray& ray) const
{
    BulbitAssert(false);
    BulbitNotUsed(ray);
    return 0;
}

LightSampleLe DirectionalLight::Sample_Le(Point2 u0, Point2 u1) const
{
    BulbitNotUsed(u0);
    BulbitNotUsed(u1);
    return {};
}

void DirectionalLight::EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const
{
    BulbitNotUsed(pdf_p);
    BulbitNotUsed(pdf_w);
    BulbitNotUsed(ray);
}

void DirectionalLight::PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const
{
    // This functions should be called by AreaLight only
    BulbitNotUsed(pdf_p);
    BulbitNotUsed(pdf_w);
    BulbitNotUsed(isect);
    BulbitNotUsed(w);
    BulbitAssert(false);
}

} // namespace bulbit
