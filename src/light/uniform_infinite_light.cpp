#include "bulbit/lights.h"

namespace bulbit
{

UniformInfiniteLight::UniformInfiniteLight(const Spectrum& l, Float scale)
    : Light(TypeIndexOf<UniformInfiniteLight>())
    , l{ l }
    , scale{ scale }
{
}

Spectrum UniformInfiniteLight::Le(const Ray& ray) const
{
    BulbitNotUsed(ray);
    return scale * l;
}

LightSampleLi UniformInfiniteLight::Sample_Li(const Intersection& ref, Point2 u) const
{
    BulbitNotUsed(ref);

    LightSampleLi light_sample;
    light_sample.wi = SampleUniformSphere(u);
    light_sample.pdf = UniformSpherePDF();
    light_sample.visibility = infinity;
    light_sample.Li = scale * l;

    return light_sample;
}

Float UniformInfiniteLight::EvaluatePDF_Li(const Ray& ray) const
{
    BulbitNotUsed(ray);
    return UniformSpherePDF();
}

LightSampleLe UniformInfiniteLight::Sample_Le(Point2 u0, Point2 u1) const
{
    BulbitNotUsed(u0);
    BulbitNotUsed(u1);
    return {};
}

void UniformInfiniteLight::EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const
{
    BulbitNotUsed(pdf_p);
    BulbitNotUsed(pdf_w);
    BulbitNotUsed(ray);
}

void UniformInfiniteLight::PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const
{
    // This functions should be called by AreaLight only
    BulbitNotUsed(pdf_p);
    BulbitNotUsed(pdf_w);
    BulbitNotUsed(isect);
    BulbitNotUsed(w);
    BulbitAssert(false);
}

} // namespace bulbit
