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

LightSampleLi UniformInfiniteLight::Sample_Li(const Intersection& ref, const Point2& u) const
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

LightSampleLe UniformInfiniteLight::Sample_Le(const Point2& u0, const Point2& u1) const
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

} // namespace bulbit
