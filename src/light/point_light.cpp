#include "bulbit/lights.h"

namespace bulbit
{

PointLight::PointLight(const Point3& position, const Spectrum& intensity, const Medium* medium)
    : Light(TypeIndexOf<PointLight>())
    , position{ position }
    , intensity{ intensity }
    , medium{ medium }
{
}

void PointLight::Preprocess(const AABB& world_bounds)
{
    BulbitNotUsed(world_bounds);
}

Spectrum PointLight::Le(const Ray& ray) const
{
    BulbitAssert(false);
    BulbitNotUsed(ray);
    return Spectrum::black;
}

bool PointLight::Sample_Li(LightSampleLi* sample, const Intersection& ref, Point2 u) const
{
    BulbitNotUsed(u);

    Vec3 d = position - ref.point;
    Float distance = d.Normalize();

    sample->wi = d;
    sample->visibility = distance;
    sample->pdf = 1;
    sample->Li = intensity / (distance * distance);

    return true;
}

Float PointLight::EvaluatePDF_Li(const Ray& ray) const
{
    BulbitAssert(false);
    BulbitNotUsed(ray);
    return 0;
}

bool PointLight::Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1) const
{
    BulbitNotUsed(u0);

    Vec3 w = SampleUniformSphere(u1);

    sample->Le = intensity;
    sample->ray = Ray(position, w);
    sample->normal = w;
    sample->pdf_p = 1;
    sample->pdf_w = UniformSpherePDF();
    sample->medium = medium;

    return true;
}

void PointLight::EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const
{
    BulbitNotUsed(ray);
    *pdf_p = 0;
    *pdf_w = UniformSpherePDF();
}

void PointLight::PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const
{
    // This function should be called by AreaLight only
    BulbitNotUsed(pdf_p);
    BulbitNotUsed(pdf_w);
    BulbitNotUsed(isect);
    BulbitNotUsed(w);
    BulbitAssert(false);
}

} // namespace bulbit
