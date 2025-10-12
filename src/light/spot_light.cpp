#include "bulbit/lights.h"
#include "bulbit/sampling.h"

namespace bulbit
{

SpotLight::SpotLight(
    const Point3& position,
    const Vec3& direction,
    const Spectrum& intensity,
    Float angle_max,
    Float angle_falloff_start,
    const Medium* medium
)
    : Light(TypeIndexOf<SpotLight>())
    , p{ position }
    , frame{ -Normalize(direction) }
    , cos_theta_min{ std::cos(DegToRad(angle_falloff_start)) }
    , cos_theta_max{ std::cos(DegToRad(angle_max)) }
    , intensity{ intensity }
    , medium{ medium }
{
    BulbitAssert(cos_theta_min > cos_theta_max);
}

void SpotLight::Preprocess(const AABB& world_bounds)
{
    BulbitNotUsed(world_bounds);
}

Spectrum SpotLight::Le(const Intersection& isect, const Vec3& wo) const
{
    BulbitAssert(false);
    BulbitNotUsed(isect);
    BulbitNotUsed(wo);
    return Spectrum::black;
}

Spectrum SpotLight::Le(const Ray& ray) const
{
    BulbitAssert(false);
    BulbitNotUsed(ray);
    return Spectrum::black;
}

bool SpotLight::Sample_Li(LightSampleLi* sample, const Intersection& ref, Point2 u) const
{
    BulbitNotUsed(u);

    Vec3 wi = p - ref.point;
    Float distance = wi.Normalize();

    Spectrum l = SmoothStep(cos_theta_max, cos_theta_min, CosTheta(-frame.ToLocal(wi))) * intensity;
    if (l.IsBlack())
    {
        return false;
    }

    sample->point = p;
    sample->normal = Vec3(0);

    sample->wi = wi;
    sample->visibility = distance;
    sample->pdf = 1;
    sample->Li = l / (distance * distance);

    return true;
}

Float SpotLight::EvaluatePDF_Li(const Ray& ray) const
{
    BulbitAssert(false);
    BulbitNotUsed(ray);
    return 0;
}

bool SpotLight::Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1) const
{
    BulbitNotUsed(u0);

    Vec3 w = SampleUniformCone(u1, cos_theta_max);

    sample->Le = SmoothStep(cos_theta_max, cos_theta_min, CosTheta(w)) * intensity;
    sample->ray = Ray(p, frame.FromLocal(w));
    sample->normal = Vec3(0);
    sample->pdf_p = 1;
    sample->pdf_w = UniformConePDF(cos_theta_max);
    sample->medium = medium;

    return true;
}

void SpotLight::EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const
{
    BulbitNotUsed(ray);

    *pdf_p = 0;
    *pdf_w = (CosTheta(frame.ToLocal(ray.d)) >= cos_theta_max) ? UniformConePDF(cos_theta_max) : 0;
}

void SpotLight::PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const
{
    // This function should be called by AreaLight only
    BulbitNotUsed(pdf_p);
    BulbitNotUsed(pdf_w);
    BulbitNotUsed(isect);
    BulbitNotUsed(w);
    BulbitAssert(false);
}

Spectrum SpotLight::Phi() const
{
    return intensity * two_pi * ((1 - cos_theta_min) + (cos_theta_min - cos_theta_max) / 2);
}

} // namespace bulbit
