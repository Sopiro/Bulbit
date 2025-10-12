#include "bulbit/lights.h"
#include "bulbit/medium.h"
#include "bulbit/primitive.h"
#include "bulbit/sampling.h"

namespace bulbit
{

SpotAreaLight::SpotAreaLight(
    const Primitive* primitive,
    const SpectrumTexture* emission,
    Float angle_max,           // degrees
    Float angle_falloff_start, // degrees
    bool two_sided
)
    : Light(TypeIndexOf<SpotAreaLight>())
    , primitive{ primitive }
    , emission{ emission }
    , cos_theta_min{ std::cos(DegToRad(angle_falloff_start)) }
    , cos_theta_max{ std::cos(DegToRad(angle_max)) }
    , two_sided{ two_sided }
{
    BulbitAssert(cos_theta_min > cos_theta_max);
}

void SpotAreaLight::Preprocess(const AABB& world_bounds)
{
    BulbitNotUsed(world_bounds);
}

Spectrum SpotAreaLight::Le(const Intersection& isect, const Vec3& wo) const
{
    if (isect.front_face || two_sided)
    {
        Float cos_theta = Dot(wo, isect.normal);
        return SmoothStep(cos_theta_max, cos_theta_min, cos_theta) * emission->Evaluate(isect.uv);
    }
    else
    {
        return Spectrum::black;
    }
}

Spectrum SpotAreaLight::Le(const Ray& ray) const
{
    BulbitAssert(false);
    BulbitNotUsed(ray);
    return Spectrum::black;
}

bool SpotAreaLight::Sample_Li(LightSampleLi* sample, const Intersection& ref, Point2 u) const
{
    const Shape* shape = primitive->GetShape();
    ShapeSample shape_sample = shape->Sample(ref.point, u);

    Vec3 wi = shape_sample.point - ref.point;
    Float distance = wi.Normalize();

    bool front_face = Dot(shape_sample.normal, wi) < 0;
    Vec3 normal = shape_sample.normal;
    if (!front_face)
    {
        normal.Negate();
    }

    Float cos_theta = Dot(-wi, normal); // directional spot along -normal
    Spectrum l = SmoothStep(cos_theta_max, cos_theta_min, cos_theta) * emission->Evaluate(shape_sample.uv);
    if (l.IsBlack())
    {
        return false;
    }

    sample->normal = normal;
    sample->point = shape_sample.point;

    sample->wi = wi;
    sample->visibility = distance - Ray::epsilon;
    sample->pdf = shape_sample.pdf;
    sample->Li = l;

    return true;
}

Float SpotAreaLight::EvaluatePDF_Li(const Ray& ray) const
{
    BulbitNotUsed(ray);
    BulbitAssert(false);
    return 0;
}

bool SpotAreaLight::Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1) const
{
    const Shape* shape = primitive->GetShape();
    ShapeSample shape_sample = shape->Sample(u0);

    sample->pdf_p = shape_sample.pdf;
    Vec3 normal = shape_sample.normal;

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
            normal.Negate();
            front_face = false;
        }

        w = SampleUniformCone(u1, cos_theta_max);
        pdf_w = 0.5f * UniformConePDF(cos_theta_max);
    }
    else
    {
        front_face = true;
        w = SampleUniformCone(u1, cos_theta_max);
        pdf_w = UniformConePDF(cos_theta_max);
    }

    sample->pdf_w = pdf_w;

    Float cos_theta = CosTheta(w);
    sample->Le = SmoothStep(cos_theta_max, cos_theta_min, cos_theta) * emission->Evaluate(shape_sample.uv);

    Frame f(normal);
    w = f.FromLocal(w);

    sample->ray = Ray(shape_sample.point, w);
    sample->normal = normal;

    MediumInterface medium_interface = primitive->GetMediumInterface();
    sample->medium = front_face ? medium_interface.outside : medium_interface.inside;

    return true;
}

void SpotAreaLight::EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const
{
    // This function shouldn't be called by AreaLight
    BulbitNotUsed(pdf_p);
    BulbitNotUsed(pdf_w);
    BulbitNotUsed(ray);
}

void SpotAreaLight::PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const
{
    BulbitNotUsed(isect);
    BulbitNotUsed(w);

    *pdf_p = primitive->GetShape()->PDF(isect);
    *pdf_w = (Dot(isect.normal, w) >= cos_theta_max) ? UniformConePDF(cos_theta_max) : 0;
}

Spectrum SpotAreaLight::Phi() const
{
    const Shape* shape = primitive->GetShape();
    Float cone_integral = two_pi * ((1 - cos_theta_min) + (cos_theta_min - cos_theta_max) / 2);
    return emission->Average() * shape->Area() * cone_integral;
}

} // namespace bulbit
