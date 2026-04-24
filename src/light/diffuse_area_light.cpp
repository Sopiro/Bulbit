#include "bulbit/lights.h"
#include "bulbit/materials.h"
#include "bulbit/medium.h"
#include "bulbit/primitive.h"
#include "bulbit/sampling.h"

namespace bulbit
{

DiffuseAreaLight::DiffuseAreaLight(
    const Primitive* primitive, const SpectrumTexture* emission, Float emission_mean_luminance, bool two_sided
)
    : Light(TypeIndexOf<DiffuseAreaLight>())
    , primitive{ primitive }
    , emission{ emission }
    , emission_mean_luminance{ emission_mean_luminance }
    , two_sided{ two_sided }
{
}

void DiffuseAreaLight::Preprocess(const AABB& world_bounds)
{
    BulbitNotUsed(world_bounds);
}

SpectrumSample DiffuseAreaLight::Le(const Intersection& isect, const Vec3& wo, const WavelengthSample& lambda) const
{
    BulbitNotUsed(wo);
    if (isect.front_face || two_sided)
    {
        return emission->Evaluate(isect.uv, lambda);
    }
    else
    {
        return SpectrumSample(0);
    }
}

SpectrumSample DiffuseAreaLight::Le(const Ray& ray, const WavelengthSample& lambda) const
{
    BulbitNotUsed(ray);
    BulbitNotUsed(lambda);
    BulbitAssert(false);
    return SpectrumSample(0);
}

bool DiffuseAreaLight::Sample_Li(LightSampleLi* sample, const Intersection& ref, Point2 u, const WavelengthSample& lambda) const
{
    ShapeSample shape_sample = primitive->GetShape()->Sample(ref.point, u);
    Vec3 wi = shape_sample.point - ref.point;

    sample->point = shape_sample.point;
    sample->visibility = wi.Normalize() - Ray::epsilon;
    sample->wi = wi;
    sample->pdf = shape_sample.pdf;

    bool front_face = Dot(shape_sample.normal, wi) < 0;
    Vec3 normal = shape_sample.normal;
    if (!front_face)
    {
        normal.Negate();
    }
    sample->normal = normal;

    Intersection isect;
    isect.front_face = front_face;
    isect.uv = shape_sample.uv;
    sample->Li = Le(isect, -wi, lambda);
    return true;
}

Float DiffuseAreaLight::EvaluatePDF_Li(const Ray& ray) const
{
    return primitive->GetShape()->EvaluatePDF(ray);
}

bool DiffuseAreaLight::Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1, const WavelengthSample& lambda) const
{
    ShapeSample shape_sample = primitive->GetShape()->Sample(u0);
    sample->pdf_p = shape_sample.pdf;
    sample->normal = shape_sample.normal;

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
            sample->normal.Negate();
            front_face = false;
        }

        w = SampleCosineHemisphere(u1);
        pdf_w = 0.5f * CosineHemispherePDF(w.z);
    }
    else
    {
        front_face = true;
        w = SampleCosineHemisphere(u1);
        pdf_w = CosineHemispherePDF(w.z);
    }

    if (pdf_w == 0)
    {
        return false;
    }

    sample->pdf_w = pdf_w;

    Frame f(sample->normal);
    w = f.FromLocal(w);

    Intersection isect;
    isect.uv = shape_sample.uv;
    isect.front_face = front_face;
    sample->Le = Le(isect, -w, lambda);

    MediumInterface medium_interface = primitive->GetMediumInterface();
    sample->medium = front_face ? medium_interface.outside : medium_interface.inside;
    sample->ray = Ray(shape_sample.point, w);
    return true;
}

void DiffuseAreaLight::EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const
{
    // This function shouldn't be called by AreaLight
    BulbitNotUsed(pdf_p);
    BulbitNotUsed(pdf_w);
    BulbitNotUsed(ray);
}

void DiffuseAreaLight::PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const
{
    *pdf_p = primitive->GetShape()->PDF(isect);
    *pdf_w = CosineHemispherePDF(AbsDot(isect.normal, w)) * (two_sided ? 0.5f : 1);
}

Float DiffuseAreaLight::Power() const
{
    const Shape* shape = primitive->GetShape();
    return emission_mean_luminance * shape->Area() * pi * (two_sided ? 2.0f : 1.0f);
}

} // namespace bulbit
