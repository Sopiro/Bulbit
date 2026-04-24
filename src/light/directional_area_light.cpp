#include "bulbit/lights.h"
#include "bulbit/materials.h"
#include "bulbit/medium.h"
#include "bulbit/primitive.h"
#include "bulbit/sampling.h"

namespace bulbit
{

DirectionalAreaLight::DirectionalAreaLight(
    const Primitive* primitive, const SpectrumTexture* emission, Float emission_mean_luminance, bool two_sided
)
    : Light(TypeIndexOf<DirectionalAreaLight>())
    , primitive{ primitive }
    , emission{ emission }
    , emission_mean_luminance{ emission_mean_luminance }
    , two_sided{ two_sided }
{
}

void DirectionalAreaLight::Preprocess(const AABB& world_bounds)
{
    BulbitNotUsed(world_bounds);
}

SpectrumSample DirectionalAreaLight::Le(const Intersection& isect, const Vec3& wo, const WavelengthSample& lambda) const
{
    BulbitNotUsed(isect);
    BulbitNotUsed(wo);
    BulbitNotUsed(lambda);
    return SpectrumSample(0);
}

SpectrumSample DirectionalAreaLight::Le(const Ray& ray, const WavelengthSample& lambda) const
{
    BulbitNotUsed(ray);
    BulbitNotUsed(lambda);
    BulbitAssert(false);
    return SpectrumSample(0);
}

bool DirectionalAreaLight::Sample_Li(
    LightSampleLi* sample, const Intersection& ref, Point2 u, const WavelengthSample& lambda
) const
{
    BulbitNotUsed(sample);
    BulbitNotUsed(ref);
    BulbitNotUsed(u);
    BulbitNotUsed(lambda);
    return false;
}

Float DirectionalAreaLight::EvaluatePDF_Li(const Ray& ray) const
{
    BulbitAssert(false);
    BulbitNotUsed(ray);
    return 0;
}

bool DirectionalAreaLight::Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1, const WavelengthSample& lambda) const
{
    ShapeSample shape_sample = primitive->GetShape()->Sample(u0);
    sample->pdf_p = shape_sample.pdf;

    Vec3 w = shape_sample.normal;
    bool front_face = true;

    if (two_sided)
    {
        if (u1[0] < 0.5f)
        {
            front_face = true;
        }
        else
        {
            w = -w;
            front_face = false;
        }

        sample->pdf_w = 0.5f;
    }
    else
    {
        front_face = true;
        sample->pdf_w = 1;
    }

    sample->ray = Ray(shape_sample.point, w);
    sample->normal = Vec3(0);
    sample->Le = emission->Evaluate(shape_sample.uv, lambda);

    MediumInterface medium_interface = primitive->GetMediumInterface();
    sample->medium = front_face ? medium_interface.outside : medium_interface.inside;
    return true;
}

void DirectionalAreaLight::PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const
{
    BulbitNotUsed(w);
    *pdf_p = primitive->GetShape()->PDF(isect);
    *pdf_w = 0;
}

Float DirectionalAreaLight::Power() const
{
    const Shape* shape = primitive->GetShape();
    return emission_mean_luminance * shape->Area() * (two_sided ? 2.0f : 1.0f);
}

} // namespace bulbit
