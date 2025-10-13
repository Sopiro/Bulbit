#include "bulbit/lights.h"
#include "bulbit/materials.h"
#include "bulbit/medium.h"
#include "bulbit/primitive.h"
#include "bulbit/sampling.h"

namespace bulbit
{

DirectionalAreaLight::DirectionalAreaLight(const Primitive* primitive, const SpectrumTexture* emission, bool two_sided)
    : Light(TypeIndexOf<DirectionalAreaLight>())
    , primitive{ primitive }
    , emission{ emission }
    , two_sided{ two_sided }
{
}

void DirectionalAreaLight::Preprocess(const AABB& world_bounds)
{
    BulbitNotUsed(world_bounds);
}

Spectrum DirectionalAreaLight::Le(const Intersection& isect, const Vec3& wo) const
{
    BulbitNotUsed(isect);
    BulbitNotUsed(wo);
    return Spectrum::black;
}

Spectrum DirectionalAreaLight::Le(const Ray& ray) const
{
    BulbitAssert(false);
    BulbitNotUsed(ray);
    return Spectrum::black;
}

bool DirectionalAreaLight::Sample_Li(LightSampleLi* sample, const Intersection& ref, Point2 u) const
{
    BulbitNotUsed(sample);
    BulbitNotUsed(ref);
    BulbitNotUsed(u);
    return false;
}

Float DirectionalAreaLight::EvaluatePDF_Li(const Ray& ray) const
{
    BulbitAssert(false);
    BulbitNotUsed(ray);
    return 0;
}

bool DirectionalAreaLight::Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1) const
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
    sample->Le = emission->Evaluate(shape_sample.uv);

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

Spectrum DirectionalAreaLight::Phi() const
{
    const Shape* shape = primitive->GetShape();
    return emission->Average() * shape->Area() * (two_sided ? 2 : 1);
}

} // namespace bulbit
