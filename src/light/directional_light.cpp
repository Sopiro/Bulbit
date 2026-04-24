#include "bulbit/lights.h"
#include "bulbit/sampling.h"

namespace bulbit
{

DirectionalLight::DirectionalLight(const Vec3& direction, const Spectrum& intensity)
    : Light(TypeIndexOf<DirectionalLight>())
    , w{ Normalize(direction) }
    , intensity{ intensity }
{
}

void DirectionalLight::Preprocess(const AABB& world_bounds)
{
    world_bounds.ComputeBoundingSphere(&world_center, &world_radius);
}

SpectrumSample DirectionalLight::Le(const Intersection& isect, const Vec3& wo, const WavelengthSample& lambda) const
{
    BulbitNotUsed(isect);
    BulbitNotUsed(wo);
    BulbitNotUsed(lambda);
    BulbitAssert(false);
    return SpectrumSample(0);
}

SpectrumSample DirectionalLight::Le(const Ray& ray, const WavelengthSample& lambda) const
{
    BulbitNotUsed(ray);
    BulbitNotUsed(lambda);
    BulbitAssert(false);
    return SpectrumSample(0);
}

bool DirectionalLight::Sample_Li(LightSampleLi* sample, const Intersection& ref, Point2 u, const WavelengthSample& lambda) const
{
    BulbitNotUsed(u);

    sample->wi = w;
    sample->normal = Vec3(0);
    sample->point = ref.point + w * 2 * world_radius;
    sample->pdf = 1;
    sample->visibility = 2 * world_radius;
    sample->Li = intensity.Sample(lambda);
    return true;
}

Float DirectionalLight::EvaluatePDF_Li(const Ray& ray) const
{
    BulbitAssert(false);
    BulbitNotUsed(ray);
    return 0;
}

bool DirectionalLight::Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1, const WavelengthSample& lambda) const
{
    BulbitNotUsed(u1);

    Frame frame(w);
    Point2 u_disk = SampleUniformUnitDiskConcentric(u0);
    Point3 p_disk = world_center + world_radius * frame.FromLocal(Point3(u_disk, 0));

    sample->ray = Ray(p_disk + world_radius * w, -w);
    sample->normal = Vec3(0);
    sample->pdf_p = 1 / (pi * Sqr(world_radius));
    sample->pdf_w = 1;
    sample->Le = intensity.Sample(lambda);
    sample->medium = nullptr;
    return true;
}

void DirectionalLight::EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const
{
    BulbitNotUsed(ray);

    *pdf_p = 1 / (pi * Sqr(world_radius));
    *pdf_w = 0;
}

void DirectionalLight::PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const
{
    // This function should be called by AreaLight only
    BulbitNotUsed(pdf_p);
    BulbitNotUsed(pdf_w);
    BulbitNotUsed(isect);
    BulbitNotUsed(w);
    BulbitAssert(false);
}

Float DirectionalLight::Power() const
{
    return intensity.Luminance() * pi * Sqr(world_radius);
}

} // namespace bulbit
