#include "bulbit/lights.h"
#include "bulbit/sampling.h"

namespace bulbit
{

DirectionalLight::DirectionalLight(const Vec3& direction, const Spectrum& intensity)
    : Light(TypeIndexOf<DirectionalLight>())
    , dir{ Normalize(direction) }
    , intensity{ intensity }
{
}

void DirectionalLight::Preprocess(const AABB& world_bounds)
{
    world_bounds.ComputeBoundingSphere(&world_center, &world_radius);
}

Spectrum DirectionalLight::Le(const Ray& ray) const
{
    BulbitAssert(false);
    BulbitNotUsed(ray);
    return Spectrum::black;
}

bool DirectionalLight::Sample_Li(LightSampleLi* sample, const Intersection& ref, Point2 u) const
{
    BulbitNotUsed(ref);
    BulbitNotUsed(u);

    sample->wi = -dir;
    sample->pdf = 1;
    sample->visibility = 2 * world_radius;
    sample->Li = intensity;

    return true;
}

Float DirectionalLight::EvaluatePDF_Li(const Ray& ray) const
{
    BulbitAssert(false);
    BulbitNotUsed(ray);
    return 0;
}

bool DirectionalLight::Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1) const
{
    BulbitNotUsed(u1);

    Frame frame(dir);

    Point2 u_disk = SampleUniformUnitDiskConcentric(u0);
    Point3 p_disk = world_center + world_radius * frame.FromLocal(Point3(u_disk, 0));

    sample->ray = Ray(p_disk - world_radius * dir, dir);
    sample->normal = dir;
    sample->pdf_p = 1 / (pi * Sqr(world_radius));
    sample->pdf_w = 1;
    sample->Le = intensity;
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
    // This functions should be called by AreaLight only
    BulbitNotUsed(pdf_p);
    BulbitNotUsed(pdf_w);
    BulbitNotUsed(isect);
    BulbitNotUsed(w);
    BulbitAssert(false);
}

} // namespace bulbit
