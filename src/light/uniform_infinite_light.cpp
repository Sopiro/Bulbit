#include "bulbit/lights.h"
#include "bulbit/sampling.h"

namespace bulbit
{

UniformInfiniteLight::UniformInfiniteLight(const Spectrum& l, Float scale)
    : Light(TypeIndexOf<UniformInfiniteLight>())
    , l{ l }
    , scale{ scale }
{
}

void UniformInfiniteLight::Preprocess(const AABB& world_bounds)
{
    world_bounds.ComputeBoundingSphere(&world_center, &world_radius);
}

Spectrum UniformInfiniteLight::Le(const Ray& ray) const
{
    BulbitNotUsed(ray);
    return scale * l;
}

bool UniformInfiniteLight::Sample_Li(LightSampleLi* sample, const Intersection& ref, Point2 u) const
{
    Vec3 wi = SampleUniformSphere(u);
    sample->wi = wi;
    sample->normal = Vec3(0);
    sample->point = ref.point + wi * 2 * world_radius;

    sample->pdf = UniformSpherePDF();
    sample->visibility = 2 * world_radius;
    sample->Li = scale * l;

    return true;
}

Float UniformInfiniteLight::EvaluatePDF_Li(const Ray& ray) const
{
    BulbitNotUsed(ray);
    return UniformSpherePDF();
}

bool UniformInfiniteLight::Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1) const
{
    Vec3 wo = SampleUniformSphere(u1);

    Frame frame(wo);

    Point2 u_disk = SampleUniformUnitDiskConcentric(u0);
    Point3 p_disk = world_center + world_radius * frame.FromLocal(Point3(u_disk, 0));

    sample->ray = Ray(p_disk - world_radius * wo, wo);
    sample->normal = Vec3(0);
    sample->pdf_p = 1 / (pi * Sqr(world_radius));
    sample->pdf_w = UniformSpherePDF();
    sample->Le = scale * l;
    sample->medium = nullptr;

    return true;
}

void UniformInfiniteLight::EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const
{
    BulbitNotUsed(ray);

    *pdf_w = UniformSpherePDF();
    *pdf_p = 1 / (pi * Sqr(world_radius));
}

void UniformInfiniteLight::PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const
{
    // This function should be called by AreaLight only
    BulbitNotUsed(pdf_p);
    BulbitNotUsed(pdf_w);
    BulbitNotUsed(isect);
    BulbitNotUsed(w);
    BulbitAssert(false);
}

Spectrum UniformInfiniteLight::Phi() const
{
    return scale * l * four_pi * pi * Sqr(world_radius);
}

} // namespace bulbit
