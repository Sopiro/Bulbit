#include "bulbit/integrators.h"

namespace bulbit
{

DebugIntegrator::DebugIntegrator(const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler)
    : UniDirectionalRayIntegrator(accel, std::move(lights), sampler)
{
}

Spectrum DebugIntegrator::Li(const Ray& primary_ray, const Medium* primary_medium, Sampler& sampler) const
{
    BulbitNotUsed(primary_medium);
    BulbitNotUsed(sampler);

    Intersection isect;
    if (!Intersect(&isect, primary_ray, Ray::epsilon, infinity))
    {
        return Spectrum::black;
    }

    // const Material* mat = isect.primitive->GetMaterial();

    // Vec3 wi = Normalize(primary_ray.d);
    // Vec3 wo = Normalize(Reflect(wi, isect.normal));

    // return Spectrum(1 / (1 + isect.t));
    return Spectrum(isect.shading.normal) * 0.5 + Spectrum(0.5);
}

} // namespace bulbit
