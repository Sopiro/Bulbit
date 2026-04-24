#include "bulbit/integrators.h"

namespace bulbit
{

DebugIntegrator::DebugIntegrator(const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler)
    : UniDirectionalRayIntegrator(accel, std::move(lights), sampler, nullptr)
{
}

SpectrumSample DebugIntegrator::Li(
    const Ray& primary_ray, const Medium* primary_medium, WavelengthSample& lambda, Sampler& sampler
) const
{
    BulbitNotUsed(primary_medium);
    BulbitNotUsed(sampler);

    Intersection isect;
    if (!Intersect(&isect, primary_ray, Ray::epsilon, infinity))
    {
        return SpectrumSample(0);
    }

    // const Material* mat = isect.primitive->GetMaterial();

    // Vec3 wi = Normalize(primary_ray.d);
    // Vec3 wo = Normalize(Reflect(wi, isect.normal));

    // return Spectrum(1 / (1 + isect.t));
    return Spectrum::FromRGB(isect.shading.normal * 0.5f + Vec3(0.5f)).Sample(lambda);
}

} // namespace bulbit
