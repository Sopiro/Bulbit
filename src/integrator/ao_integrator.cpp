#include "bulbit/frame.h"
#include "bulbit/integrators.h"
#include "bulbit/sampler.h"

namespace bulbit
{

AOIntegrator::AOIntegrator(const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler, Float ao_range)
    : UniDirectionalRayIntegrator(accel, std::move(lights), sampler, nullptr)
    , range{ ao_range }
{
}

SpectrumSample AOIntegrator::Li(
    const Ray& primary_ray, const Medium* primary_medium, WavelengthSample& lambda, Sampler& sampler
) const
{
    BulbitNotUsed(primary_medium);
    BulbitNotUsed(lambda);

    Intersection isect;
    if (!Intersect(&isect, primary_ray, Ray::epsilon, infinity))
    {
        return SpectrumSample(0);
    }

    Vec3 wi_local = SampleCosineHemisphere(sampler.Next2D());
    // Float pdf = CosineHemispherePDF(wi_local.z);

    if (wi_local.z <= 0)
    {
        return SpectrumSample(0);
    }

    Frame frame(isect.normal);
    Vec3 wi = frame.FromLocal(wi_local);

    Ray ao_ray(isect.point, wi);
    if (IntersectAny(ao_ray, Ray::epsilon, range) == false)
    {
#if 0
        return Spectrum(wi_local.z * inv_pi / pdf);
#else
        return SpectrumSample(1); // importance sampling cancels all terms!
#endif
    }
    else
    {
        return SpectrumSample(0);
    }
}

} // namespace bulbit
