#include "bulbit/bxdfs.h"
#include "bulbit/integrators.h"
#include "bulbit/lights.h"
#include "bulbit/material.h"

namespace bulbit
{

BiDirectionalPathIntegrator::BiDirectionalPathIntegrator(
    const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler, int32 max_bounces, bool regularize_bsdf
)
    : BiDirectionalRayIntegrator(accel, std::move(lights), sampler)
    , light_sampler{ all_lights }
    , max_bounces{ max_bounces }
    , regularize_bsdf{ regularize_bsdf }
{
    for (Light* light : all_lights)
    {
        switch (light->type_index)
        {
        case Light::TypeIndexOf<UniformInfiniteLight>():
        case Light::TypeIndexOf<ImageInfiniteLight>():
        {
            infinite_lights.push_back(light);
        }
        break;
        case Light::TypeIndexOf<AreaLight>():
        {
            AreaLight* area_light = light->Cast<AreaLight>();
            area_lights.emplace(area_light->GetPrimitive(), area_light);
        }
        break;
        default:
            break;
        }
    }
}

BiDirectionalRaySample BiDirectionalPathIntegrator::L(
    const Camera& camera, const Ray& primary_ray, const Medium* primary_medium, Sampler& sampler
) const
{
    BulbitNotUsed(camera);
    BulbitNotUsed(primary_ray);
    BulbitNotUsed(primary_medium);
    BulbitNotUsed(sampler);

    BiDirectionalRaySample l;

    Intersection isect;
    if (!Intersect(&isect, primary_ray, Ray::epsilon, infinity))
    {
        l.Li = Spectrum::black;
        return l;
    }

    // const Material* mat = isect.primitive->GetMaterial();

    // Vec3 wi = Normalize(primary_ray.d);
    // Vec3 wo = Normalize(Reflect(wi, isect.normal));

    // return Spectrum(1 / (1 + isect.t));
    l.Li = Spectrum(isect.shading.normal) * 0.5 + Spectrum(0.5);
    return l;
}

} // namespace bulbit
