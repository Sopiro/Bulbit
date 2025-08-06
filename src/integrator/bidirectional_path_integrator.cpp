#include "bulbit/bxdfs.h"
#include "bulbit/integrators.h"
#include "bulbit/lights.h"
#include "bulbit/material.h"

namespace bulbit
{

BiDirectionalPathIntegrator::BiDirectionalPathIntegrator(
    const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler, int32 max_bounces
)
    : BiDirectionalRayIntegrator(accel, std::move(lights), sampler)
    , light_sampler{ all_lights }
    , max_bounces{ max_bounces }
{
}

bool BiDirectionalPathIntegrator::V(const Point3 p1, const Point3 p2) const
{
    Vec3 d = p2 - p1;
    Float visibility = d.Normalize();
    Ray ray(p1, d);

    Intersection isect;
    while (visibility > 0 && Intersect(&isect, ray, Ray::epsilon, visibility))
    {
        if (isect.primitive->GetMaterial())
        {
            return false;
        }

        ray.o = isect.point;
        visibility -= isect.t;
    }

    return true;
}

Spectrum BiDirectionalPathIntegrator::L(
    const Ray& primary_ray, const Medium* primary_medium, const Camera* camera, Film& film, Sampler& sampler
) const
{
    BulbitNotUsed(primary_medium);
    BulbitNotUsed(camera);
    BulbitNotUsed(film);
    BulbitNotUsed(sampler);

    Intersection isect;
    if (Intersect(&isect, primary_ray, Ray::epsilon, infinity))
    {
        return Spectrum(std::exp(-isect.t));
    }

    return Spectrum(0);
}

} // namespace bulbit
