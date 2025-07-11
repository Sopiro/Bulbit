#include "bulbit/integrators.h"

namespace bulbit
{

RandomWalkIntegrator::RandomWalkIntegrator(
    const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler, int32 max_bounces
)
    : UniDirectionalRayIntegrator(accel, std::move(lights), sampler)
    , max_bounces{ max_bounces }
{
    for (Light* light : all_lights)
    {
        if (light->Is<ImageInfiniteLight>() || light->Is<UniformInfiniteLight>())
        {
            infinite_lights.push_back(light);
        }
    }
}

Spectrum RandomWalkIntegrator::Li(const Ray& primary_ray, const Medium* primary_medium, Sampler& sampler) const
{
    BulbitNotUsed(primary_medium);

    int32 bounce = 0;
    Spectrum L(0), beta(1);
    Ray ray = primary_ray;

    while (true)
    {
        Intersection isect;
        if (!Intersect(&isect, ray, Ray::epsilon, infinity))
        {
            for (Light* light : infinite_lights)
            {
                L += beta * light->Le(ray);
            }

            break;
        }

        Vec3 wo = Normalize(-ray.d);

        // Add surface emission
        L += beta * isect.Le(wo);

        if (bounce++ >= max_bounces)
        {
            break;
        }

        int8 mem[max_bxdf_size];
        Resource res(mem, sizeof(mem));
        Allocator alloc(&res);
        BSDF bsdf;
        if (!isect.GetBSDF(&bsdf, wo, alloc))
        {
            ray = Ray(isect.point, -wo);
            --bounce;
            continue;
        }

        Vec3 wi = SampleUniformSphere(sampler.Next2D());
        Float pdf = UniformSpherePDF();

        if (Dot(wi, isect.shading.normal) < 0 && !(bsdf.Flags() & BxDF_Flags::Transmission))
        {
            break;
        }

        beta *= bsdf.f(wo, wi) * AbsDot(isect.shading.normal, wi) / pdf;
        ray = Ray(isect.point, wi);
    }

    return L;
}

} // namespace bulbit
