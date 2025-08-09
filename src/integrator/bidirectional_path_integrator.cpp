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

int32 BiDirectionalPathIntegrator::SampleCameraPath(Vertex* path, const Ray& ray, const Camera* camera, Sampler& sampler) const
{
    if (max_bounces == 0)
    {
        return 0;
    }

    Spectrum beta(1);
    Float pdf_p, pdf_w;
    camera->PDF_We(&pdf_p, &pdf_w, ray);

    // Camera vertex
    {
        Vertex v{ VertexType::camera };
        v.cv.camera = camera;
        v.beta = Spectrum(1);
        v.point = ray.o;

        path[0] = v;
    }

    return 1 + RandomWalk(path + 1, ray, beta, pdf_w, max_bounces - 1, TransportDirection::ToLight, sampler);
}

int32 BiDirectionalPathIntegrator::SampleLightPath(Vertex* path, Sampler& sampler) const
{
    if (max_bounces == 0)
    {
        return 0;
    }

    Intersection isect;
    SampledLight sl;
    if (!light_sampler.Sample(&sl, isect, sampler.Next1D()))
    {
        return 0;
    }

    LightSampleLe light_sample;
    if (!sl.light->Sample_Le(&light_sample, sampler.Next2D(), sampler.Next2D()))
    {
        return 0;
    }

    // Light vertex
    {
        Vertex v{ VertexType::light };
        v.lv.light = sl.light;
        v.beta = light_sample.Le / (sl.pmf * light_sample.pdf_p);
        v.point = light_sample.ray.o;
        v.normal = light_sample.normal;
        path[0] = v;
    }

    Spectrum beta =
        light_sample.Le * AbsDot(light_sample.normal, light_sample.ray.d) / (sl.pmf * light_sample.pdf_p * light_sample.pdf_w);

    return 1 + RandomWalk(
                   path + 1, light_sample.ray, beta, light_sample.pdf_w, max_bounces - 1, TransportDirection::ToCamera, sampler
               );
}

int32 BiDirectionalPathIntegrator::RandomWalk(
    Vertex* path, const Ray& ray, const Spectrum& beta, Float pdf, int32 bounces, TransportDirection direction, Sampler& sampler
) const
{
    BulbitNotUsed(path);
    BulbitNotUsed(ray);
    BulbitNotUsed(beta);
    BulbitNotUsed(pdf);
    BulbitNotUsed(bounces);
    BulbitNotUsed(direction);
    BulbitNotUsed(sampler);
    return 0;
}

Spectrum BiDirectionalPathIntegrator::L(
    const Ray& primary_ray, const Medium* primary_medium, const Camera* camera, Film& film, Sampler& sampler
) const
{
    BulbitNotUsed(primary_medium);
    BulbitNotUsed(film);

    BufferResource buffer(2 * sizeof(Vertex) * (max_bounces + 2));
    Allocator alloc(&buffer);
    Vertex* camera_path = (Vertex*)alloc.allocate(sizeof(Vertex) * (max_bounces + 2));
    Vertex* light_path = (Vertex*)alloc.allocate(sizeof(Vertex) * (max_bounces + 2));

    int32 n_camera = SampleCameraPath(camera_path, primary_ray, camera, sampler);
    int32 n_light = SampleLightPath(light_path, sampler);

    BulbitNotUsed(n_camera);
    BulbitNotUsed(n_light);

    return Spectrum::black;
}

} // namespace bulbit
