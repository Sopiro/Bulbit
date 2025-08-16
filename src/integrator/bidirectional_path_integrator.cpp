#include "bulbit/bsdf.h"
#include "bulbit/bxdfs.h"
#include "bulbit/camera.h"
#include "bulbit/film.h"
#include "bulbit/integrators.h"
#include "bulbit/material.h"
#include "bulbit/path.h"
#include "bulbit/sampler.h"
#include "bulbit/transport.h"

namespace bulbit
{

BiDirectionalPathIntegrator::BiDirectionalPathIntegrator(
    const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler, int32 max_bounces
)
    : BiDirectionalRayIntegrator(accel, std::move(lights), sampler)
    , max_bounces{ max_bounces }
{
}

int32 BiDirectionalPathIntegrator::SampleCameraPath(
    Vertex* path, const Ray& ray, const Camera* camera, Sampler& sampler, Allocator& alloc
) const
{
    Spectrum beta(1);
    Float pdf_p, pdf_w;
    camera->PDF_We(&pdf_p, &pdf_w, ray);

    // Sample camera vertex
    {
        Vertex& v = path[0];
        v.type = VertexType::camera;
        v.cv.camera = camera;

        v.point = ray.o;
        v.normal = Vec3(0);
        v.shading_normal = Vec3(0);
        v.wo = Vec3(0);

        v.beta = Spectrum(1);
        v.delta = false;

        v.pdf_fwd = 1;
        v.pdf_rev = 0;
    }

    return 1 + RandomWalk(this, path + 1, ray, beta, pdf_w, max_bounces + 1, TransportDirection::ToLight, sampler, alloc);
}

int32 BiDirectionalPathIntegrator::SampleLightPath(Vertex* path, Sampler& sampler, Allocator& alloc) const
{
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

    // Sample light vertex
    {
        Vertex& v = path[0];
        v.type = VertexType::light;
        v.lv.light = sl.light;
        v.lv.infinite_light = sl.light->IsInfiniteLight();

        v.point = light_sample.ray.o;
        v.normal = light_sample.normal;
        v.shading_normal = light_sample.normal;
        v.wo = Vec3(0);

        v.beta = light_sample.Le;
        v.delta = false;

        v.pdf_fwd = sl.pmf * light_sample.pdf_p;
        v.pdf_rev = 0;
    }

    Spectrum beta = light_sample.Le / (sl.pmf * light_sample.pdf_p * light_sample.pdf_w);
    if (light_sample.normal != Vec3::zero)
    {
        beta *= AbsDot(light_sample.normal, light_sample.ray.d);
    }

    // Note light paths sample one fewer vertex than the target path length
    int32 num_light_vertices = RandomWalk(
        this, path + 1, light_sample.ray, beta, light_sample.pdf_w, max_bounces, TransportDirection::ToCamera, sampler, alloc
    );

    // Correct sampling densities for initial infinite light vertex
    Vertex& v0 = path[0];
    Vertex& v1 = path[1];
    if (v0.IsInfiniteLight())
    {
        // Set spatial density of primary hit vertex for infinite light
        // by removing inv_dist2 factor
        if (num_light_vertices > 0)
        {
            v1.pdf_fwd = light_sample.pdf_p;

            if (v1.IsOnSurface())
            {
                v1.pdf_fwd *= AbsDot(light_sample.ray.d, v1.normal);
            }
        }

        // Set spatial density of aggregate infinite lights
        // as solid angle density for the aggregate infinite lights
        Ray ray(light_sample.ray.o, -light_sample.ray.d);

        Float pdf = 0;
        for (const Light* light : infinite_lights)
        {
            pdf += light_sampler.EvaluatePMF(light) * light->EvaluatePDF_Li(ray);
        }
        v0.pdf_fwd = pdf;
    }

    return 1 + num_light_vertices;
}

Spectrum BiDirectionalPathIntegrator::L(
    const Ray& primary_ray, const Medium* primary_medium, const Camera* camera, Film& film, Sampler& sampler
) const
{
    BulbitNotUsed(primary_medium);

    BufferResource path_buffer(2 * sizeof(Vertex) * (max_bounces + 2));
    BufferResource vertex_buffer(2 * max_bxdf_size * (max_bounces + 2));
    Allocator path_alloc(&path_buffer);
    Allocator vertex_alloc(&vertex_buffer);

    Vertex* camera_path = (Vertex*)path_alloc.allocate(sizeof(Vertex) * (max_bounces + 2));
    Vertex* light_path = (Vertex*)path_alloc.allocate(sizeof(Vertex) * (max_bounces + 2));

    int32 num_camera_vertices = SampleCameraPath(camera_path, primary_ray, camera, sampler, vertex_alloc);
    int32 num_light_vertices = SampleLightPath(light_path, sampler, vertex_alloc);

    Spectrum Li(0);

    for (int32 t = 1; t <= num_camera_vertices; ++t)
    {
        for (int32 s = 0; s <= num_light_vertices; ++s)
        {
            int32 bounces = t + s - 2;
            if ((s == 1 && t == 1) || bounces < 0 || bounces > max_bounces)
            {
                continue;
            }

            Point2 p_raster;
            Spectrum L_path = ConnectPaths(this, light_path, camera_path, s, t, camera, sampler, &p_raster);

            if (t == 1)
            {
                film.AddSplat(p_raster, L_path);
            }
            else
            {
                Li += L_path;
            }
        }
    }

    return Li;
}

} // namespace bulbit
