#include "bulbit/bxdfs.h"
#include "bulbit/integrators.h"
#include "bulbit/material.h"
#include "bulbit/path.h"

namespace bulbit
{

BiDirectionalPathIntegrator::BiDirectionalPathIntegrator(
    const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler, int32 max_bounces
)
    : BiDirectionalRayIntegrator(accel, std::move(lights), sampler)
    , light_sampler{ all_lights }
    , max_bounces{ max_bounces }
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

Float BiDirectionalPathIntegrator::G(const Vertex& v0, const Vertex& v1) const
{
    if (!V(v0.point, v1.point))
    {
        return 0;
    }

    Vec3 d = v0.point - v1.point;
    Float g = 1 / d.Normalize();

    if (v0.IsOnSurface())
    {
        g *= AbsDot(v0.normal, d);
    }

    if (v1.IsOnSurface())
    {
        g *= AbsDot(v1.normal, d);
    }

    return g;
}

int32 BiDirectionalPathIntegrator::SampleCameraPath(
    Vertex* path, const Ray& ray, const Camera* camera, Sampler& sampler, Allocator& alloc
) const
{
    Spectrum beta(1);
    Float pdf_p, pdf_w;
    camera->PDF_We(&pdf_p, &pdf_w, ray);

    // Create camera vertex
    {
        Vertex& v = path[0];
        v.type = VertexType::camera;
        v.cv.camera = camera;
        v.beta = Spectrum(1);
        v.point = ray.o;
    }

    return 1 + RandomWalk(path + 1, ray, beta, pdf_w, max_bounces + 1, TransportDirection::ToLight, sampler, alloc);
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

    // Create light vertex
    {
        Vertex& v = path[0];
        v.type = VertexType::light;
        v.lv.light = sl.light;
        v.point = light_sample.ray.o;
        v.normal = light_sample.normal;
        v.pdf_fwd = sl.pmf * light_sample.pdf_p;
    }

    Spectrum beta =
        light_sample.Le * AbsDot(light_sample.normal, light_sample.ray.d) / (sl.pmf * light_sample.pdf_p * light_sample.pdf_w);

    // Note light paths sample one fewer vertex than the target path length
    return 1 + RandomWalk(
                   path + 1, light_sample.ray, beta, light_sample.pdf_w, max_bounces, TransportDirection::ToCamera, sampler, alloc
               );
}

int32 BiDirectionalPathIntegrator::RandomWalk(
    Vertex* path,
    Ray ray,
    Spectrum beta,
    Float pdf,
    int32 max_bounces,
    TransportDirection direction,
    Sampler& sampler,
    Allocator& alloc
) const
{
    if (max_bounces == 0)
    {
        return 0;
    }

    int32 bounces = 0;
    while (true)
    {
        Vertex& vertex = path[bounces];
        Vertex& prev = path[bounces - 1];

        Intersection isect;
        bool found_intersection = Intersect(&isect, ray, Ray::epsilon, infinity);
        if (!found_intersection)
        {
            // Don't handle infinite light for now..
            break;
        }

        Vec3 wo = -ray.d;

        BSDF bsdf;
        if (!isect.GetBSDF(&bsdf, wo, alloc))
        {
            ray = Ray(isect.point, -wo);
            continue;
        }

        // Create surface vertex
        {
            vertex.type = VertexType::surface;
            vertex.sv.primitive = isect.primitive;
            vertex.sv.area_light = area_lights.contains(isect.primitive) ? area_lights.at(isect.primitive) : nullptr;
            vertex.sv.front_face = isect.front_face;
            vertex.sv.bsdf = bsdf;

            vertex.point = isect.point;
            vertex.normal = isect.normal;
            vertex.wo = wo;
            vertex.beta = beta;
            vertex.pdf_fwd = prev.ConvertDensity(pdf, vertex);
        }

        if (++bounces >= max_bounces)
        {
            break;
        }

        BSDFSample bsdf_sample;
        if (!bsdf.Sample_f(&bsdf_sample, wo, sampler.Next1D(), sampler.Next2D(), direction))
        {
            break;
        }

        pdf = bsdf_sample.is_stochastic ? bsdf.PDF(wo, bsdf_sample.wi, direction) : bsdf_sample.pdf;
        beta *= bsdf_sample.f * AbsDot(isect.normal, bsdf_sample.wi) / bsdf_sample.pdf;
        ray = Ray(isect.point, bsdf_sample.wi);

        Float pdf_rev = bsdf.PDF(bsdf_sample.wi, wo, !direction);
        if (bsdf_sample.IsSpecular())
        {
            vertex.delta = true;
            pdf = 0;
            pdf_rev = 0;
        }

        prev.pdf_rev = vertex.ConvertDensity(pdf_rev, prev);

        if (beta == Spectrum::black)
        {
            break;
        }
    }

    return bounces;
}

Spectrum BiDirectionalPathIntegrator::ConnectPaths(
    Vertex* light_path, Vertex* camera_path, int32 s, int32 t, const Camera* camera, Film& film, Sampler& sampler
) const
{
    Spectrum L(0);
    Float mis_weight = (s + t == 2) ? 1.0f : 1.0f / (s + t);

    if (s == 0)
    {
        // Interpret the camera path as a complete path
        const Vertex& v = camera_path[t - 1];
        L = v.beta * v.Le(camera_path[t - 2]);
    }
    else if (t == 1)
    {
        // Sample camera sample and connect it to the light subpath
        const Vertex& v = light_path[s - 1];
        if (!v.IsConnectible())
        {
            return Spectrum::black;
        }

        CameraSampleWi camera_sample;
        Intersection ref{ .point = v.point };
        if (!camera->SampleWi(&camera_sample, ref, sampler.Next2D()))
        {
            return Spectrum::black;
        }

        if (!V(v.point, camera_sample.p_aperture))
        {
            return Spectrum::black;
        }

        L = v.beta * camera_sample.Wi * v.f(camera_sample.wi, TransportDirection::ToCamera) / camera_sample.pdf;

        if (v.IsOnSurface())
        {
            L *= AbsDot(v.normal, camera_sample.wi);
        }

        film.AddSplat(camera_sample.p_raster, mis_weight * L);
        return Spectrum::black;
    }
    else if (s == 1)
    {
        // Sample light sample and connect it to the camera subpath
        const Vertex& v = camera_path[t - 1];
        if (!v.IsConnectible())
        {
            return Spectrum::black;
        }

        SampledLight sampled_light;
        Intersection isect{ .point = v.point };
        if (!light_sampler.Sample(&sampled_light, isect, sampler.Next1D()))
        {
            return Spectrum::black;
        }

        LightSampleLi light_sample;
        if (!sampled_light.light->Sample_Li(&light_sample, isect, sampler.Next2D()))
        {
            return Spectrum::black;
        }

        L = v.beta * v.f(light_sample.wi, TransportDirection::ToLight) * light_sample.Li / (sampled_light.pmf * light_sample.pdf);

        if (v.IsOnSurface())
        {
            L *= AbsDot(v.normal, light_sample.wi);
        }

        if (!L.IsBlack() && IntersectAny(Ray(v.point, light_sample.wi), Ray::epsilon, light_sample.visibility))
        {
            return Spectrum::black;
        }
    }
    else
    {
        // Join two paths in the middle
        const Vertex& vl = light_path[s - 1];
        const Vertex& vc = camera_path[t - 1];
        if (vl.IsConnectible() && vc.IsConnectible())
        {
            L = vc.beta * vc.f(vl, TransportDirection::ToLight) * vl.f(vc, TransportDirection::ToCamera) * vl.beta;
        }

        if (!L.IsBlack())
        {
            L *= G(vl, vc);
        }
    }

    return mis_weight * L;
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
    int32 num_light_vertces = SampleLightPath(light_path, sampler, vertex_alloc);

    Spectrum L(0);

    for (int32 t = 1; t <= num_camera_vertices; ++t)
    {
        for (int32 s = 0; s <= num_light_vertces; ++s)
        {
            int32 bounces = t + s - 2;
            if ((s == 1 && t == 1) || bounces < 0 || bounces > max_bounces)
            {
                continue;
            }

            Spectrum L_path = ConnectPaths(light_path, camera_path, s, t, camera, film, sampler);
            if (t != 1)
            {
                L += L_path;
            }
        }
    }

    return L;
}

} // namespace bulbit
