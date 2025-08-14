#include "bulbit/bsdf.h"
#include "bulbit/bxdfs.h"
#include "bulbit/camera.h"
#include "bulbit/film.h"
#include "bulbit/integrators.h"
#include "bulbit/material.h"
#include "bulbit/path.h"
#include "bulbit/sampler.h"

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
        v.wo = Vec3(0);

        v.beta = Spectrum(1);
        v.delta = false;

        v.pdf_fwd = 1;
        v.pdf_rev = 0;
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

    // Sample light vertex
    {
        Vertex& v = path[0];
        v.type = VertexType::light;
        v.lv.light = sl.light;
        v.lv.infinite_light = sl.light->IsInfiniteLight();

        v.point = light_sample.ray.o;
        v.normal = light_sample.normal;
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
            vertex.delta = false;

            vertex.pdf_fwd = prev.ConvertDensity(pdf, vertex);
            vertex.pdf_rev = 0;
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
    Vertex* light_path, Vertex* camera_path, int32 s, int32 t, const Camera* camera, Sampler& sampler, Point2* p_raster
) const
{
    Spectrum L(0);

    Vertex ve;

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
        if (!v.IsConnectable())
        {
            return Spectrum::black;
        }

        CameraSampleWi camera_sample;
        Intersection ref{ .point = v.point };
        if (!camera->SampleWi(&camera_sample, ref, sampler.Next2D()))
        {
            return Spectrum::black;
        }

        // Sample new camera vertex
        {
            ve.type = VertexType::camera;
            ve.cv.camera = camera;

            ve.point = camera_sample.p_aperture;
            ve.normal = Vec3(0);
            ve.wo = Vec3(0);

            ve.beta = camera_sample.Wi / camera_sample.pdf;
            ve.delta = false;

            ve.pdf_fwd = 1;
            ve.pdf_rev = 0;
        }

        if (!V(v.point, camera_sample.p_aperture))
        {
            return Spectrum::black;
        }

        L = ve.beta * v.f(camera_sample.wi, TransportDirection::ToCamera) * v.beta;

        if (v.IsOnSurface())
        {
            L *= AbsDot(v.normal, camera_sample.wi);
        }

        *p_raster = camera_sample.p_raster;
    }
    else if (s == 1)
    {
        // Sample light sample and connect it to the camera subpath
        const Vertex& v = camera_path[t - 1];
        if (!v.IsConnectable())
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

        // Sample new light vertex
        {
            ve.type = VertexType::light;
            ve.lv.light = sampled_light.light;
            ve.lv.infinite_light = sampled_light.light->IsInfiniteLight();

            ve.point = light_sample.point;
            ve.normal = light_sample.normal;
            ve.wo = Vec3(0);

            ve.beta = light_sample.Li / (sampled_light.pmf * light_sample.pdf);
            ve.delta = false;

            ve.pdf_fwd = ve.PDFLightOrigin(v, infinite_lights, light_sampler);
            ve.pdf_rev = 0;
        }

        L = v.beta * v.f(light_sample.wi, TransportDirection::ToLight) * ve.beta;

        if (v.IsOnSurface())
        {
            L *= AbsDot(v.normal, light_sample.wi);
        }

        if (L.IsBlack())
        {
            return Spectrum::black;
        }

        if (!V(v.point, ve.point))
        {
            return Spectrum::black;
        }
    }
    else
    {
        // Join two paths in the middle
        const Vertex& vl = light_path[s - 1];
        const Vertex& vc = camera_path[t - 1];
        if (!vl.IsConnectable() || !vc.IsConnectable())
        {
            return Spectrum::black;
        }

        L = vc.beta * vc.f(vl, TransportDirection::ToLight) * vl.f(vc, TransportDirection::ToCamera) * vl.beta;

        if (L.IsBlack())
        {
            return Spectrum::black;
        }

        // Incorporate generalized geometry term
        if (!V(vl.point, vc.point))
        {
            return Spectrum::black;
        }

        Vec3 d = vl.point - vc.point;
        Float G = 1 / Length2(d);
        d *= std::sqrt(G);

        if (vl.IsOnSurface())
        {
            G *= AbsDot(vl.normal, d);
        }

        if (vc.IsOnSurface())
        {
            G *= AbsDot(vc.normal, d);
        }

        L *= G;
    }

    if (L.IsBlack())
    {
        return L;
    }

    // Temporally swap end vertex
    if (t == 1)
    {
        std::swap(camera_path[0], ve);
    }
    else if (s == 1)
    {
        std::swap(light_path[0], ve);
    }

    Float mis_weight = WeightMIS(light_path, camera_path, s, t);

    // Reswap
    if (t == 1)
    {
        std::swap(camera_path[0], ve);
    }
    else if (s == 1)
    {
        std::swap(light_path[0], ve);
    }

    return mis_weight * L;
}

Float BiDirectionalPathIntegrator::WeightMIS(Vertex* light_path, Vertex* camera_path, int32 s, int32 t) const
{
    if (s + t == 2)
    {
        return 1;
    }

    Vertex* vc = t > 0 ? &camera_path[t - 1] : nullptr;
    Vertex* vc_prev = t > 1 ? &camera_path[t - 2] : nullptr;
    Vertex* vl = s > 0 ? &light_path[s - 1] : nullptr;
    Vertex* vl_prev = s > 1 ? &light_path[s - 2] : nullptr;

    Float camera_pdf_revs[2] = { 0 };
    Float light_pdf_revs[2] = { 0 };

    if (vc)
    {
        camera_pdf_revs[0] = s > 0 ? vl->PDF(*vc, vl_prev) : vc->PDFLightOrigin(*vc_prev, infinite_lights, light_sampler);
        std::swap(vc->pdf_rev, camera_pdf_revs[0]);
    }
    if (vc_prev)
    {
        camera_pdf_revs[1] = s > 0 ? vc->PDF(*vc_prev, vl) : vc->PDFLight(*vc_prev);
        std::swap(vc_prev->pdf_rev, camera_pdf_revs[1]);
    }
    if (vl)
    {
        light_pdf_revs[0] = vc->PDF(*vl, vc_prev);
        std::swap(vl->pdf_rev, light_pdf_revs[0]);
    }
    if (vl_prev)
    {
        light_pdf_revs[1] = vl->PDF(*vl_prev, vc);
        std::swap(vl_prev->pdf_rev, light_pdf_revs[1]);
    }

    Float ri = 1;
    Float ri_sum = 0;

    for (int32 i = t - 1; i > 0; --i)
    {
        const Vertex* vi = &camera_path[i];
        const Vertex* vi_prev = &camera_path[i - 1];

        Float pdf_rev = vi->pdf_rev;
        Float pdf_fwd = vi->pdf_fwd;

        ri *= ((pdf_rev == 0) ? 1 : pdf_rev) / ((pdf_fwd == 0) ? 1 : pdf_fwd);

        if (!vi->delta && !vi_prev->delta)
        {
            ri_sum += ri;
        }
    }

    ri = 1;
    for (int32 i = s - 1; i >= 0; --i)
    {
        const Vertex* vi = &light_path[i];
        const Vertex* vi_prev = i > 0 ? &light_path[i - 1] : nullptr;

        Float pdf_rev = vi->pdf_rev;
        Float pdf_fwd = vi->pdf_fwd;

        ri *= ((pdf_rev == 0) ? 1 : pdf_rev) / ((pdf_fwd == 0) ? 1 : pdf_fwd);

        if (!vi->delta && !(vi_prev ? vi_prev->delta : vi->IsDeltaLight()))
        {
            ri_sum += ri;
        }
    }

    if (vc)
    {
        std::swap(vc->pdf_rev, camera_pdf_revs[0]);
    }
    if (vc_prev)
    {
        std::swap(vc_prev->pdf_rev, camera_pdf_revs[1]);
    }
    if (vl)
    {
        std::swap(vl->pdf_rev, light_pdf_revs[0]);
    }
    if (vl_prev)
    {
        std::swap(vl_prev->pdf_rev, light_pdf_revs[1]);
    }

    return 1 / (1 + ri_sum);
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

    Spectrum Li(0);

    for (int32 t = 1; t <= num_camera_vertices; ++t)
    {
        for (int32 s = 0; s <= num_light_vertces; ++s)
        {
            int32 bounces = t + s - 2;
            if ((s == 1 && t == 1) || bounces < 0 || bounces > max_bounces)
            {
                continue;
            }

            Point2 p_raster;
            Spectrum L_path = ConnectPaths(light_path, camera_path, s, t, camera, sampler, &p_raster);

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
