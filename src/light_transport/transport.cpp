#include "bulbit/transport.h"
#include "bulbit/camera.h"
#include "bulbit/integrator.h"
#include "bulbit/light_sampler.h"
#include "bulbit/lights.h"
#include "bulbit/sampler.h"
#include "bulbit/visibility.h"

namespace bulbit
{

int32 RandomWalk(
    const Integrator* I,
    Vertex* path,
    Ray ray,
    Spectrum beta,
    Float pdf,
    int32 max_bounces,
    TransportDirection direction,
    Sampler& sampler,
    Allocator& alloc
)
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

        Vec3 wo = -ray.d;

        Intersection isect;
        bool found_intersection = I->Intersect(&isect, ray, Ray::epsilon, infinity);
        if (!found_intersection)
        {
            // Create infinite light vertex
            if (direction == TransportDirection::ToLight)
            {
                vertex.type = VertexType::light;
                vertex.lv.infinite_light = true;
                vertex.lv.light = nullptr;

                vertex.point = ray.At(1);
                vertex.normal = Vec3(0);
                vertex.shading_normal = Vec3(0);
                vertex.wo = wo;

                vertex.beta = beta;
                vertex.delta = false;

                // Note it stores solid angle density for infinite lights, not area density
                vertex.pdf_fwd = pdf;
                vertex.pdf_rev = 0;
                ++bounces;
            }

            break;
        }

        BSDF bsdf;
        if (!isect.GetBSDF(&bsdf, wo, alloc))
        {
            ray = Ray(isect.point, -wo);
            continue;
        }

        // Create surface vertex
        {
            const AreaLightMap& area_lights = I->AreaLights();

            vertex.type = VertexType::surface;
            vertex.sv.primitive = isect.primitive;
            vertex.sv.area_light = area_lights.contains(isect.primitive) ? area_lights.at(isect.primitive) : nullptr;
            vertex.sv.front_face = isect.front_face;
            vertex.sv.bsdf = bsdf;

            vertex.point = isect.point;
            vertex.normal = isect.normal;
            vertex.shading_normal = isect.shading.normal;
            vertex.wo = wo;

            vertex.beta = beta;
            vertex.delta = false;

            vertex.pdf_fwd = ConvertDensity(prev, vertex, pdf);
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
        beta *= bsdf_sample.f * AbsDot(isect.shading.normal, bsdf_sample.wi) / bsdf_sample.pdf;
        ray = Ray(isect.point, bsdf_sample.wi);

        Float pdf_rev = bsdf.PDF(bsdf_sample.wi, wo, !direction);
        if (bsdf_sample.IsSpecular())
        {
            vertex.delta = true;
            pdf = 0;
            pdf_rev = 0;
        }

        prev.pdf_rev = ConvertDensity(vertex, prev, pdf_rev);

        if (beta == Spectrum::black)
        {
            break;
        }
    }

    return bounces;
}

Float WeightMIS(const Integrator* I, Vertex* light_path, Vertex* camera_path, int32 s, int32 t)
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
        BulbitAssert(vc->IsLight());

        camera_pdf_revs[0] = s > 0 ? vl->PDF(*vc, vl_prev, I) : vc->PDFLightOrigin(*vc_prev, I);
        std::swap(vc->pdf_rev, camera_pdf_revs[0]);
    }
    if (vc_prev)
    {
        camera_pdf_revs[1] = s > 0 ? vc->PDF(*vc_prev, vl, I) : vc->PDFLight(*vc_prev, I);
        std::swap(vc_prev->pdf_rev, camera_pdf_revs[1]);
    }
    if (vl)
    {
        light_pdf_revs[0] = vc->PDF(*vl, vc_prev, I);
        std::swap(vl->pdf_rev, light_pdf_revs[0]);
    }
    if (vl_prev)
    {
        light_pdf_revs[1] = vl->PDF(*vl_prev, vc, I);
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

Spectrum ConnectPaths(
    const Integrator* I,
    Vertex* light_path,
    Vertex* camera_path,
    int32 s,
    int32 t,
    const Camera* camera,
    Sampler& sampler,
    Point2* p_raster
)
{
    // Ignore invalid connections that attempt to connect a light vertex to another light vertex
    if (t > 1 && s > 0 && camera_path[t - 1].type == VertexType::light)
    {
        return Spectrum(0);
    }

    Spectrum L(0);

    Vertex ve;

    if (s == 0)
    {
        // Interpret the camera path as a complete path
        const Vertex& v = camera_path[t - 1];
        L = v.beta * v.Le(camera_path[t - 2], I);
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
            ve.shading_normal = Vec3(0);
            ve.wo = Vec3(0);

            ve.beta = camera_sample.Wi / camera_sample.pdf;
            ve.delta = false;

            ve.pdf_fwd = 1;
            ve.pdf_rev = 0;
        }

        if (!V(I, v.point, camera_sample.p_aperture))
        {
            return Spectrum::black;
        }

        L = ve.beta * v.f(camera_sample.wi, TransportDirection::ToCamera) * v.beta;

        if (v.IsOnSurface())
        {
            L *= AbsDot(v.shading_normal, camera_sample.wi);
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
        if (!I->LightSampler().Sample(&sampled_light, isect, sampler.Next1D()))
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
            ve.shading_normal = light_sample.normal;
            ve.wo = Vec3(0);

            ve.beta = light_sample.Li / (sampled_light.pmf * light_sample.pdf);
            ve.delta = false;

            ve.pdf_fwd = ve.PDFLightOrigin(v, I);
            ve.pdf_rev = 0;
        }

        L = v.beta * v.f(light_sample.wi, TransportDirection::ToLight) * ve.beta;

        if (v.IsOnSurface())
        {
            L *= AbsDot(v.shading_normal, light_sample.wi);
        }

        if (L.IsBlack())
        {
            return Spectrum::black;
        }

        if (!V(I, v.point, ve.point))
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
        if (!V(I, vl.point, vc.point))
        {
            return Spectrum::black;
        }

        Vec3 d = vl.point - vc.point;
        Float G = 1 / Length2(d);
        d *= std::sqrt(G);

        if (vl.IsOnSurface())
        {
            G *= AbsDot(vl.shading_normal, d);
        }

        if (vc.IsOnSurface())
        {
            G *= AbsDot(vc.shading_normal, d);
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

    Float mis_weight = WeightMIS(I, light_path, camera_path, s, t);

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

} // namespace bulbit
