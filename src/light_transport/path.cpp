#include "bulbit/path.h"
#include "bulbit/camera.h"
#include "bulbit/integrator.h"
#include "bulbit/light_sampler.h"
#include "bulbit/lights.h"
#include "bulbit/medium.h"
#include "bulbit/primitive.h"

namespace bulbit
{

bool Vertex::IsConnectable() const
{
    switch (type)
    {
    case VertexType::camera:
        return true;
    case VertexType::light:
        return !lv.light->Is<DirectionalLight>();
    case VertexType::surface:
        return IsNonSpecular(sv.bsdf.Flags());
    case VertexType::medium:
        return true;
    default:
        BulbitAssert(false);
        return false;
    }
}

bool Vertex::IsLight() const
{
    return (type == VertexType::light) || (type == VertexType::surface && sv.area_light);
}

bool Vertex::IsDeltaLight() const
{
    return (type == VertexType::light) && lv.light && lv.light->IsDeltaLight();
}

bool Vertex::IsInfiniteLight() const
{
    return (type == VertexType::light) && lv.infinite_light;
}

Spectrum Vertex::Le(const Vertex& v, const Integrator* I) const
{
    if (!IsLight())
    {
        return Spectrum(0);
    }

    if (IsInfiniteLight())
    {
        Ray ray(point, -wo);

        // Return emitted radiance for infinite light sources
        Spectrum Le(0);
        for (const Light* light : I->InfiniteLights())
        {
            Le += light->Le(ray);
        }
        return Le;
    }
    else
    {
        Vec3 wo = v.point - point;
        if (wo.Normalize() == 0)
        {
            return Spectrum::black;
        }

        Intersection isect{ .normal = normal, .front_face = sv.front_face };
        return sv.primitive->GetMaterial()->Le(isect, wo);
    }
}

Spectrum Vertex::f(const Vec3& wi, TransportDirection direction) const
{
    switch (type)
    {
    case VertexType::surface:
        return sv.bsdf.f(wo, wi, direction);
    case VertexType::medium:
        return Spectrum(mv.phase->p(wo, wi));
    default:
        BulbitAssert(false);
        return Spectrum::black;
    }
}

Spectrum Vertex::f(const Vertex& next, TransportDirection direction) const
{
    Vec3 wi = next.point - point;
    if (wi.Normalize() == 0)
    {
        return Spectrum::black;
    }

    return f(wi, direction);
}

Float Vertex::PDF(const Vertex& next, const Vertex* prev, const Integrator* I) const
{
    if (type == VertexType::light)
    {
        BulbitAssert(prev == nullptr);
        return PDFLight(next, I);
    }

    Vec3 wi = next.point - point;
    if (wi.Normalize() == 0)
    {
        return 0;
    }

    Vec3 wo;
    if (prev)
    {
        wo = prev->point - point;
        if (wo.Normalize() == 0)
        {
            return 0;
        }
    }
    else
    {
        BulbitAssert(type == VertexType::camera);
    }

    Float pdf;
    switch (type)
    {
    case VertexType::surface:
        pdf = sv.bsdf.PDF(wo, wi);
        break;
    case VertexType::medium:
        pdf = mv.phase->PDF(wo, wi);
        break;
    case VertexType::camera:
    {
        Float pdf_p;
        cv.camera->PDF_We(&pdf_p, &pdf, Ray(point, wi));
    }
    break;
    default:
        BulbitAssert(false);
        return 0;
    }

    return ConvertDensity(*this, next, pdf);
}

Float Vertex::PDFLight(const Vertex& next, const Integrator* I) const
{
    Vec3 w = next.point - point;
    Float inv_dist2 = 1 / Length2(w);
    w *= std::sqrt(inv_dist2);

    Float pdf;
    if (IsInfiniteLight())
    {
        AABB world_bounds = I->World()->GetAABB();

        Point3 center;
        Float radius;

        world_bounds.ComputeBoundingSphere(&center, &radius);
        pdf = 1 / (pi * Sqr(radius));
    }
    else if (IsOnSurface())
    {
        const Light* light = (type == VertexType::light) ? lv.light : sv.area_light;
        BulbitAssert(light);

        Intersection isect{ .point = point, .normal = normal };
        Float pdf_p, pdf_w;
        light->PDF_Le(&pdf_p, &pdf_w, isect, w);
        pdf = pdf_w * inv_dist2;
    }
    else
    {
        Float pdf_p, pdf_w;
        lv.light->EvaluatePDF_Le(&pdf_p, &pdf_w, Ray(point, w));
        pdf = pdf_w * inv_dist2;
    }

    if (next.IsOnSurface())
    {
        pdf *= AbsDot(next.normal, w);
    }
    return pdf;
}

Float Vertex::PDFLightOrigin(const Vertex& next, const Integrator* I, const LightSampler& light_sampler) const
{
    Vec3 w = next.point - point;
    if (w.Normalize() == 0)
    {
        return 0;
    }

    if (IsInfiniteLight())
    {
        // Return solid angle density for aggregate infinite lights
        Ray ray(point, -w);

        Float pdf = 0;
        for (const Light* light : I->InfiniteLights())
        {
            pdf += light_sampler.EvaluatePMF(light) * light->EvaluatePDF_Li(ray);
        }
        return pdf;
    }
    else
    {
        const Light* light = (type == VertexType::light) ? lv.light : sv.area_light;
        Float pdf_p, pdf_w;
        if (IsOnSurface())
        {
            Intersection isect{ .point = point, .normal = normal };
            light->PDF_Le(&pdf_p, &pdf_w, isect, w);
        }
        else
        {
            light->EvaluatePDF_Le(&pdf_p, &pdf_w, Ray(point, w));
        }

        return light_sampler.EvaluatePMF(light) * pdf_p;
    }
}

} // namespace bulbit
