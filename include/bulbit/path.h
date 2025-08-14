#pragma once

#include "bsdf.h"
#include "camera.h"
#include "light_sampler.h"
#include "lights.h"
#include "media.h"
#include "primitive.h"

namespace bulbit
{

struct SurfaceVertex
{
    const Primitive* primitive;
    const AreaLight* area_light;
    bool front_face;

    BSDF bsdf;
};

struct MediumVertex
{
    const PhaseFunction* phase;
};

struct CameraVertex
{
    const Camera* camera;
};

struct LightVertex
{
    const Light* light;
    bool infinite_light;
};

enum VertexType
{
    surface = 0,
    medium,
    camera,
    light,
};

struct Vertex
{
    VertexType type;

    union
    {
        SurfaceVertex sv;
        MediumVertex mv;
        CameraVertex cv;
        LightVertex lv;
    };

    Point3 point;
    Vec3 normal;
    Vec3 wo;

    Spectrum beta;
    bool delta;

    Float pdf_fwd, pdf_rev;

    Vertex()
        : sv{ 0 }
        , point{ 0 }
        , normal{ 0 }
        , wo{ 0 }
        , beta{ 0 }
        , delta{ false }
        , pdf_fwd{ 0 }
        , pdf_rev{ 0 }
    {
    }

    bool IsOnSurface() const
    {
        return normal != Vec3::zero;
    }

    bool IsConnectable() const
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

    bool IsLight() const
    {
        return (type == VertexType::light) || (type == VertexType::surface && sv.area_light);
    }

    bool IsDeltaLight() const
    {
        return (type == VertexType::light) && lv.light && lv.light->IsDeltaLight();
    }

    bool IsInfiniteLight() const
    {
        return (type == VertexType::light) && lv.infinite_light;
    }

    Spectrum Le(const Vertex& v) const
    {
        Vec3 w = v.point - point;
        if (w.Normalize() == 0)
        {
            return Spectrum::black;
        }

        Intersection isect{ .normal = normal, .front_face = sv.front_face };
        return sv.primitive->GetMaterial()->Le(isect, w);
    }

    Spectrum f(const Vec3& wi, TransportDirection direction) const
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

    Spectrum f(const Vertex& next, TransportDirection direction) const
    {
        Vec3 wi = next.point - point;
        if (wi.Normalize() == 0)
        {
            return Spectrum::black;
        }

        return f(wi, direction);
    }

    Float PDF(const Vertex& next, const Vertex* prev = nullptr) const
    {
        if (type == VertexType::light)
        {
            BulbitAssert(prev == nullptr);
            return PDFLight(next);
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

        return ConvertDensity(pdf, next);
    }

    Float PDFLight(const Vertex& next) const
    {
        Vec3 w = next.point - point;
        Float inv_dist2 = 1 / Length2(w);
        w *= std::sqrt(inv_dist2);

        Float pdf;
        if (IsInfiniteLight())
        {
            // Not handled yet
            pdf = 0;
        }
        else if (IsOnSurface())
        {
            const Light* light = (type == VertexType::light) ? lv.light : sv.area_light;
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

    Float PDFLightOrigin(const Vertex& next, const std::vector<Light*>& infinite_lights, const LightSampler& light_sampler) const
    {
        Vec3 w = next.point - point;
        if (w.Normalize() == 0)
        {
            return 0;
        }

        if (IsInfiniteLight())
        {
            BulbitNotUsed(infinite_lights);
            // Not handled yet
            return 0;
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

    Float ConvertDensity(Float pdf, const Vertex& next) const
    {
        Vec3 w = next.point - point;
        if (Length2(w) == 0)
        {
            return 0;
        }

        Float inv_dist2 = 1 / Length2(w);
        if (next.IsOnSurface())
        {
            pdf *= AbsDot(next.normal, w * std::sqrt(inv_dist2));
        }

        return pdf * inv_dist2;
    }
};

} // namespace bulbit
