#pragma once

#include "bsdf.h"
#include "intersectable.h"

namespace bulbit
{

class Primitive;
class AreaLight;

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
        : point{ 0 }
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

    bool IsConnectible() const
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
        return type == VertexType::light || (type == VertexType::surface && sv.area_light);
    }

    bool IsDeltaLight() const
    {
        return type == VertexType::light && lv.light && lv.light->IsDeltaLight();
    }

    Spectrum Le(const Vertex& v) const
    {
        if (!IsLight())
        {
            return Spectrum::black;
        }

        Vec3 w = v.point - point;
        if (w.Normalize() == 0)
        {
            return Spectrum::black;
        }

        if (sv.area_light)
        {
            Intersection isect{ .normal = normal, .front_face = sv.front_face };
            return sv.primitive->GetMaterial()->Le(isect, w);
        }
        else
        {
            return Spectrum::black;
        }
    }

    Spectrum f(const Vec3& wi, TransportDirection mode) const
    {
        switch (type)
        {
        case VertexType::surface:
            return sv.bsdf.f(wo, wi, mode);
        case VertexType::medium:
            return Spectrum(mv.phase->p(wo, wi));
        default:
            BulbitAssert(false);
            return Spectrum::black;
        }
    }

    Spectrum f(const Vertex& next, TransportDirection mode) const
    {
        Vec3 wi = next.point - point;
        if (wi.Normalize() == 0)
        {
            return Spectrum::black;
        }

        return f(wi, mode);
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
