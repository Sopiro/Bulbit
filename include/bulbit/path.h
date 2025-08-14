#pragma once

#include "bsdf.h"

namespace bulbit
{

class Primitive;
class Camera;
class Light;
class AreaLight;
class PhaseFunction;
class LightSampler;

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

    bool IsConnectable() const;

    bool IsLight() const;
    bool IsDeltaLight() const;
    bool IsInfiniteLight() const;

    Spectrum Le(const Vertex& v) const;

    Spectrum f(const Vec3& wi, TransportDirection direction) const;
    Spectrum f(const Vertex& next, TransportDirection direction) const;

    Float PDF(const Vertex& next, const Vertex* prev = nullptr) const;
    Float PDFLight(const Vertex& next) const;
    Float PDFLightOrigin(const Vertex& next, const std::vector<Light*>& infinite_lights, const LightSampler& light_sampler) const;

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
};

// Convert solid angle density to area density
inline Float ConvertDensity(const Vertex& from, const Vertex& to, Float pdf)
{
    Vec3 w = to.point - from.point;
    if (Length2(w) == 0)
    {
        return 0;
    }

    Float inv_dist2 = 1 / Length2(w);
    if (to.IsOnSurface())
    {
        pdf *= AbsDot(to.normal, w * std::sqrt(inv_dist2));
    }

    return pdf * inv_dist2;
}

} // namespace bulbit
