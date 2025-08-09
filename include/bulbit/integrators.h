#pragma once

#include "integrator.h"
#include "light_samplers.h"
#include "lights.h"

namespace bulbit
{

class DebugIntegrator : public UniDirectionalRayIntegrator
{
public:
    DebugIntegrator(const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler);

    virtual Spectrum Li(const Ray& ray, const Medium* medium, Sampler& sampler) const override;
};

class RandomWalkIntegrator : public UniDirectionalRayIntegrator
{
public:
    RandomWalkIntegrator(const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler, int32 max_bounces);

    virtual Spectrum Li(const Ray& ray, const Medium* medium, Sampler& sampler) const override;

private:
    std::vector<Light*> infinite_lights;
    int32 max_bounces;
};

class AmbientOcclusion : public UniDirectionalRayIntegrator
{
public:
    AmbientOcclusion(const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler, Float ao_range);

    virtual Spectrum Li(const Ray& ray, const Medium* medium, Sampler& sampler) const override;

private:
    // maximum range to consider occlusuion
    Float range;
};

class AlbedoIntegrator : public UniDirectionalRayIntegrator
{
public:
    AlbedoIntegrator(const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler);

    virtual Spectrum Li(const Ray& ray, const Medium* medium, Sampler& sampler) const override;

private:
    std::vector<Light*> infinite_lights;
};

// Whitted-style raytracer
class WhittedStyle : public UniDirectionalRayIntegrator
{
public:
    WhittedStyle(const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler, int32 max_depth);

    virtual Spectrum Li(const Ray& ray, const Medium* medium, Sampler& sampler) const override
    {
        BulbitNotUsed(medium);

        return Li(ray, sampler, 0);
    }

private:
    Spectrum Li(const Ray& ray, Sampler& sampler, int32 depth) const;

    std::vector<Light*> infinite_lights;

    int32 max_depth;
};

class NaivePathIntegrator : public UniDirectionalRayIntegrator
{
public:
    NaivePathIntegrator(const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler, int32 max_bounces);

    virtual Spectrum Li(const Ray& ray, const Medium* medium, Sampler& sampler) const override;

private:
    Spectrum Li(const Ray& ray, Sampler& sampler, int32 depth) const;

    std::vector<Light*> infinite_lights;

    int32 max_bounces;
};

// Uni-directional path tracer
class PathIntegrator : public UniDirectionalRayIntegrator
{
public:
    PathIntegrator(
        const Intersectable* accel,
        std::vector<Light*> lights,
        const Sampler* sampler,
        int32 max_bounces,
        bool regularize_bsdf = false
    );

    virtual Spectrum Li(const Ray& ray, const Medium* medium, Sampler& sampler) const override;

private:
    Spectrum SampleDirectLight(
        const Vec3& wo, const Intersection& isect, BSDF* bsdf, Sampler& sampler, const Spectrum& beta
    ) const;

    std::vector<Light*> infinite_lights;
    std::unordered_map<const Primitive*, AreaLight*> area_lights;
    PowerLightSampler light_sampler;

    int32 max_bounces;
    bool regularize_bsdf;
};

class NaiveVolPathIntegrator : public UniDirectionalRayIntegrator
{
public:
    NaiveVolPathIntegrator(const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler, int32 max_bounces);

    virtual Spectrum Li(const Ray& ray, const Medium* medium, Sampler& sampler) const override;

private:
    std::vector<Light*> infinite_lights;

    int32 max_bounces;
};

class VolPathIntegrator : public UniDirectionalRayIntegrator
{
public:
    VolPathIntegrator(
        const Intersectable* accel,
        std::vector<Light*> lights,
        const Sampler* sampler,
        int32 max_bounces,
        bool regularize_bsdf = false
    );

    virtual Spectrum Li(const Ray& ray, const Medium* medium, Sampler& sampler) const override;

private:
    Spectrum SampleDirectLight(
        const Vec3& wo,
        const Intersection& isect,
        const Medium* medium,
        const BSDF* bsdf,
        const PhaseFunction* phase,
        int32 wavelength,
        Sampler& sampler,
        Spectrum beta,
        Spectrum r_p
    ) const;

    std::vector<Light*> infinite_lights;
    std::unordered_map<const Primitive*, AreaLight*> area_lights;
    PowerLightSampler light_sampler;

    int32 max_bounces;
    bool regularize_bsdf;
};

// Light/Particle tracing integrator
class LightPathIntegrator : public BiDirectionalRayIntegrator
{
public:
    LightPathIntegrator(const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler, int32 max_bounces);

    virtual Spectrum L(const Ray& ray, const Medium* medium, const Camera* camera, Film& film, Sampler& sampler) const override;

private:
    bool V(const Point3 p1, const Point3 p2) const;

    PowerLightSampler light_sampler;
    int32 max_bounces;
};

class LightVolPathIntegrator : public BiDirectionalRayIntegrator
{
public:
    LightVolPathIntegrator(const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler, int32 max_bounces);

    virtual Spectrum L(const Ray& ray, const Medium* medium, const Camera* camera, Film& film, Sampler& sampler) const override;

private:
    Spectrum Tr(const Point3 p1, const Point3 p2, const Medium* medium, int32 wavelength) const;

    PowerLightSampler light_sampler;
    int32 max_bounces;
};

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

    Spectrum f(const Vertex& next, TransportDirection mode) const
    {
        Vec3 wi = next.point - point;
        if (wi.Normalize() == 0)
        {
            return Spectrum::black;
        }

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

class BiDirectionalPathIntegrator : public BiDirectionalRayIntegrator
{
public:
    BiDirectionalPathIntegrator(
        const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler, int32 max_bounces
    );

    virtual Spectrum L(const Ray& ray, const Medium* medium, const Camera* camera, Film& film, Sampler& sampler) const override;

private:
    bool V(const Point3 p1, const Point3 p2) const;
    int32 SampleCameraPath(Vertex* path, const Ray& ray, const Camera* camera, Sampler& sampler, Allocator& alloc) const;
    int32 SampleLightPath(Vertex* path, Sampler& sampler, Allocator& alloc) const;
    int32 RandomWalk(
        Vertex* path,
        Ray ray,
        Spectrum beta,
        Float pdf,
        int32 bounces,
        TransportDirection direction,
        Sampler& sampler,
        Allocator& alloc
    ) const;

    Spectrum ConnectPaths(
        Vertex* light_path, Vertex* camera_path, int32 s, int32 t, const Camera* camera, Film& film, Sampler& sampler
    ) const;

    std::vector<Light*> infinite_lights;
    std::unordered_map<const Primitive*, AreaLight*> area_lights;

    PowerLightSampler light_sampler;
    int32 max_bounces;
};
} // namespace bulbit
