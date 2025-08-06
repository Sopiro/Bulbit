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

    Vec3 wo;
    Point3 point;
    Vec3 normal;
    Spectrum beta;
    bool delta;
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

    PowerLightSampler light_sampler;
    int32 max_bounces;
};
} // namespace bulbit
