#pragma once

#include "bxdf.h"
#include "integrator.h"
#include "light_samplers.h"
#include "lights.h"
#include "photon.h"

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
    int32 max_bounces;
};

class AOIntegrator : public UniDirectionalRayIntegrator
{
public:
    AOIntegrator(const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler, Float ao_range);

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

    int32 max_depth;
};

class NaivePathIntegrator : public UniDirectionalRayIntegrator
{
public:
    NaivePathIntegrator(
        const Intersectable* accel,
        std::vector<Light*> lights,
        const Sampler* sampler,
        int32 max_bounces,
        int32 rr_min_bounces = 1
    );

    virtual Spectrum Li(const Ray& ray, const Medium* medium, Sampler& sampler) const override;

private:
    Spectrum Li(const Ray& ray, Sampler& sampler, int32 depth) const;

    int32 max_bounces;
    int32 rr_min_bounces;
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
        int32 rr_min_bounces = 1,
        bool regularize_bsdf = false
    );

    virtual Spectrum Li(const Ray& ray, const Medium* medium, Sampler& sampler) const override;

private:
    Spectrum SampleDirectLight(
        const Vec3& wo, const Intersection& isect, BSDF* bsdf, Sampler& sampler, const Spectrum& beta
    ) const;

    int32 max_bounces;
    int32 rr_min_bounces;
    bool regularize_bsdf;
};

class NaiveVolPathIntegrator : public UniDirectionalRayIntegrator
{
public:
    NaiveVolPathIntegrator(
        const Intersectable* accel,
        std::vector<Light*> lights,
        const Sampler* sampler,
        int32 max_bounces,
        int32 rr_min_bounces = 1
    );

    virtual Spectrum Li(const Ray& ray, const Medium* medium, Sampler& sampler) const override;

private:
    int32 max_bounces;
    int32 rr_min_bounces;
};

class VolPathIntegrator : public UniDirectionalRayIntegrator
{
public:
    VolPathIntegrator(
        const Intersectable* accel,
        std::vector<Light*> lights,
        const Sampler* sampler,
        int32 max_bounces,
        int32 rr_min_bounces = 1,
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

    int32 max_bounces;
    int32 rr_min_bounces;
    bool regularize_bsdf;
};

// Light/Particle tracing integrator
class LightPathIntegrator : public BiDirectionalRayIntegrator
{
public:
    LightPathIntegrator(
        const Intersectable* accel,
        std::vector<Light*> lights,
        const Sampler* sampler,
        int32 max_bounces,
        int32 rr_min_bounces = 1
    );

    virtual Spectrum L(const Ray& ray, const Medium* medium, const Camera* camera, Film& film, Sampler& sampler) const override;

private:
    int32 max_bounces;
    int32 rr_min_bounces;
};

class LightVolPathIntegrator : public BiDirectionalRayIntegrator
{
public:
    LightVolPathIntegrator(
        const Intersectable* accel,
        std::vector<Light*> lights,
        const Sampler* sampler,
        int32 max_bounces,
        int32 rr_min_bounces = 1
    );

    virtual Spectrum L(const Ray& ray, const Medium* medium, const Camera* camera, Film& film, Sampler& sampler) const override;

private:
    int32 max_bounces;
    int32 rr_min_bounces;
};

struct Vertex;

class BiDirectionalPathIntegrator : public BiDirectionalRayIntegrator
{
public:
    BiDirectionalPathIntegrator(
        const Intersectable* accel,
        std::vector<Light*> lights,
        const Sampler* sampler,
        int32 max_bounces,
        int32 rr_min_bounces = 1
    );

    virtual Spectrum L(const Ray& ray, const Medium* medium, const Camera* camera, Film& film, Sampler& sampler) const override;

private:
    int32 SampleCameraPath(Vertex* path, const Ray& ray, const Camera* camera, Sampler& sampler, Allocator& alloc) const;
    int32 SampleLightPath(Vertex* path, Sampler& sampler, Allocator& alloc) const;

    int32 max_bounces;
    int32 rr_min_bounces;
};

class BiDirectionalVolPathIntegrator : public BiDirectionalRayIntegrator
{
public:
    BiDirectionalVolPathIntegrator(
        const Intersectable* accel,
        std::vector<Light*> lights,
        const Sampler* sampler,
        int32 max_bounces,
        int32 rr_min_bounces = 1
    );

    virtual Spectrum L(const Ray& ray, const Medium* medium, const Camera* camera, Film& film, Sampler& sampler) const override;

private:
    int32 SampleCameraPath(
        Vertex* path,
        const Ray& ray,
        const Medium* medium,
        const Camera* camera,
        int32 wavelength,
        Sampler& sampler,
        Allocator& alloc
    ) const;
    int32 SampleLightPath(Vertex* path, int32 wavelength, Sampler& sampler, Allocator& alloc) const;

    int32 max_bounces;
    int32 rr_min_bounces;
};

class MultiPhaseRendering;

class PhotonMappingIntegrator : public Integrator
{
public:
    PhotonMappingIntegrator(
        const Intersectable* accel,
        std::vector<Light*> lights,
        const Sampler* sampler,
        int32 max_bounces,
        int32 n_photons,
        Float gather_radius = -1,
        bool sample_direct_light = true
    );

    virtual Rendering* Render(Allocator& alloc, const Camera* camera) override;

private:
    void EmitPhotons(MultiPhaseRendering* progress);
    void GatherPhotons(const Camera* camera, int32 tile_size, MultiPhaseRendering* progress);

    Spectrum SampleDirectLight(
        const Vec3& wo, const Intersection& isect, const BSDF* bsdf, Sampler& sampler, const Spectrum& beta
    ) const;
    Spectrum Li(const Ray& ray, const Medium* medium, Sampler& sampler) const;

    const Sampler* sampler_prototype;
    int32 max_bounces;

    int32 n_photons;
    Float gather_radius;
    bool sample_dl;

    std::vector<Photon> photons;
    HashGrid photon_map;
};

class VolPhotonMappingIntegrator : public Integrator
{
public:
    VolPhotonMappingIntegrator(
        const Intersectable* accel,
        std::vector<Light*> lights,
        const Sampler* sampler,
        int32 max_bounces,
        int32 n_photons,
        Float gather_radius_surface = -1,
        Float gather_radius_volume = -1,
        bool sample_direct_light = true
    );

    virtual Rendering* Render(Allocator& alloc, const Camera* camera) override;

private:
    void EmitPhotons(MultiPhaseRendering* progress);
    void GatherPhotons(const Camera* camera, int32 tile_size, MultiPhaseRendering* progress);

    Spectrum SampleDirectLight(
        const Vec3& wo,
        const Intersection& isect,
        const Medium* medium,
        const BSDF* bsdf,
        const PhaseFunction* phase,
        int32 wavelength,
        Sampler& sampler,
        const Spectrum& beta,
        Spectrum r_p
    ) const;

    Spectrum Li(const Ray& ray, const Medium* medium, Sampler& sampler) const;

    const Sampler* sampler_prototype;
    int32 max_bounces;

    int32 n_photons;
    Float radius, vol_radius;
    bool sample_dl;

    std::vector<Photon> photons, vol_photons;
    HashGrid photon_map, vol_photon_map;
};

// Stochastic Progressive Photon Mapping
class SPPMIntegrator : public Integrator
{
public:
    SPPMIntegrator(
        const Intersectable* accel,
        std::vector<Light*> lights,
        const Sampler* sampler,
        int32 max_bounces,
        int32 photons_per_interation,
        Float initial_radius = -1
    );

    virtual Rendering* Render(Allocator& alloc, const Camera* camera) override;

private:
    Spectrum SampleDirectLight(
        const Vec3& wo, const Intersection& isect, BSDF* bsdf, Sampler& sampler, const Spectrum& beta
    ) const;

    const Sampler* sampler_prototype;
    int32 max_bounces;

    int32 photons_per_iteration;
    Float initial_radius;
};

// Volumetric SPPM
class VolSPPMIntegrator : public Integrator
{
public:
    VolSPPMIntegrator(
        const Intersectable* accel,
        std::vector<Light*> lights,
        const Sampler* sampler,
        int32 max_bounces,
        int32 photons_per_interation,
        Float initial_radius_surface = -1,
        Float initial_radius_volume = -1
    );

    virtual Rendering* Render(Allocator& alloc, const Camera* camera) override;

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

    const Sampler* sampler_prototype;
    int32 max_bounces;

    int32 photons_per_iteration;
    Float initial_radius_surface, initial_radius_volume;
};

} // namespace bulbit
