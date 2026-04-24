#pragma once

#include "bxdf.h"
#include "common.h"
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

    virtual SpectrumSample Li(const Ray& ray, const Medium* medium, WavelengthSample& lambda, Sampler& sampler) const override;
};

class RandomWalkIntegrator : public UniDirectionalRayIntegrator
{
public:
    RandomWalkIntegrator(const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler, int32 max_bounces);

    virtual SpectrumSample Li(const Ray& ray, const Medium* medium, WavelengthSample& lambda, Sampler& sampler) const override;

private:
    int32 max_bounces;
};

class AOIntegrator : public UniDirectionalRayIntegrator
{
public:
    AOIntegrator(const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler, Float ao_range);

    virtual SpectrumSample Li(const Ray& ray, const Medium* medium, WavelengthSample& lambda, Sampler& sampler) const override;

private:
    Float range;
};

class AlbedoIntegrator : public UniDirectionalRayIntegrator
{
public:
    AlbedoIntegrator(const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler);

    virtual SpectrumSample Li(const Ray& ray, const Medium* medium, WavelengthSample& lambda, Sampler& sampler) const override;
};

class WhittedStyle : public UniDirectionalRayIntegrator
{
public:
    WhittedStyle(const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler, int32 max_depth);

    virtual SpectrumSample Li(const Ray& ray, const Medium* medium, WavelengthSample& lambda, Sampler& sampler) const override
    {
        BulbitNotUsed(medium);
        return Li(ray, lambda, sampler, 0);
    }

private:
    SpectrumSample Li(const Ray& ray, WavelengthSample& lambda, Sampler& sampler, int32 depth) const;

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

    virtual SpectrumSample Li(const Ray& ray, const Medium* medium, WavelengthSample& lambda, Sampler& sampler) const override;

private:
    SpectrumSample Li(const Ray& ray, WavelengthSample& lambda, Sampler& sampler, int32 depth) const;

    int32 max_bounces;
    int32 rr_min_bounces;
};

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

    virtual SpectrumSample Li(const Ray& ray, const Medium* medium, WavelengthSample& lambda, Sampler& sampler) const override;

private:
    SpectrumSample SampleDirectLight(
        const Vec3& wo,
        const Intersection& isect,
        BSDF* bsdf,
        const WavelengthSample& lambda,
        Sampler& sampler,
        const SpectrumSample& beta
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

    virtual SpectrumSample Li(const Ray& ray, const Medium* medium, WavelengthSample& lambda, Sampler& sampler) const override;

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

    virtual SpectrumSample Li(const Ray& ray, const Medium* medium, WavelengthSample& lambda, Sampler& sampler) const override;

private:
    SpectrumSample SampleDirectLight(
        const Vec3& wo,
        const Intersection& isect,
        const Medium* medium,
        const BSDF* bsdf,
        const PhaseFunction* phase,
        const WavelengthSample& lambda,
        Sampler& sampler,
        SpectrumSample beta,
        SpectrumSample r_p
    ) const;

    int32 max_bounces;
    int32 rr_min_bounces;
    bool regularize_bsdf;
};

class ReSTIRDIIntegrator : public Integrator
{
public:
    ReSTIRDIIntegrator(
        const Intersectable* accel,
        std::vector<Light*> lights,
        const Sampler* sampler,
        Float spatial_radius = 5.0f,
        int32 spatial_samples = 5,
        int32 M_light = 16,
        int32 M_bsdf = 1,
        bool include_visibility = false
    );

    virtual Rendering* Render(Allocator& alloc, const Camera* camera) override;

private:
    const Sampler* sampler_prototype;

    Float spatial_radius;
    int32 num_spatial_samples;

    int32 M_light;
    int32 M_bsdf;
    bool include_visibility;
};

class ReSTIRPTIntegrator : public Integrator
{
public:
    ReSTIRPTIntegrator(
        const Intersectable* accel,
        std::vector<Light*> lights,
        const Sampler* sampler,
        int32 max_bounces,
        int32 rr_min_bounces = 1,
        Float spatial_radius = 10.0f,
        int32 spatial_samples = 10
    );

    virtual Rendering* Render(Allocator& alloc, const Camera* camera) override;

private:
    const Sampler* sampler_prototype;
    int32 max_bounces;
    int32 rr_min_bounces;

    Float spatial_radius;
    int32 num_spatial_samples;
};

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

    virtual SpectrumSample L(
        const Ray& ray, const Medium* medium, WavelengthSample& lambda, const Camera* camera, Film& film, Sampler& sampler
    ) const override;

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

    virtual SpectrumSample L(
        const Ray& ray, const Medium* medium, WavelengthSample& lambda, const Camera* camera, Film& film, Sampler& sampler
    ) const override;

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

    virtual SpectrumSample L(
        const Ray& ray, const Medium* medium, WavelengthSample& lambda, const Camera* camera, Film& film, Sampler& sampler
    ) const override;

private:
    int32 SampleCameraPath(
        Vertex* path, const Ray& ray, const Camera* camera, WavelengthSample& lambda, Sampler& sampler, Allocator& alloc
    ) const;
    int32 SampleLightPath(Vertex* path, WavelengthSample& lambda, Sampler& sampler, Allocator& alloc) const;

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

    virtual SpectrumSample L(
        const Ray& ray, const Medium* medium, WavelengthSample& lambda, const Camera* camera, Film& film, Sampler& sampler
    ) const override;

private:
    int32 SampleCameraPath(
        Vertex* path,
        const Ray& ray,
        const Medium* medium,
        const Camera* camera,
        WavelengthSample& lambda,
        Sampler& sampler,
        Allocator& alloc
    ) const;
    int32 SampleLightPath(Vertex* path, WavelengthSample& lambda, Sampler& sampler, Allocator& alloc) const;

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
    void EmitPhotons(MultiPhaseRendering* progress, WavelengthSample lambda, int32 phase_index);
    void GatherPhotons(
        const Camera* camera, WavelengthSample lambda, int32 tile_size, MultiPhaseRendering* progress, int32 phase_index
    );

    SpectrumSample SampleDirectLight(
        const Vec3& wo,
        const Intersection& isect,
        const BSDF* bsdf,
        const WavelengthSample& lambda,
        Sampler& sampler,
        const SpectrumSample& beta
    ) const;
    Vec3 Li(const Ray& ray, WavelengthSample& lambda, Sampler& sampler) const;

    const Sampler* sampler_prototype;
    int32 max_bounces;

    int32 n_photons;
    Float gather_radius;
    bool sample_direct_light;

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
    void EmitPhotons(MultiPhaseRendering* progress, WavelengthSample lambda, int32 phase_index);
    void GatherPhotons(
        const Camera* camera, WavelengthSample lambda, int32 tile_size, MultiPhaseRendering* progress, int32 phase_index
    );

    SpectrumSample SampleDirectLight(
        const Vec3& wo,
        const Intersection& isect,
        const Medium* medium,
        const BSDF* bsdf,
        const PhaseFunction* phase,
        const WavelengthSample& lambda,
        Sampler& sampler,
        const SpectrumSample& beta,
        SpectrumSample r_p
    ) const;

    Vec3 Li(const Ray& ray, const Medium* medium, WavelengthSample& lambda, Sampler& sampler) const;

    const Sampler* sampler_prototype;
    int32 max_bounces;

    int32 n_photons;
    Float radius, vol_radius;
    bool sample_direct_light;

    std::vector<Photon> photons, vol_photons;
    HashGrid photon_map, vol_photon_map;
};

class SPPMIntegrator : public Integrator
{
public:
    SPPMIntegrator(
        const Intersectable* accel,
        std::vector<Light*> lights,
        const Sampler* sampler,
        int32 max_bounces,
        int32 photons_per_interation,
        Float initial_radius = -1,
        bool sample_direct_light = true
    );

    virtual Rendering* Render(Allocator& alloc, const Camera* camera) override;

private:
    SpectrumSample SampleDirectLight(
        const Vec3& wo,
        const Intersection& isect,
        BSDF* bsdf,
        const WavelengthSample& lambda,
        Sampler& sampler,
        const SpectrumSample& beta
    ) const;

    const Sampler* sampler_prototype;
    int32 max_bounces;

    int32 photons_per_iteration;
    Float initial_radius;
    bool sample_direct_light;
};

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
        Float initial_radius_volume = -1,
        bool sample_direct_light = true
    );

    virtual Rendering* Render(Allocator& alloc, const Camera* camera) override;

private:
    SpectrumSample SampleDirectLight(
        const Vec3& wo,
        const Intersection& isect,
        const Medium* medium,
        const BSDF* bsdf,
        const PhaseFunction* phase,
        const WavelengthSample& lambda,
        Sampler& sampler,
        SpectrumSample beta,
        SpectrumSample r_p
    ) const;

    const Sampler* sampler_prototype;
    int32 max_bounces;

    int32 photons_per_iteration;
    Float initial_radius_surface, initial_radius_volume;
    bool sample_direct_light;
};

class VCMIntegrator : public Integrator
{
public:
    VCMIntegrator(
        const Intersectable* accel,
        std::vector<Light*> lights,
        const Sampler* sampler,
        int32 max_bounces,
        int32 rr_min_bounces = 1,
        Float initial_radius = -1,
        Float radius_alpha = 0.75f
    );

    virtual Rendering* Render(Allocator& alloc, const Camera* camera) override;

    const Sampler* sampler_prototype;
    int32 max_bounces;
    int32 rr_min_bounces;

    Float initial_radius;
    Float radius_alpha;
};

} // namespace bulbit
