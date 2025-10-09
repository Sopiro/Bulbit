#pragma once

#include "scene.h"

namespace bulbit
{

class Rendering;
class Medium;
class Camera;
class Film;
class Sampler;
class LightSampler;

using AreaLightMap = std::unordered_map<const Primitive*, const Light*>;

struct IntegratorInfo;

class Integrator
{
public:
    static Integrator* Create(
        Allocator& alloc,
        const IntegratorInfo& integrator_info,
        const Intersectable* accel,
        const std::vector<Light*>& lights,
        const Sampler* sampler
    );

    virtual ~Integrator() = default;

    Integrator(Integrator&) = delete;
    Integrator& operator=(Integrator&) = delete;

    virtual Rendering* Render(Allocator& alloc, const Camera* camera) = 0;

    bool Intersect(Intersection* out_isect, const Ray& ray, Float t_min, Float t_max) const
    {
        return accel->Intersect(out_isect, ray, t_min, t_max);
    }

    bool IntersectAny(const Ray& ray, Float t_min, Float t_max) const
    {
        return accel->IntersectAny(ray, t_min, t_max);
    }

    const Intersectable* World() const
    {
        return accel;
    }

    const std::vector<Light*>& Lights() const
    {
        return all_lights;
    }

    const std::vector<Light*>& InfiniteLights() const
    {
        return infinite_lights;
    }

    const AreaLightMap& AreaLights() const
    {
        return area_lights;
    }

    const LightSampler* GetLightSampler() const
    {
        return light_sampler.get();
    }

protected:
    Integrator(const Intersectable* accel, std::vector<Light*> lights, std::unique_ptr<LightSampler> light_sampler);

    const Intersectable* accel;

    std::vector<Light*> all_lights;
    std::vector<Light*> infinite_lights;
    AreaLightMap area_lights;

    std::unique_ptr<LightSampler> light_sampler;
};

class UniDirectionalRayIntegrator : public Integrator
{
public:
    UniDirectionalRayIntegrator(
        const Intersectable* accel,
        std::vector<Light*> lights,
        const Sampler* sampler,
        std::unique_ptr<LightSampler> light_sampler
    );

    virtual Rendering* Render(Allocator& alloc, const Camera* camera) override;

    virtual Spectrum Li(const Ray& ray, const Medium* medium, Sampler& sampler) const = 0;

private:
    const Sampler* sampler_prototype;
};

class BiDirectionalRayIntegrator : public Integrator
{
public:
    BiDirectionalRayIntegrator(
        const Intersectable* accel,
        std::vector<Light*> lights,
        const Sampler* sampler,
        std::unique_ptr<LightSampler> light_sampler
    );

    virtual Rendering* Render(Allocator& alloc, const Camera* camera) override;

    virtual Spectrum L(const Ray& ray, const Medium* medium, const Camera* camera, Film& film, Sampler& sampler) const = 0;

private:
    const Sampler* sampler_prototype;
};

} // namespace bulbit
