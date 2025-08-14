#pragma once

#include "scene.h"

namespace bulbit
{

class Rendering;
class Medium;
class Camera;
class Film;
class Sampler;

class Integrator
{
public:
    virtual ~Integrator() = default;

    virtual std::unique_ptr<Rendering> Render(const Camera* camera) = 0;

protected:
    Integrator(const Intersectable* accel, std::vector<Light*> lights);

    bool V(const Point3 p1, const Point3 p2) const;
    Spectrum Tr(const Point3 p1, const Point3 p2, const Medium* medium, int32 wavelength) const;

    bool Intersect(Intersection* out_isect, const Ray& ray, Float t_min, Float t_max) const
    {
        return accel->Intersect(out_isect, ray, t_min, t_max);
    }

    bool IntersectAny(const Ray& ray, Float t_min, Float t_max) const
    {
        return accel->IntersectAny(ray, t_min, t_max);
    }

    const Intersectable* accel;

    std::vector<Light*> all_lights;
    std::vector<Light*> infinite_lights;
    std::unordered_map<const Primitive*, AreaLight*> area_lights;
};

class UniDirectionalRayIntegrator : public Integrator
{
public:
    UniDirectionalRayIntegrator(const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler);

    virtual std::unique_ptr<Rendering> Render(const Camera* camera) override;

    virtual Spectrum Li(const Ray& ray, const Medium* medium, Sampler& sampler) const = 0;

private:
    const Sampler* sampler_prototype;
};

class BiDirectionalRayIntegrator : public Integrator
{
public:
    BiDirectionalRayIntegrator(const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler);

    virtual std::unique_ptr<Rendering> Render(const Camera* camera) override;

    virtual Spectrum L(const Ray& ray, const Medium* medium, const Camera* camera, Film& film, Sampler& sampler) const = 0;

private:
    const Sampler* sampler_prototype;
};

} // namespace bulbit
