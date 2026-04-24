#pragma once

#include "intersectable.h"
#include "light.h"
#include "sampling.h"
#include "spectrum.h"
#include "textures.h"
#include "transform.h"

namespace bulbit
{

class Primitive;

class PointLight : public Light
{
public:
    PointLight(const Point3& position, const Spectrum& intensity, const Medium* medium);
    void Destroy() {}

    void Preprocess(const AABB& world_bounds);

    SpectrumSample Le(const Intersection& isect, const Vec3& wo, const WavelengthSample& lambda) const;
    SpectrumSample Le(const Ray& ray, const WavelengthSample& lambda) const;

    bool Sample_Li(LightSampleLi* sample, const Intersection& ref, Point2 u, const WavelengthSample& lambda) const;
    Float EvaluatePDF_Li(const Ray& ray) const;

    bool Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1, const WavelengthSample& lambda) const;
    void EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const;
    void PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const;

    Float Power() const;

private:
    Point3 p;
    Spectrum intensity; // radiance
    const Medium* medium;
};

class SpotLight : public Light
{
public:
    SpotLight(
        const Point3& position,
        const Vec3& direction,     // Incident direction
        const Spectrum& intensity,
        Float angle_max,           // degrees
        Float angle_falloff_start, // degrees
        const Medium* medium
    );
    void Destroy() {}

    void Preprocess(const AABB& world_bounds);

    SpectrumSample Le(const Intersection& isect, const Vec3& wo, const WavelengthSample& lambda) const;
    SpectrumSample Le(const Ray& ray, const WavelengthSample& lambda) const;

    bool Sample_Li(LightSampleLi* sample, const Intersection& ref, Point2 u, const WavelengthSample& lambda) const;
    Float EvaluatePDF_Li(const Ray& ray) const;

    bool Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1, const WavelengthSample& lambda) const;
    void EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const;
    void PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const;

    Float Power() const;

private:
    Point3 p;
    Frame frame;
    Float cos_theta_min, cos_theta_max;

    Spectrum intensity; // radiance
    const Medium* medium;
};

class DirectionalLight : public Light
{
public:
    DirectionalLight(
        const Vec3& direction, // Incident direction
        const Spectrum& intensity
    );
    void Destroy() {}

    void Preprocess(const AABB& world_bounds);

    SpectrumSample Le(const Intersection& isect, const Vec3& wo, const WavelengthSample& lambda) const;
    SpectrumSample Le(const Ray& ray, const WavelengthSample& lambda) const;

    bool Sample_Li(LightSampleLi* sample, const Intersection& ref, Point2 u, const WavelengthSample& lambda) const;
    Float EvaluatePDF_Li(const Ray& ray) const;

    bool Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1, const WavelengthSample& lambda) const;
    void EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const;
    void PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const;

    Float Power() const;

private:
    Vec3 w;
    Spectrum intensity; // radiance

    Point3 world_center;
    Float world_radius;
};

class DiffuseAreaLight : public Light
{
public:
    DiffuseAreaLight(const Primitive* primitive, const SpectrumTexture* emission, Float emission_mean_luminance, bool two_sided);
    void Destroy() {}

    void Preprocess(const AABB& world_bounds);

    SpectrumSample Le(const Intersection& isect, const Vec3& wo, const WavelengthSample& lambda) const;
    SpectrumSample Le(const Ray& ray, const WavelengthSample& lambda) const;

    bool Sample_Li(LightSampleLi* sample, const Intersection& ref, Point2 u, const WavelengthSample& lambda) const;
    Float EvaluatePDF_Li(const Ray& ray) const;

    bool Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1, const WavelengthSample& lambda) const;
    void EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const;
    void PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const;

    Float Power() const;

    const Primitive* primitive;

private:
    const SpectrumTexture* emission;
    Float emission_mean_luminance;
    bool two_sided;
};

class DirectionalAreaLight : public Light
{
    // This light can only be rendered correctly with integrators that support particle traced light transport,
    // such as LightPathIntegrator or BidirectionalPathIntegrator,
    // and also will not rendered correctly with photon mapping based integrators
    // that explicitly sample direct illumination, since it cannot be sampled directly.
public:
    DirectionalAreaLight(
        const Primitive* primitive, const SpectrumTexture* emission, Float emission_mean_luminance, bool two_sided
    );
    void Destroy() {}

    void Preprocess(const AABB& world_bounds);

    SpectrumSample Le(const Intersection& isect, const Vec3& wo, const WavelengthSample& lambda) const;
    SpectrumSample Le(const Ray& ray, const WavelengthSample& lambda) const;

    bool Sample_Li(LightSampleLi* sample, const Intersection& ref, Point2 u, const WavelengthSample& lambda) const;
    Float EvaluatePDF_Li(const Ray& ray) const;

    bool Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1, const WavelengthSample& lambda) const;
    void PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const;

    Float Power() const;

    const Primitive* primitive;

private:
    const SpectrumTexture* emission;
    Float emission_mean_luminance;
    bool two_sided;
};

class SpotAreaLight : public Light
{
    // Spot area light emits light from a surface primitive within a cone.
    // Each point on the primitive behaves like a spot light with
    // angular falloff defined by angle_max and angle_falloff_start.
public:
    SpotAreaLight(
        const Primitive* primitive,
        const SpectrumTexture* emission,
        Float emission_mean_luminance,
        Float angle_max,           // degrees
        Float angle_falloff_start, // degrees
        bool two_sided = false
    );

    void Destroy() {}

    void Preprocess(const AABB& world_bounds);

    SpectrumSample Le(const Intersection& isect, const Vec3& wo, const WavelengthSample& lambda) const;
    SpectrumSample Le(const Ray& ray, const WavelengthSample& lambda) const;

    bool Sample_Li(LightSampleLi* sample, const Intersection& ref, Point2 u, const WavelengthSample& lambda) const;
    Float EvaluatePDF_Li(const Ray& ray) const;

    bool Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1, const WavelengthSample& lambda) const;
    void EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const;
    void PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const;

    Float Power() const;

    const Primitive* primitive;

private:
    const SpectrumTexture* emission;
    Float emission_mean_luminance;

    Float cos_theta_min, cos_theta_max;
    bool two_sided;
};

class ImageInfiniteLight : public Light
{
public:
    ImageInfiniteLight(const SpectrumImageTexture* l_map, const Transform& transform = identity, Float l_scale = 1);
    void Destroy();

    void Preprocess(const AABB& world_bounds);

    SpectrumSample Le(const Intersection& isect, const Vec3& wo, const WavelengthSample& lambda) const;
    SpectrumSample Le(const Ray& ray, const WavelengthSample& lambda) const;

    bool Sample_Li(LightSampleLi* sample, const Intersection& ref, Point2 u, const WavelengthSample& lambda) const;
    Float EvaluatePDF_Li(const Ray& ray) const;

    bool Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1, const WavelengthSample& lambda) const;
    void EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const;
    void PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const;

    Float Power() const;

private:
    const SpectrumImageTexture* l_map; // Environment(Radiance) map
    Float l_scale;
    Float l_map_mean_luminance;

    std::unique_ptr<Distribution2D> distribution;

    Transform transform;

    Point3 world_center;
    Float world_radius;
};

class UniformInfiniteLight : public Light
{
public:
    UniformInfiniteLight(const Spectrum& l, Float scale = 1);
    void Destroy() {}

    void Preprocess(const AABB& world_bounds);

    SpectrumSample Le(const Intersection& isect, const Vec3& wo, const WavelengthSample& lambda) const;
    SpectrumSample Le(const Ray& ray, const WavelengthSample& lambda) const;

    bool Sample_Li(LightSampleLi* sample, const Intersection& ref, Point2 u, const WavelengthSample& lambda) const;
    Float EvaluatePDF_Li(const Ray& ray) const;

    bool Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1, const WavelengthSample& lambda) const;
    void EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const;
    void PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const;

    Float Power() const;

private:
    Spectrum l;
    Float scale;

    Point3 world_center;
    Float world_radius;
};

inline Light::~Light()
{
    Dispatch([&](auto light) { light->Destroy(); });
}

inline void Light::Preprocess(const AABB& world_bounds)
{
    Dispatch([&](auto light) { light->Preprocess(world_bounds); });
}

inline SpectrumSample Light::Le(const Intersection& isect, const Vec3& wo, const WavelengthSample& lambda) const
{
    return Dispatch([&](auto light) { return light->Le(isect, wo, lambda); });
}

inline SpectrumSample Light::Le(const Ray& ray, const WavelengthSample& lambda) const
{
    return Dispatch([&](auto light) { return light->Le(ray, lambda); });
}

inline bool Light::Sample_Li(LightSampleLi* sample, const Intersection& ref, Point2 u, const WavelengthSample& lambda) const
{
    return Dispatch([&](auto light) { return light->Sample_Li(sample, ref, u, lambda); });
}

inline Float Light::EvaluatePDF_Li(const Ray& ray) const
{
    return Dispatch([&](auto light) { return light->EvaluatePDF_Li(ray); });
}

inline bool Light::Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1, const WavelengthSample& lambda) const
{
    return Dispatch([&](auto light) { return light->Sample_Le(sample, u0, u1, lambda); });
}

inline void Light::EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const
{
    Dispatch([&](auto light) { light->EvaluatePDF_Le(pdf_p, pdf_w, ray); });
}

inline void Light::PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const
{
    Dispatch([&](auto light) { light->PDF_Le(pdf_p, pdf_w, isect, w); });
}

inline Float Light::Power() const
{
    return Dispatch([&](auto light) { return light->Power(); });
}

inline bool Light::IsDeltaLight() const
{
    return Is<PointLight>() || Is<SpotLight>() || Is<DirectionalLight>() || Is<DirectionalAreaLight>();
}

inline bool Light::IsInfiniteLight() const
{
    return Is<UniformInfiniteLight>() || Is<ImageInfiniteLight>();
}

} // namespace bulbit
