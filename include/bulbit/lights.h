#pragma once

#include "intersectable.h"
#include "light.h"
#include "primitive.h"
#include "spectrum.h"
#include "textures.h"
#include "transform.h"

namespace bulbit
{

class PointLight : public Light
{
public:
    PointLight(const Point3& position, const Spectrum& intensity, const Medium* medium);

    void Destroy() {}
    void Preprocess(const AABB& world_bounds);

    Spectrum Le(const Ray& ray) const;

    bool Sample_Li(LightSampleLi* sample, const Intersection& ref, Point2 u) const;
    Float EvaluatePDF_Li(const Ray& ray) const;

    bool Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1) const;
    void EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const;
    void PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const;

private:
    Point3 position;
    Spectrum intensity; // radiance
    const Medium* medium;
};

class DirectionalLight : public Light
{
public:
    DirectionalLight(const Vec3& direction, const Spectrum& intensity);

    void Destroy() {}
    void Preprocess(const AABB& world_bounds);

    Spectrum Le(const Ray& ray) const;

    bool Sample_Li(LightSampleLi* sample, const Intersection& ref, Point2 u) const;
    Float EvaluatePDF_Li(const Ray& ray) const;

    bool Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1) const;
    void EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const;
    void PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const;

private:
    Vec3 dir;
    Spectrum intensity; // radiance
    Float visible_radius;

    Point3 world_center;
    Float world_radius;
};

class AreaLight : public Light
{
public:
    AreaLight(const Primitive* primitive, bool two_sided);

    void Destroy() {}
    void Preprocess(const AABB& world_bounds);

    Spectrum Le(const Ray& ray) const;

    bool Sample_Li(LightSampleLi* sample, const Intersection& ref, Point2 u) const;
    Float EvaluatePDF_Li(const Ray& ray) const;

    bool Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1) const;
    void EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const;
    void PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const;

    const Primitive* GetPrimitive() const
    {
        return primitive;
    }

private:
    const Primitive* primitive;
    bool two_sided;
};

class ImageInfiniteLight : public Light
{
public:
    ImageInfiniteLight(const SpectrumImageTexture* l_map, const Transform& transform = identity, Float l_scale = 1);

    void Destroy();
    void Preprocess(const AABB& world_bounds);

    Spectrum Le(const Ray& ray) const;

    bool Sample_Li(LightSampleLi* sample, const Intersection& ref, Point2 u) const;
    Float EvaluatePDF_Li(const Ray& ray) const;

    bool Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1) const;
    void EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const;
    void PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const;

private:
    const SpectrumImageTexture* l_map; // Environment(Radiance) map
    Float l_scale;

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

    Spectrum Le(const Ray& ray) const;

    bool Sample_Li(LightSampleLi* sample, const Intersection& ref, Point2 u) const;
    Float EvaluatePDF_Li(const Ray& ray) const;

    bool Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1) const;
    void EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const;
    void PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const;

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

inline Spectrum Light::Le(const Ray& ray) const
{
    return Dispatch([&](auto light) { return light->Le(ray); });
}

inline bool Light::Sample_Li(LightSampleLi* sample, const Intersection& ref, Point2 u) const
{
    return Dispatch([&](auto light) { return light->Sample_Li(sample, ref, u); });
}

inline Float Light::EvaluatePDF_Li(const Ray& ray) const
{
    return Dispatch([&](auto light) { return light->EvaluatePDF_Li(ray); });
}

inline bool Light::Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1) const
{
    return Dispatch([&](auto light) { return light->Sample_Le(sample, u0, u1); });
}

inline void Light::EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const
{
    Dispatch([&](auto light) { light->EvaluatePDF_Le(pdf_p, pdf_w, ray); });
}

inline void Light::PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const
{
    Dispatch([&](auto light) { light->PDF_Le(pdf_p, pdf_w, isect, w); });
}

} // namespace bulbit
