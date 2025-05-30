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
    PointLight(const Point3& position, const Spectrum& intensity);
    void Destroy() {}

    Spectrum Le(const Ray& ray) const;

    LightSampleLi Sample_Li(const Intersection& ref, Point2 u) const;
    Float EvaluatePDF_Li(const Ray& ray) const;

    LightSampleLe Sample_Le(Point2 u0, Point2 u1) const;
    void EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const;
    void PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const;

private:
    Point3 position;
    Spectrum intensity; // radiance
};

class DirectionalLight : public Light
{
public:
    DirectionalLight(const Vec3& direction, const Spectrum& intensity, Float visible_radius);
    void Destroy() {}

    Spectrum Le(const Ray& ray) const;

    LightSampleLi Sample_Li(const Intersection& ref, Point2 u) const;
    Float EvaluatePDF_Li(const Ray& ray) const;

    LightSampleLe Sample_Le(Point2 u0, Point2 u1) const;
    void EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const;
    void PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const;

private:
    Vec3 dir;
    Spectrum intensity; // radiance
    Float radius;       // visible radius
};

class AreaLight : public Light
{
public:
    AreaLight(const Primitive* primitive, bool two_sided);
    void Destroy() {}

    Spectrum Le(const Ray& ray) const;

    LightSampleLi Sample_Li(const Intersection& ref, Point2 u) const;
    Float EvaluatePDF_Li(const Ray& ray) const;

    LightSampleLe Sample_Le(Point2 u0, Point2 u1) const;
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

    Spectrum Le(const Ray& ray) const;

    LightSampleLi Sample_Li(const Intersection& ref, Point2 u) const;
    Float EvaluatePDF_Li(const Ray& ray) const;

    LightSampleLe Sample_Le(Point2 u0, Point2 u1) const;
    void EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const;
    void PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const;

private:
    const SpectrumImageTexture* l_map; // Environment(Radiance) map
    Float l_scale;

    std::unique_ptr<Distribution2D> distribution;

    Transform transform;
};

class UniformInfiniteLight : public Light
{
public:
    UniformInfiniteLight(const Spectrum& l, Float scale = 1);
    void Destroy() {}

    Spectrum Le(const Ray& ray) const;

    LightSampleLi Sample_Li(const Intersection& ref, Point2 u) const;
    Float EvaluatePDF_Li(const Ray& ray) const;

    LightSampleLe Sample_Le(Point2 u0, Point2 u1) const;
    void EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const;
    void PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const;

private:
    Spectrum l;
    Float scale;
};

inline Light::~Light()
{
    Dispatch([&](auto light) { light->Destroy(); });
}

inline Spectrum Light::Le(const Ray& ray) const
{
    return Dispatch([&](auto light) { return light->Le(ray); });
}

inline LightSampleLi Light::Sample_Li(const Intersection& ref, Point2 u) const
{
    return Dispatch([&](auto light) { return light->Sample_Li(ref, u); });
}

inline Float Light::EvaluatePDF_Li(const Ray& ray) const
{
    return Dispatch([&](auto light) { return light->EvaluatePDF_Li(ray); });
}

inline LightSampleLe Light::Sample_Le(Point2 u0, Point2 u1) const
{
    return Dispatch([&](auto light) { return light->Sample_Le(u0, u1); });
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
