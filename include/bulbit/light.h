#pragma once

#include "bounding_box.h"
#include "dynamic_dispatcher.h"
#include "ray.h"
#include "spectrum.h"

namespace bulbit
{

struct Intersection;
class Medium;

struct LightSampleLi
{
    LightSampleLi() = default;

    Point3 point;
    Vec3 normal;

    Vec3 wi;
    Float visibility;

    Spectrum Li;
    Float pdf;
};

struct LightSampleLe
{
    LightSampleLe() = default;

    Ray ray;
    Vec3 normal;
    Float pdf_p, pdf_w;
    Spectrum Le;
    const Medium* medium;
};

using Lights = TypePack<
    class PointLight,
    class SpotLight,
    class DirectionalLight,
    class DiffuseAreaLight,
    class DirectionalAreaLight,
    class UniformInfiniteLight,
    class ImageInfiniteLight>;

class Light : public DynamicDispatcher<Lights>
{
public:
    using Types = Lights;

protected:
    Light(int32 type_index)
        : DynamicDispatcher(type_index)
    {
    }

public:
    ~Light();

    void Preprocess(const AABB& world_bounds);

    Spectrum Le(const Intersection& isect, const Vec3& wo) const;
    Spectrum Le(const Ray& ray) const;

    bool Sample_Li(LightSampleLi* sample, const Intersection& ref, Point2 u) const;
    Float EvaluatePDF_Li(const Ray& ray) const;

    bool Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1) const;
    void EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const;
    void PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const;

    Spectrum Phi() const;

    bool IsDeltaLight() const;
    bool IsInfiniteLight() const;
};

} // namespace bulbit
