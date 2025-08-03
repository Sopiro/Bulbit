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

    LightSampleLi(Vec3 wi, Float pdf, Float visibility, Spectrum Li)
        : wi{ wi }
        , pdf{ pdf }
        , visibility{ visibility }
        , Li{ Li }
    {
    }

    Vec3 wi;
    Float pdf;
    Float visibility;
    Spectrum Li;
};

struct LightSampleLe
{
    LightSampleLe() = default;

    LightSampleLe(Ray ray, Vec3 normal, Float pdf_p, Float pdf_w, Spectrum Le, const Medium* medium)
        : ray{ ray }
        , normal{ normal }
        , pdf_p{ pdf_p }
        , pdf_w{ pdf_w }
        , Le{ Le }
        , medium{ medium }
    {
    }

    Ray ray;
    Vec3 normal;
    Float pdf_p, pdf_w;
    Spectrum Le;
    const Medium* medium;
};

using Lights =
    TypePack<class PointLight, class DirectionalLight, class AreaLight, class UniformInfiniteLight, class ImageInfiniteLight>;

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

    Spectrum Le(const Ray& ray) const;

    bool Sample_Li(LightSampleLi* sample, const Intersection& ref, Point2 u) const;
    Float EvaluatePDF_Li(const Ray& ray) const;

    bool Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1) const;
    void EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const;
    void PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const;

    bool IsDeltaLight() const
    {
        return Is<PointLight>() || Is<DirectionalLight>();
    }
};

} // namespace bulbit
