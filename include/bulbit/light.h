#pragma once

#include "dynamic_dispatcher.h"
#include "ray.h"
#include "spectrum.h"

namespace bulbit
{

struct Intersection;

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

    Spectrum Le(const Ray& ray) const;

    LightSampleLi Sample_Li(const Intersection& ref, const Point2& u) const;
    Float EvaluatePDF_Li(const Ray& ray) const;

    bool IsDeltaLight() const;
};

} // namespace bulbit
