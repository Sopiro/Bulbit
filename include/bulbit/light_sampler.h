#pragma once

#include "common.h"
#include "math.h"

namespace bulbit
{

struct Intersection;
class Light;

struct SampledLight
{
    const Light* light;
    Float pmf;
};

class LightSampler
{
public:
    LightSampler() = default;
    virtual ~LightSampler() = default;

    virtual void Init(std::span<Light*> all_lights);

    virtual bool Sample(SampledLight* sampled_light, const Intersection& isect, Float u) const = 0;
    virtual Float EvaluatePMF(const Light* light) const = 0;

protected:
    std::span<Light*> lights;
};

inline void LightSampler::Init(std::span<Light*> all_lights)
{
    lights = all_lights;
}

} // namespace bulbit
