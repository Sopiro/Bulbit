#pragma once

#include "light_sampler.h"
#include "sampling.h"

namespace bulbit
{

class UniformLightSampler : public LightSampler
{
public:
    UniformLightSampler(std::span<Light*> lights);

    virtual bool Sample(SampledLight* sampled_light, const Intersection& isect, Float u) const override;
    virtual Float EvaluatePMF(const Light* light) const override;
};

class PowerLightSampler : public LightSampler
{
public:
    PowerLightSampler(std::span<Light*> lights);

    virtual bool Sample(SampledLight* sampled_light, const Intersection& isect, Float u) const override;
    virtual Float EvaluatePMF(const Light* light) const override;

private:
    Distribution1D distribution;
    std::unordered_map<const Light*, int32> light_to_index;
};

} // namespace bulbit
