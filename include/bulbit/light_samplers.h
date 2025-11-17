#pragma once

#include "hash_map.h"
#include "light_sampler.h"
#include "sampling.h"

namespace bulbit
{

class UniformLightSampler : public LightSampler
{
public:
    virtual bool Sample(SampledLight* sampled_light, const Intersection& isect, Float u) const override;
    virtual Float EvaluatePMF(const Light* light) const override;
};

class PowerLightSampler : public LightSampler
{
public:
    virtual void Init(std::span<Light*> all_lights) override;

    virtual bool Sample(SampledLight* sampled_light, const Intersection& isect, Float u) const override;
    virtual Float EvaluatePMF(const Light* light) const override;

private:
    Distribution1D distribution;
    HashMap<const Light*, int32> light_to_index;
};

} // namespace bulbit
