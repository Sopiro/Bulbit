#include "bulbit/light_samplers.h"
#include "bulbit/lights.h"
#include "bulbit/sampling.h"

namespace bulbit
{

PowerLightSampler::PowerLightSampler(std::span<Light*> lights)
    : LightSampler(lights)
{
    int32 light_count = int32(lights.size());
    std::vector<Float> powers(light_count);

    for (int32 i = 0; i < light_count; ++i)
    {
        Light* light = lights[i];
        powers[i] = light->Phi().Luminance();
        light_to_index[light] = i;
    }

    distribution = Distribution1D(&powers[0], light_count);
}

// Sample light based on light's radient intensity
bool PowerLightSampler::Sample(SampledLight* sampled_light, const Intersection& isect, Float u) const
{
    BulbitNotUsed(isect);

    Float pmf;
    int32 index = distribution.SampleDiscrete(u, &pmf);

    sampled_light->light = lights[index];
    sampled_light->pmf = pmf;

    return true;
}

Float PowerLightSampler::EvaluatePMF(const Light* light) const
{
    return distribution.DiscretePDF(light_to_index.at(light));
}

} // namespace bulbit
