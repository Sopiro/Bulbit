#include "bulbit/light_samplers.h"
#include "bulbit/lights.h"
#include "bulbit/sampling.h"

namespace bulbit
{

PowerLightSampler::PowerLightSampler(std::span<Light*> lights)
    : LightSampler(lights)
{
    size_t light_count = lights.size();
    std::vector<Float> powers(light_count);

    for (size_t i = 0; i < light_count; ++i)
    {
        Light* light = lights[i];
        powers[i] = light->Phi().Average();
        light_to_index[light] = int32(i);
    }

    distribution = Distribution1D(&powers[0], light_count);
}

bool PowerLightSampler::Sample(SampledLight* sampled_light, const Intersection& isect, Float u) const
{
    BulbitNotUsed(isect);

    // Sample light based on light's radient intensity

    Float pmf;
    int32 index = distribution.SampleDiscrete(u, &pmf);

    sampled_light->light = lights[index];
    sampled_light->weight = 1 / pmf;

    return true;
}

Float PowerLightSampler::EvaluatePMF(const Light* light) const
{
    return distribution.DiscretePDF(light_to_index.at(light));
}

} // namespace bulbit
