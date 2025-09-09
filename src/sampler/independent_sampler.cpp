#include "bulbit/hash.h"
#include "bulbit/samplers.h"

namespace bulbit
{

IndependentSampler::IndependentSampler(int32 samples_per_pixel, int32 seed)
    : Sampler(samples_per_pixel)
{
    rng.Seed(seed);
}

void IndependentSampler::StartPixelSample(const Point2i& pixel, int32 sample_index)
{
    Sampler::StartPixelSample(pixel, sample_index);
    rng.Seed(Hash(pixel, seed, sample_index));
}

Float IndependentSampler::Next1D()
{
    return rng.NextFloat();
}

Point2 IndependentSampler::Next2D()
{
    return Point2{ rng.NextFloat(), rng.NextFloat() };
}

Sampler* IndependentSampler::Clone(Allocator& alloc) const
{
    return alloc.new_object<IndependentSampler>(samples_per_pixel, seed);
}

} // namespace bulbit
