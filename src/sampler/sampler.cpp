#include "bulbit/renderer_info.h"
#include "bulbit/samplers.h"

namespace bulbit
{

Sampler* Sampler::Create(Allocator& alloc, const SamplerInfo& si)
{
    switch (si.type)
    {
    case SamplerType::independent:
        return alloc.new_object<IndependentSampler>(si.spp);
    case SamplerType::stratified:
    {
        int32 h = std::sqrt(si.spp);
        return alloc.new_object<StratifiedSampler>(h, h, true);
    }

    default:
        return nullptr;
    }
}

} // namespace bulbit
