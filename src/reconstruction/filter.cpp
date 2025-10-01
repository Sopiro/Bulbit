#include "bulbit/filters.h"
#include "bulbit/renderer_info.h"

namespace bulbit
{

Filter* Filter::Create(Allocator& alloc, const ReconFilterInfo& fi)
{
    switch (fi.type)
    {
    case ReconFilterType::box:
        return alloc.new_object<BoxFilter>(fi.extent);
    case ReconFilterType::tent:
        return alloc.new_object<TentFilter>(fi.extent);
    case ReconFilterType::gaussian:
        return alloc.new_object<GaussianFilter>(fi.gaussian_stddev, fi.extent);

    default:
        return nullptr;
    }
}

} // namespace bulbit
