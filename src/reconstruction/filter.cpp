#include "bulbit/filters.h"
#include "bulbit/renderer_info.h"

namespace bulbit
{

Filter* Filter::Create(Allocator& alloc, const FilterInfo& fi)
{
    switch (fi.type)
    {
    case FilterType::box:
        return alloc.new_object<BoxFilter>(fi.extent);
    case FilterType::tent:
        return alloc.new_object<TentFilter>(fi.extent);
    case FilterType::gaussian:
        return alloc.new_object<GaussianFilter>(fi.gaussian_stddev, fi.extent);

    default:
        return nullptr;
    }
}

} // namespace bulbit
