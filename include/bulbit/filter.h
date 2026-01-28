#pragma once

#include "allocator.h"
#include "vectors.h"

namespace bulbit
{

struct FilterInfo;

// Pixel reconstruction filter
class Filter
{
public:
    static Filter* Create(Allocator& alloc, const FilterInfo& filter_info);

    Filter(Float extent)
        : extent{ extent }
    {
    }
    virtual ~Filter() = default;

    virtual Float Evaluate(Point2 p) const = 0;
    virtual Point2 Sample(Point2 u) const = 0;

    const Float extent;
};

} // namespace bulbit
