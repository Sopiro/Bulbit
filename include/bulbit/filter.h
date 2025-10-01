#pragma once

#include "vectors.h"

namespace bulbit
{

struct ReconFilterInfo;

// Pixel reconstruction filter
class Filter
{
public:
    static Filter* Create(Allocator& alloc, const ReconFilterInfo& filter_info);

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
