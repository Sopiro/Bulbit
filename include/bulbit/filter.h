#pragma once

#include "vectors.h"

namespace bulbit
{

// Pixel reconstruction filter
class Filter
{
public:
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
