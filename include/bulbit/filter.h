#pragma once

#include "vectors.h"

namespace bulbit
{

// Pixel reconstruction filter
class Filter
{
public:
    Filter() = default;
    virtual ~Filter() = default;

    virtual Point2 Sample(Point2 u) const = 0;
};

} // namespace bulbit
