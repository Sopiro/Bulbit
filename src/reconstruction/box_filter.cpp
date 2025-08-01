#include "bulbit/filters.h"

namespace bulbit
{

Float BoxFilter::Evaluate(Point2 p) const
{
    Float half_extent = extent / 2;
    if (std::abs(p.x) <= half_extent && std::abs(p.y) <= half_extent)
    {
        return 1 / Sqr(extent);
    }

    return 0;
}

Point2 BoxFilter::Sample(Point2 u) const
{
    // Remap [0, 1]^2 to [-half_extent, half_extent]^2
    return (2 * u - 1) * extent / 2;
}

} // namespace bulbit
