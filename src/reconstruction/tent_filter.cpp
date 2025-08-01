#include "bulbit/filters.h"

namespace bulbit
{

Float TentFilter::Evaluate(Point2 p) const
{
    Float half_extent = extent / 2;
    Float dist_x = std::abs(p.x);
    Float dist_y = std::abs(p.y);

    if (dist_x > half_extent || dist_y > half_extent)
    {
        return 0;
    }

    Float normalization = 1 / Sqr(Sqr(half_extent));
    return (half_extent - dist_x) * (half_extent - dist_y) * normalization;
}

Point2 TentFilter::Sample(Point2 u) const
{
    Float half_extent = extent / 2;

    Float x = u[0] < 0.5f ? half_extent * (std::sqrt(2 * u[0]) - 1) : half_extent * (1 - std::sqrt(1 - 2 * (u[0] - 0.5f)));
    Float y = u[1] < 0.5f ? half_extent * (std::sqrt(2 * u[1]) - 1) : half_extent * (1 - std::sqrt(1 - 2 * (u[1] - 0.5f)));

    return Point2(x, y);
}

} // namespace bulbit
