#pragma once

#include "filter.h"
#include "sampling.h"

namespace bulbit
{

class BoxFilter : public Filter
{
public:
    BoxFilter(Float extent)
        : Filter(extent)
    {
    }

    virtual Float Evaluate(Point2 p) const override
    {
        Float half_extent = extent / 2;
        if (std::abs(p.x) <= half_extent && std::abs(p.y) <= half_extent)
        {
            return 1 / Sqr(extent);
        }

        return 0;
    }

    virtual Point2 Sample(Point2 u) const override
    {
        // Remap [0, 1]^2 to [-half_extent, half_extent]^2
        return (2 * u - 1) * extent / 2;
    }
};

class TentFilter : public Filter
{
public:
    TentFilter(Float extent)
        : Filter(extent)
    {
    }

    virtual Float Evaluate(Point2 p) const override
    {
        Float half_extent = extent / 2;
        Float dist_x = std::abs(p.x);
        Float dist_y = std::abs(p.y);

        if (dist_x > half_extent || dist_y > half_extent)
        {
            return 0;
        }

        Float inv_r = 1 / half_extent;
        return (half_extent - dist_x) * (half_extent - dist_y) * (Sqr(Sqr(inv_r)));
    }

    virtual Point2 Sample(Point2 u) const override
    {
        Float half_extent = extent / 2;
        Float x = u[0] < 0.5f ? half_extent * (sqrt(2 * u[0]) - 1) : half_extent * (1 - sqrt(1 - 2 * (u[0] - 0.5f)));
        Float y = u[1] < 0.5f ? half_extent * (sqrt(2 * u[1]) - 1) : half_extent * (1 - sqrt(1 - 2 * (u[1] - 0.5f)));

        return Point2(x, y);
    }
};

class GaussianFilter : public Filter
{
public:
    GaussianFilter(Float sigma, Float extent = 3)
        : Filter(extent)
        , sigma{ sigma }
    {
        const int32 samples = 32;
        Float values[samples * samples];

        Float half_extent = extent / 2;
        for (int32 y = 0; y < samples; ++y)
        {
            for (int32 x = 0; x < samples; ++x)
            {
                Point2 p = (Point2i(x, y) + Point2(0.5f)) / samples * extent - half_extent;

                values[y * samples + x] = Gaussian(p.x, 0, sigma) * Gaussian(p.y, 0, sigma);
            }
        }

        dist = std::make_unique<Distribution2D>(values, samples, samples);
    }

    virtual Float Evaluate(Point2 p) const override
    {
        Float half_extent = extent / 2;
        if (std::abs(p.x) > half_extent || std::abs(p.y) > half_extent)
        {
            return 0;
        }

        return dist->PDF(p / extent + Point2(0.5f)) / Sqr(extent);
    }

    virtual Point2 Sample(Point2 u) const override
    {
        return dist->SampleContinuous(u) * extent - extent / 2;
    }

    const Float sigma;

private:
    std::unique_ptr<Distribution2D> dist;
};

} // namespace bulbit
