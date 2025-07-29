#pragma once

#include "filter.h"
#include "sampling.h"

namespace bulbit
{

class BoxFilter : public Filter
{
public:
    BoxFilter(Float extent)
        : Filter(extent / 2)
        , inv_area{ 1 / Sqr(extent) }
    {
    }

    virtual Float Evaluate(Point2 p) const override
    {
        if (std::abs(p.x) <= radius && std::abs(p.y) <= radius)
        {
            return inv_area;
        }

        return 0;
    }

    virtual Point2 Sample(Point2 u) const override
    {
        // Remap [0, 1]^2 to [-radius, radius]^2
        return (2 * u - 1) * radius;
    }

private:
    Float inv_area;
};

class TentFilter : public Filter
{
public:
    TentFilter(Float extent)
        : Filter(extent / 2)
        , extent{ extent }
    {
    }

    virtual Float Evaluate(Point2 p) const override
    {
        Float dist_x = std::abs(p.x);
        Float dist_y = std::abs(p.y);
        if (dist_x > radius || dist_y > radius)
        {
            return 0;
        }

        Float inv_r = 1 / radius;
        return (radius - dist_x) * (radius - dist_y) * (Sqr(Sqr(inv_r)));
    }

    virtual Point2 Sample(Point2 u) const override
    {
        Float x = u[0] < Float(0.5) ? radius * (sqrt(2 * u[0]) - 1) : radius * (1 - sqrt(1 - 2 * (u[0] - Float(0.5))));
        Float y = u[1] < Float(0.5) ? radius * (sqrt(2 * u[1]) - 1) : radius * (1 - sqrt(1 - 2 * (u[1] - Float(0.5))));

        return Point2(x, y);
    }

private:
    Float extent;
};

class GaussianFilter : public Filter
{
public:
    GaussianFilter(Float sigma, Float extent = 3)
        : Filter(extent / 2)
        , extent{ extent }
        , sigma{ sigma }
    {
        const int32 samples = 32;
        Float values[samples * samples];

        for (int32 y = 0; y < samples; ++y)
        {
            for (int32 x = 0; x < samples; ++x)
            {
                Point2 p = (Point2(x, y) + 0.5f) / samples * extent - radius;

                values[y * samples + x] = Gaussian(p.x, 0, sigma) * Gaussian(p.y, 0, sigma);
            }
        }

        dist = std::make_unique<Distribution2D>(values, samples, samples);
    }

    virtual Float Evaluate(Point2 p) const override
    {
        if (std::abs(p.x) > radius || std::abs(p.y) > radius)
        {
            return 0;
        }

        return dist->Pdf(p / extent + Point2(0.5f)) / Sqr(extent);
    }

    virtual Point2 Sample(Point2 u) const override
    {
        Float pdf;
        return dist->SampleContinuous(&pdf, u) * extent - radius;
    }

private:
    std::unique_ptr<Distribution2D> dist;
    Float extent;
    Float sigma;
};

} // namespace bulbit
