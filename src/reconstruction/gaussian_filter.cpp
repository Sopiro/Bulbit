#include "bulbit/filters.h"
#include "bulbit/parallel_for.h"
#include "bulbit/sampling.h"

namespace bulbit
{

GaussianFilter::GaussianFilter(Float sigma, Float extent)
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
            Point2 p = ((Point2i(x, y) + Point2(0.5f)) / samples) * extent - half_extent;

            values[y * samples + x] = Gaussian(p.x, 0, sigma) * Gaussian(p.y, 0, sigma);
        }
    }

    dist = std::make_unique<Distribution2D>(values, samples, samples);
}

Float GaussianFilter::Evaluate(Point2 p) const
{
    Float half_extent = extent / 2;
    if (std::abs(p.x) > half_extent || std::abs(p.y) > half_extent)
    {
        return 0;
    }

    Float normalization = 1 / Sqr(extent);
    return dist->PDF(p / extent + Point2(0.5f)) * normalization;
}

Point2 GaussianFilter::Sample(Point2 u) const
{
    return dist->SampleContinuous(u) * extent - extent / 2;
}

} // namespace bulbit
