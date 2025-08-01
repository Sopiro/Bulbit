#pragma once

#include "filter.h"
#include "sampling.h"

namespace bulbit
{

class BoxFilter : public Filter
{
public:
    BoxFilter(Float extent = 1)
        : Filter(extent)
    {
    }

    virtual Float Evaluate(Point2 p) const override;
    virtual Point2 Sample(Point2 u) const override;
};

class TentFilter : public Filter
{
public:
    TentFilter(Float extent = 2)
        : Filter(extent)
    {
    }

    virtual Float Evaluate(Point2 p) const override;
    virtual Point2 Sample(Point2 u) const override;
};

class GaussianFilter : public Filter
{
public:
    GaussianFilter(Float sigma = 0.5f, Float extent = 3);

    virtual Float Evaluate(Point2 p) const override;
    virtual Point2 Sample(Point2 u) const override;

    const Float sigma;

private:
    std::unique_ptr<Distribution2D> dist;
};

} // namespace bulbit
