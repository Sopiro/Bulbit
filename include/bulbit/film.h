#pragma once

#include "image.h"

namespace bulbit
{

class Camera;

class Film
{
public:
    Film(const Camera* camera);

    void AddSample(const Point2i& pixel, const SpectrumSample& L, const WavelengthSample& lambda);
    void AddSample(const Point2i& pixel, const Vec3& xyz);
    void AddSplat(const Point2& pixel, const SpectrumSample& L, const WavelengthSample& lambda);

    void WeightSplats(Float weight);

    Image3 GetRenderedImage() const;
    Image1 GetVarianceImage() const;

private:
    void AccumulateSample(const Point2i& pixel, const Vec3& value);
    void AccumulateSplat(const Point2& pixel, const Vec3& value);

    const Camera* camera;

    std::unique_ptr<Vec3[]> samples;
    std::unique_ptr<int32[]> sample_counts;

    std::unique_ptr<std::atomic<Float>[]> splats;

    // luminance moments (l, l^2)
    std::unique_ptr<Point2[]> moments;
};

} // namespace bulbit
