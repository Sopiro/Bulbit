#pragma once

#include "camera.h"
#include "image.h"
#include "parallel_for.h"
#include "spectrum.h"

namespace bulbit
{

class Film
{
public:
    Film(const Point2i& resolution);

    void AddSample(const Point2i& pixel, const Spectrum& L, Float weight);
    Image3 GetRenderedImage() const;
    Image1 GetVarianceImage() const;

    const Point2i resolution;

private:
    std::unique_ptr<Spectrum[]> samples;
    std::unique_ptr<int32[]> sample_counts;

    // luminance moments (l, l^2)
    std::unique_ptr<Point2[]> moments;
};

inline Film::Film(const Point2i& resolution)
    : resolution{ resolution }
{
    int32 size = resolution.x * resolution.y;
    samples = std::make_unique<Spectrum[]>(size);
    sample_counts = std::make_unique<int32[]>(size);
    moments = std::make_unique<Point2[]>(size);

    memset((void*)samples.get(), 0, sizeof(Spectrum) * size);
    memset((void*)sample_counts.get(), 0, sizeof(int32) * size);
    memset((void*)moments.get(), 0, sizeof(Point2) * size);
}

inline void Film::AddSample(const Point2i& pixel, const Spectrum& L, Float w)
{
    const int32 index = pixel.y * resolution.x + pixel.x;
    samples[index] += L * w;
    sample_counts[index] += 1;

    Float l = L.Luminance();

    Float alpha = 1.0f / sample_counts[index];
    moments[index] = Lerp(moments[index], Point2(l, l * l), alpha);
}

inline Image3 Film::GetRenderedImage() const
{
    int32 width = resolution.x;
    int32 height = resolution.y;
    Image3 image(width, height);

    ParallelFor(0, width * height, [&](int32 i) { image[i] = samples[i] / sample_counts[i]; });

    return image;
}

inline Image1 Film::GetVarianceImage() const
{
    int32 width = resolution.x;
    int32 height = resolution.y;
    Image1 image(width, height);

    ParallelFor(0, width * height, [&](int32 i) {
        int32 n = sample_counts[i];

        // bessel's correction factor
        Float correction = n > 1 ? n / (n - 1.0f) : 1.0f;
        Float variance = std::max(0.0f, moments[i][1] - Sqr(moments[i][0])) * correction;

        image[i] = variance;
    });

    return image;
}

} // namespace bulbit
