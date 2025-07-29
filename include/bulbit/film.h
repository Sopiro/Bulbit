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
    Film(const Camera* camera);

    void AddSample(const Point2i& pixel, const Spectrum& L, Float weight);
    void AddSplat(const Point2& pixel, const Spectrum& L);

    void WeightSplats(Float weight);

    Image3 GetRenderedImage() const;
    Image1 GetVarianceImage() const;

private:
    const Camera* camera;

    std::unique_ptr<Spectrum[]> samples;
    std::unique_ptr<int32[]> sample_counts;

    std::unique_ptr<std::atomic<Float>[]> splats;

    // luminance moments (l, l^2)
    std::unique_ptr<Point2[]> moments;
};

inline Film::Film(const Camera* camera)
    : camera{ camera }
{
    Point2i res = camera->GetScreenResolution();
    int32 size = res.x * res.y;
    samples = std::make_unique<Spectrum[]>(size);
    sample_counts = std::make_unique<int32[]>(size);
    moments = std::make_unique<Point2[]>(size);

    splats = std::make_unique<std::atomic<Float>[]>(Spectrum::num_spectral_samples * size);

    memset((void*)samples.get(), 0, sizeof(Spectrum) * size);
    memset((void*)sample_counts.get(), 0, sizeof(int32) * size);
    memset((void*)moments.get(), 0, sizeof(Point2) * size);

    ParallelFor(0, Spectrum::num_spectral_samples * size, [&](int32 i) { splats[i].store(0); });
}

inline void Film::AddSample(const Point2i& pixel, const Spectrum& L, Float w)
{
    Point2i res = camera->GetScreenResolution();
    int32 index = pixel.y * res.x + pixel.x;
    samples[index] += L * w;
    sample_counts[index] += 1;

    Float l = L.Luminance();

    Float alpha = 1.0f / sample_counts[index];
    moments[index] = Lerp(moments[index], Point2(l, l * l), alpha);
}

inline void Film::AddSplat(const Point2& pixel, const Spectrum& L)
{
    const Filter* filter = camera->GetFilter();
    Float r = filter->radius;

    Point2i lower(std::floor(pixel.x - r), std::floor(pixel.y - r));
    Point2i upper(std::floor(pixel.x + r) + 1, std::floor(pixel.y + r) + 1);
    AABB2i bounds(lower, upper);

    // Compute the pixel bounds affected by this splat
    Point2i res = camera->GetScreenResolution();
    bounds = AABB2i::Intersection(AABB2i(Point2i(0), res), bounds);

    for (Point2i pi : bounds)
    {
        Float weight = filter->Evaluate(pixel - (Point2(pi) + Point2(0.5f)));
        if (weight > 0)
        {
            int32 index = Spectrum::num_spectral_samples * (pi.y * res.x + pi.x);
            for (int32 s = 0; s < Spectrum::num_spectral_samples; ++s)
            {
                Float old = splats[index + s].load();
                splats[index + s].store(old + weight * L[s]);
            }
        }
    }
}

inline void Film::WeightSplats(Float weight)
{
    Point2i res = camera->GetScreenResolution();
    int32 size = Spectrum::num_spectral_samples * res.x * res.y;

    ParallelFor(0, size, [&](int32 i) {
        Float value = splats[i].load();
        value *= weight;

        splats[i].store(value);
    });
}

inline Image3 Film::GetRenderedImage() const
{
    Point2i res = camera->GetScreenResolution();
    Image3 image(res.x, res.y);

    ParallelFor(0, res.x * res.y, [&](int32 i) {
        image[i] = samples[i] / sample_counts[i];

        int32 index = Spectrum::num_spectral_samples * i;

        Spectrum splat;
        for (int32 s = 0; s < Spectrum::num_spectral_samples; ++s)
        {
            splat[s] = splats[index + s].load();
        }

        image[i] += splat;
    });

    return image;
}

inline Image1 Film::GetVarianceImage() const
{
    Point2i res = camera->GetScreenResolution();
    Image1 image(res.x, res.y);

    ParallelFor(0, res.x * res.y, [&](int32 i) {
        int32 n = sample_counts[i];

        // bessel's correction factor
        Float correction = n > 1 ? n / (n - 1.0f) : 1.0f;
        Float variance = std::max(0.0f, moments[i][1] - Sqr(moments[i][0])) * correction;

        image[i] = variance;
    });

    return image;
}

} // namespace bulbit
