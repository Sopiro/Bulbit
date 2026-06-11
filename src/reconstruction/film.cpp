#include "bulbit/film.h"
#include "bulbit/camera.h"
#include "bulbit/parallel_for.h"

namespace bulbit
{

Film::Film(const Camera* camera)
    : camera{ camera }
{
    Point2i res = camera->GetScreenResolution();
    int32 size = res.x * res.y;
    samples = std::make_unique<Vec3[]>(size);
    sample_counts = std::make_unique<int32[]>(size);
    moments = std::make_unique<Point2[]>(size);

    splats = std::make_unique<std::atomic<Float>[]>(Spectrum::num_spectral_samples * size);

    memset((void*)samples.get(), 0, sizeof(Vec3) * size);
    memset((void*)sample_counts.get(), 0, sizeof(int32) * size);
    memset((void*)moments.get(), 0, sizeof(Point2) * size);

    ParallelFor(0, Spectrum::num_spectral_samples * size, [&](int32 i) { splats[i].store(0); });
}

void Film::AddSample(const Point2i& pixel, const SpectrumSample& L, const WavelengthSample& lambda)
{
    AccumulateSample(pixel, spectral::ToXYZ(L, lambda));
}

void Film::AddSample(const Point2i& pixel, const Vec3& xyz)
{
    AccumulateSample(pixel, xyz);
}

void Film::AccumulateSample(const Point2i& pixel, const Vec3& xyz)
{
    Point2i res = camera->GetScreenResolution();
    int32 index = pixel.y * res.x + pixel.x;
    samples[index] += xyz;
    sample_counts[index] += 1;

    Float l = xyz.y;

    Float alpha = 1.0f / sample_counts[index];
    moments[index] = Lerp(moments[index], Point2(l, l * l), alpha);
}

void Film::AddSplat(const Point2& pixel, const SpectrumSample& L, const WavelengthSample& lambda)
{
    AccumulateSplat(pixel, spectral::ToXYZ(L, lambda));
}

void Film::AccumulateSplat(const Point2& pixel, const Vec3& xyz)
{
    const Filter* filter = camera->GetFilter();
    Float half_extent = filter->extent / 2;

    Point2i lower(int32(std::floor(pixel.x - half_extent)), int32(std::floor(pixel.y - half_extent)));
    Point2i upper(int32(std::floor(pixel.x + half_extent)) + 1, int32(std::floor(pixel.y + half_extent)) + 1);
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
                splats[index + s].fetch_add(weight * xyz[s], std::memory_order_relaxed);
            }
        }
    }
}

void Film::WeightSplats(Float weight)
{
    Point2i res = camera->GetScreenResolution();
    int32 size = Spectrum::num_spectral_samples * res.x * res.y;

    ParallelFor(0, size, [&](int32 i) {
        Float value = splats[i].load();
        value *= weight;

        splats[i].store(value);
    });
}

Image3 Film::GetRenderedImage() const
{
    Point2i res = camera->GetScreenResolution();
    Image3 image(res.x, res.y);

    ParallelFor(0, res.x * res.y, [&](int32 i) {
        Vec3 xyz = sample_counts[i] > 0 ? samples[i] / sample_counts[i] : Vec3(0);

        int32 index = Spectrum::num_spectral_samples * i;

        Vec3 splat(0);
        for (int32 s = 0; s < Spectrum::num_spectral_samples; ++s)
        {
            splat[s] = splats[index + s].load();
        }

        xyz += splat;
        image[i] = Max(spectral::XYZToLinearRGB(xyz), Vec3(0));
    });

    return image;
}

Image1 Film::GetVarianceImage() const
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
