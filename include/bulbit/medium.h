#pragma once

#include "allocator.h"
#include "dynamic_dispatcher.h"
#include "random.h"
#include "ray.h"
#include "sampling.h"
#include "spectrum.h"

namespace bulbit
{

struct PhaseFunctionSample
{
    Float p;
    Vec3 wi;
    Float pdf;
};

class PhaseFunction
{
public:
    virtual ~PhaseFunction() = default;

    virtual Float p(Vec3 wo, Vec3 wi) const = 0;
    virtual Float PDF(Vec3 wo, Vec3 wi) const = 0;

    virtual bool Sample_p(PhaseFunctionSample* sample, Vec3 wo, Point2 u) const = 0;
};

struct MediumSample
{
    SpectrumSample sigma_a, sigma_s;
    SpectrumSample Le;
    const PhaseFunction* phase;
};

struct RayMajorantSegment
{
    Float t_min, t_max;
    SpectrumSample sigma_maj;
};

class RayMajorantIterator
{
public:
    virtual bool Next(RayMajorantSegment* next_segment) = 0;
};

using Media = TypePack<class HomogeneousMedium, class NanoVDBMedium>;

class Medium : public DynamicDispatcher<Media>
{
public:
    using Types = Media;

    bool IsEmissive() const;
    MediumSample SamplePoint(Point3 p, const WavelengthSample& lambda) const;
    RayMajorantIterator* SampleRay(Ray ray, Float t_max, const WavelengthSample& lambda, Allocator& alloc) const;

    ~Medium();

protected:
    Medium(int32 index)
        : DynamicDispatcher(index)
    {
    }
};

struct MediumInterface
{
    MediumInterface()
        : inside{ nullptr }
        , outside{ nullptr }
    {
    }

    MediumInterface(const Medium* medium)
        : inside{ medium }
        , outside{ medium }
    {
    }

    MediumInterface(const Medium* inside, const Medium* outside)
        : inside{ inside }
        , outside{ outside }
    {
    }

    bool IsMediumTransition() const
    {
        return inside != outside;
    }

    const Medium* inside;
    const Medium* outside;
};

template <typename F>
SpectrumSample Sample_MajorantTransmittance(
    const Medium* medium, const WavelengthSample& lambda, Ray ray, Float t_max, Float u, RNG& rng, F callback
)
{
    return medium->Dispatch([&](auto m) -> SpectrumSample {
        return Sample_MajorantTransmittance(m, lambda, ray, t_max, u, rng, callback);
    });
}

// bool callback(Point3 p, MediumSample ms, SpectrumSample sigma_maj, SpectrumSample T_maj);
template <typename MediumType, typename F>
SpectrumSample Sample_MajorantTransmittance(
    const MediumType* medium, const WavelengthSample& lambda, Ray ray, Float t_max, Float u, RNG& rng, F callback
)
{
    t_max *= ray.d.Normalize();

    typename MediumType::MajorantIterator iter = medium->SampleRay(ray, t_max, lambda);

    const int32 hero = WavelengthSample::hero_lane;
    SpectrumSample T_maj(1);
    bool done = false;
    while (!done)
    {
        RayMajorantSegment segment;
        if (!iter.Next(&segment))
        {
            return T_maj;
        }

        SpectrumSample sigma_maj = segment.sigma_maj;
        if (sigma_maj[hero] == 0)
        {
            Float dt = segment.t_max - segment.t_min;
            if (dt == infinity)
            {
                dt = max_float;
            }

            T_maj *= Exp(-dt * sigma_maj);
            continue;
        }

        Float t_min = segment.t_min;

        while (true)
        {
            Float t = t_min + SampleExponential(u, sigma_maj[hero]);
            u = rng.NextFloat();

            if (t < segment.t_max)
            {
                T_maj *= Exp(-(t - t_min) * sigma_maj);
                Point3 p = ray.At(t);
                MediumSample ms = medium->SamplePoint(p, lambda);
                if (!callback(p, ms, sigma_maj, T_maj))
                {
                    done = true;
                    break;
                }

                T_maj = SpectrumSample(1);
                t_min = t;
            }
            else
            {
                Float dt = segment.t_max - t_min;
                if (dt == infinity)
                {
                    dt = max_float;
                }

                T_maj *= Exp(-dt * sigma_maj);
                break;
            }
        }
    }

    return SpectrumSample(1);
}

} // namespace bulbit
