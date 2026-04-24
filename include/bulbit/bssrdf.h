#pragma once

#include "bsdf.h"
#include "bxdfs.h"
#include "intersectable.h"
#include "media.h"

namespace bulbit
{

struct BSSRDFSample
{
    Intersection pi;
    Vec3 wo; // Dummy direction

    SpectrumSample Sp, pdf;
    Float p; // vertex sampling probability

    BSDF Sw;
};

class BSSRDF
{
public:
    BSSRDF(const Intersection& po, Float eta)
        : po{ po }
        , eta{ eta }
    {
    }

    virtual SpectrumSample S(
        const Intersection& pi, const Vec3& wo, const Vec3& wi, TransportDirection direction = TransportDirection::ToLight
    ) const = 0;
    virtual bool Sample_S(
        BSSRDFSample* bssrdf_sample,
        const BSDFSample& bsdf_sample,
        const Intersectable* accel,
        const WavelengthSample& lambda,
        Float u0,
        Point2 u12
    ) = 0;

protected:
    const Intersection& po;
    Float eta;
};

class SeparableBSSRDF : public BSSRDF
{
public:
    SeparableBSSRDF(const Intersection& po, Float eta)
        : BSSRDF(po, eta)
        , sw{ eta }
    {
    }

    virtual SpectrumSample S(
        const Intersection& pi, const Vec3& wo, const Vec3& wi, TransportDirection direction = TransportDirection::ToLight
    ) const override;
    SpectrumSample Sw(const Intersection& pi, const Vec3& wi, TransportDirection direction = TransportDirection::ToLight) const;
    SpectrumSample Sp(const Intersection& pi) const;

    virtual bool Sample_S(
        BSSRDFSample* bssrdf_sample,
        const BSDFSample& bsdf_sample,
        const Intersectable* accel,
        const WavelengthSample& lambda,
        Float u0,
        Point2 u12
    ) override;

    SpectrumSample PDF_Sp(const Intersection& pi) const;

    virtual Float MaxSr(const WavelengthSample& lambda) const = 0;
    virtual SpectrumSample Sr(Float d) const = 0;
    virtual Float Sample_Sr(const WavelengthSample& lambda, Float u) const = 0;
    virtual SpectrumSample PDF_Sr(Float r) const = 0;

private:
    static inline Float axis_sampling_probabilities[3] = { 0.25f, 0.25f, 0.5f };
    NormalizedFresnelBxDF sw;
};

class RandomWalkBSSRDF : public BSSRDF
{
public:
    RandomWalkBSSRDF(
        const SpectrumSample& R,
        const SpectrumSample& sigma_a,
        const SpectrumSample& sigma_s,
        const Intersection& po,
        Float eta,
        Float g
    )
        : BSSRDF(po, eta)
        , R{ R }
        , sigma_a{ sigma_a }
        , sigma_s{ sigma_s }
        , phase_function{ g }
        , sw{ eta }
    {
    }

    virtual SpectrumSample S(
        const Intersection& pi, const Vec3& wo, const Vec3& wi, TransportDirection direction = TransportDirection::ToLight
    ) const override;
    virtual bool Sample_S(
        BSSRDFSample* bssrdf_sample,
        const BSDFSample& bsdf_sample,
        const Intersectable* accel,
        const WavelengthSample& lambda,
        Float u0,
        Point2 u12
    ) override;

private:
    SpectrumSample R;
    SpectrumSample sigma_a, sigma_s;
    HenyeyGreensteinPhaseFunction phase_function;
    NormalizedFresnelBxDF sw;
};

} // namespace bulbit
