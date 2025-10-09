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

    Spectrum Sp, pdf;
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

    virtual Spectrum S(
        const Intersection& pi, const Vec3& wo, const Vec3& wi, TransportDirection direction = TransportDirection::ToLight
    ) const = 0;
    virtual bool Sample_S(
        BSSRDFSample* bssrdf_sample,
        const BSDFSample& bsdf_sample,
        const Intersectable* accel,
        int32 wavelength,
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

    virtual Spectrum S(
        const Intersection& pi, const Vec3& wo, const Vec3& wi, TransportDirection direction = TransportDirection::ToLight
    ) const override;
    Spectrum Sw(const Intersection& pi, const Vec3& wi, TransportDirection direction = TransportDirection::ToLight) const;
    Spectrum Sp(const Intersection& pi) const;

    virtual bool Sample_S(
        BSSRDFSample* bssrdf_sample,
        const BSDFSample& bsdf_sample,
        const Intersectable* accel,
        int32 wavelength,
        Float u0,
        Point2 u12
    ) override;

    Spectrum PDF_Sp(const Intersection& pi) const;

    virtual Float MaxSr(int32 wavelength) const = 0;
    virtual Spectrum Sr(Float d) const = 0;
    virtual Float Sample_Sr(int32 wavelength, Float u) const = 0;
    virtual Spectrum PDF_Sr(Float r) const = 0;

private:
    static inline Float axis_sampling_probabilities[3] = { 0.25f, 0.25f, 0.5f };
    NormalizedFresnelBxDF sw;
};

class RandomWalkBSSRDF : public BSSRDF
{
public:
    RandomWalkBSSRDF(
        const Spectrum& R, const Spectrum& sigma_a, const Spectrum& sigma_s, const Intersection& po, Float eta, Float g
    )
        : BSSRDF(po, eta)
        , R{ R }
        , sigma_t{ sigma_a + sigma_s }
        , albedo{ sigma_s / (sigma_a + sigma_s) }
        , phase_function{ g }
        , sw{ eta }
    {
    }

    virtual Spectrum S(
        const Intersection& pi, const Vec3& wo, const Vec3& wi, TransportDirection direction = TransportDirection::ToLight
    ) const override;
    virtual bool Sample_S(
        BSSRDFSample* bssrdf_sample,
        const BSDFSample& bsdf_sample,
        const Intersectable* accel,
        int32 wavelength,
        Float u0,
        Point2 u12
    ) override;

private:
    Spectrum R;
    Spectrum sigma_t, albedo;
    HenyeyGreensteinPhaseFunction phase_function;
    NormalizedFresnelBxDF sw;
};

} // namespace bulbit
