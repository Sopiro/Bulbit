#pragma once

#include "bxdf.h"
#include "frame.h"

namespace bulbit
{

// BxDF bound with shading frame
class BSDF
{
public:
    BSDF() = default;
    BSDF(Vec3 n, BxDF* bxdf);
    BSDF(Vec3 n, Vec3 t, BxDF* bxdf);

    BxDF_Flags Flags() const;

    Spectrum f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const;
    Float PDF(
        Vec3 wo,
        Vec3 wi,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const;

    bool Sample_f(
        BSDFSample* sample,
        Vec3 wo,
        Float u0,
        Point2 u12,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const;

    Spectrum rho(
        Vec3 wo, std::span<const Float> uc, std::span<const Point2> u, TransportDirection direction = TransportDirection::ToLight
    ) const;
    Spectrum rho(
        std::span<const Point2> u1,
        std::span<const Float> uc,
        std::span<const Point2> u2,
        TransportDirection direction = TransportDirection::ToLight
    ) const;

    Vec3 WorldToLocal(const Vec3& v) const;
    Vec3 LocalToWorld(const Vec3& v) const;

    void Regularize();

    BxDF* GetBxDF();

private:
    Frame frame;
    BxDF* bxdf;
};

inline BSDF::BSDF(Vec3 n, BxDF* bxdf)
    : frame{ Frame::FromZ(n) }
    , bxdf{ bxdf }
{
}

inline BSDF::BSDF(Vec3 n, Vec3 t, BxDF* bxdf)
    : frame{ Frame::FromXZ(t, n) }
    , bxdf{ bxdf }
{
}

inline BxDF_Flags BSDF::Flags() const
{
    return bxdf->Flags();
}

inline Spectrum BSDF::f(Vec3 wo, Vec3 wi, TransportDirection direction) const
{
    wi = WorldToLocal(wi);
    wo = WorldToLocal(wo);

    if (wo.z == 0)
    {
        return Spectrum::black;
    }

    return bxdf->f(wo, wi, direction);
}

inline Float BSDF::PDF(Vec3 wo, Vec3 wi, TransportDirection direction, BxDF_SamplingFlags flags) const
{
    wo = WorldToLocal(wo);
    wi = WorldToLocal(wi);

    if (wo.z == 0)
    {
        return 0;
    }

    return bxdf->PDF(wo, wi, direction, flags);
}

inline bool BSDF::Sample_f(
    BSDFSample* sample, Vec3 wo, Float u0, Point2 u12, TransportDirection direction, BxDF_SamplingFlags flags
) const
{
    BulbitAssert(sample != nullptr);

    wo = WorldToLocal(wo);
    if (wo.z == 0 || !(bxdf->Flags() & flags))
    {
        return false;
    }

    if (!bxdf->Sample_f(sample, wo, u0, u12, direction, flags))
    {
        return false;
    }

    if (sample->f.IsBlack() || sample->pdf == 0 || sample->wi.z == 0)
    {
        return false;
    }

    sample->wi = LocalToWorld(sample->wi);
    return true;
}

inline Spectrum BSDF::rho(
    std::span<const Point2> u1, std::span<const Float> uc, std::span<const Point2> u2, TransportDirection direction
) const
{
    return bxdf->rho(u1, uc, u2, direction);
}

inline Spectrum BSDF::rho(Vec3 wo, std::span<const Float> uc, std::span<const Point2> u, TransportDirection direction) const
{
    wo = WorldToLocal(wo);
    return bxdf->rho(wo, uc, u, direction);
}

inline Vec3 BSDF::WorldToLocal(const Vec3& v) const
{
    return frame.ToLocal(v);
}

inline Vec3 BSDF::LocalToWorld(const Vec3& v) const
{
    return frame.FromLocal(v);
}

inline void BSDF::Regularize()
{
    bxdf->Regularize();
}

inline BxDF* BSDF::GetBxDF()
{
    return bxdf;
}

} // namespace bulbit
