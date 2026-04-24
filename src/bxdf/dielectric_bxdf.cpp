#include "bulbit/bxdfs.h"
#include "bulbit/frame.h"
#include "bulbit/sampling.h"
#include "bulbit/scattering.h"

namespace bulbit
{

namespace
{

Float GetHeroEta(const SpectrumSample& eta)
{
    return eta[WavelengthSample::hero_lane];
}
Float DielectricPDF(const TrowbridgeReitzDistribution& mf, Float eta, Vec3 wo, Vec3 wi, BxDF_SamplingFlags flags)
{
    if (eta == 1 || mf.EffectivelySmooth())
    {
        return 0;
    }

    Float cos_theta_o = CosTheta(wo);
    Float cos_theta_i = CosTheta(wi);

    Float eta_p = 1;
    bool reflect = cos_theta_i * cos_theta_o > 0;
    if (!reflect)
    {
        eta_p = cos_theta_o > 0 ? eta : (1 / eta);
    }

    Vec3 wm = wi * eta_p + wo;
    if (cos_theta_i == 0 || cos_theta_o == 0 || Length2(wm) == 0)
    {
        return 0;
    }

    wm.Normalize();
    if (Dot(wm, z_axis) < 0)
    {
        wm.Negate();
    }

    if (Dot(wm, wi) * cos_theta_i < 0 || Dot(wm, wo) * cos_theta_o < 0)
    {
        return 0;
    }

    Float R = FresnelDielectric(Dot(wo, wm), eta);
    Float T = 1 - R;

    Float pr = R;
    Float pt = T;
    if (!(flags & BxDF_SamplingFlags::Reflection)) pr = 0;
    if (!(flags & BxDF_SamplingFlags::Transmission)) pt = 0;
    if (pr == 0 && pt == 0)
    {
        return 0;
    }

    if (reflect)
    {
        return mf.PDF(wo, wm) / (4 * AbsDot(wo, wm)) * pr / (pr + pt);
    }

    Float dwm_dwi = AbsDot(wi, wm) / Sqr(Dot(wi, wm) + Dot(wo, wm) / eta_p);
    return mf.PDF(wo, wm) * dwm_dwi * pt / (pr + pt);
}

SpectrumSample EvaluateRoughDielectric(
    const TrowbridgeReitzDistribution& mf,
    const SpectrumSample& r,
    const SpectrumSample& eta,
    Vec3 wo,
    Vec3 wi,
    TransportDirection direction
)
{
    SpectrumSample result(0);
    for (int32 i = 0; i < SpectrumSample::num_lanes; ++i)
    {
        Float eta_i = eta[i];
        if (eta_i == 1 || mf.EffectivelySmooth())
        {
            continue;
        }

        Float cos_theta_o = CosTheta(wo);
        Float cos_theta_wi = CosTheta(wi);

        Float eta_p = 1;
        bool reflect = cos_theta_wi * cos_theta_o > 0;
        if (!reflect)
        {
            eta_p = cos_theta_o > 0 ? eta_i : (1 / eta_i);
        }

        Vec3 wm = wi * eta_p + wo;
        if (cos_theta_wi == 0 || cos_theta_o == 0 || Length2(wm) == 0)
        {
            continue;
        }

        wm.Normalize();
        if (Dot(wm, z_axis) < 0)
        {
            wm.Negate();
        }

        if (Dot(wm, wi) * cos_theta_wi < 0 || Dot(wm, wo) * cos_theta_o < 0)
        {
            continue;
        }

        Float F = FresnelDielectric(Dot(wo, wm), eta_i);
        if (reflect)
        {
            result[i] = F * mf.D(wm) * mf.G(wo, wi) / std::abs(4 * cos_theta_wi * cos_theta_o);
        }
        else
        {
            Float denom = Sqr(Dot(wi, wm) + Dot(wo, wm) / eta_p) * cos_theta_wi * cos_theta_o;
            Float ft = (1 - F) * mf.D(wm) * mf.G(wo, wi) * std::abs(Dot(wi, wm) * Dot(wo, wm) / denom);
            if (direction == TransportDirection::ToLight)
            {
                ft /= Sqr(eta_p);
            }

            result[i] = r[i] * ft;
        }
    }

    return result;
}

} // namespace

SpectrumSample DielectricBxDF::f(Vec3 wo, Vec3 wi, TransportDirection direction) const
{
    return EvaluateRoughDielectric(mf, r, eta, wo, wi, direction);
}

Float DielectricBxDF::PDF(Vec3 wo, Vec3 wi, TransportDirection direction, BxDF_SamplingFlags flags) const
{
    BulbitNotUsed(direction);
    return DielectricPDF(mf, GetHeroEta(eta), wo, wi, flags);
}

bool DielectricBxDF::Sample_f(
    BSDFSample* sample, Vec3 wo, Float u0, Point2 u12, TransportDirection direction, BxDF_SamplingFlags flags
) const
{
    Float eta_hero = GetHeroEta(eta);
    if (eta_hero == 1 || mf.EffectivelySmooth())
    {
        Float R = FresnelDielectric(CosTheta(wo), eta_hero);
        Float T = 1 - R;

        Float pr = R;
        Float pt = T;
        if (!(flags & BxDF_SamplingFlags::Reflection)) pr = 0;
        if (!(flags & BxDF_SamplingFlags::Transmission)) pt = 0;
        if (pr == 0 && pt == 0)
        {
            return false;
        }

        if (u0 < pr / (pr + pt))
        {
            Vec3 wi(-wo.x, -wo.y, wo.z);
            SpectrumSample fr(0);
            for (int32 i = 0; i < SpectrumSample::num_lanes; ++i)
            {
                fr[i] = FresnelDielectric(CosTheta(wo), eta[i]) / AbsCosTheta(wi);
            }

            *sample = BSDFSample(fr, wi, pr / (pr + pt), BxDF_Flags::SpecularReflection);
        }
        else
        {
            Vec3 wi;
            Float eta_p_hero;
            if (!Refract(&wi, wo, z_axis, eta_hero, &eta_p_hero))
            {
                return false;
            }

            SpectrumSample ft(0);
            for (int32 i = 0; i < SpectrumSample::num_lanes; ++i)
            {
                Float eta_p = CosTheta(wo) > 0 ? eta[i] : (1 / eta[i]);
                ft[i] = (1 - FresnelDielectric(CosTheta(wo), eta[i])) / AbsCosTheta(wi);
                if (direction == TransportDirection::ToLight)
                {
                    ft[i] /= Sqr(eta_p);
                }
            }

            SpectrumSample fs = r * ft;
            *sample = BSDFSample(fs, wi, pt / (pr + pt), BxDF_Flags::SpecularTransmission, eta_p_hero);
        }

        return true;
    }

    Vec3 wm = mf.Sample_Wm(wo, u12);

    Float R = FresnelDielectric(Dot(wo, wm), eta_hero);
    Float T = 1 - R;

    Float pr = R;
    Float pt = T;
    if (!(flags & BxDF_SamplingFlags::Reflection)) pr = 0;
    if (!(flags & BxDF_SamplingFlags::Transmission)) pt = 0;
    if (pr == 0 && pt == 0)
    {
        return false;
    }

    if (u0 < pr / (pr + pt))
    {
        Vec3 wi = Reflect(wo, wm);
        if (!SameHemisphere(wo, wi))
        {
            return false;
        }

        Float pdf = mf.PDF(wo, wm) / (4 * AbsDot(wo, wm)) * pr / (pr + pt);
        SpectrumSample fs = f(wo, wi, direction);
        *sample = BSDFSample(fs, wi, pdf, BxDF_Flags::GlossyReflection);
    }
    else
    {
        Float eta_p_hero;
        Vec3 wi;
        bool tir = !Refract(&wi, wo, wm, eta_hero, &eta_p_hero);
        if (SameHemisphere(wo, wi) || wi.z == 0 || tir)
        {
            return false;
        }

        Float denom = Sqr(Dot(wi, wm) + Dot(wo, wm) / eta_p_hero);
        Float dwm_dwi = AbsDot(wi, wm) / denom;
        Float pdf = mf.PDF(wo, wm) * dwm_dwi * pt / (pr + pt);
        SpectrumSample fs = f(wo, wi, direction);
        *sample = BSDFSample(fs, wi, pdf, BxDF_Flags::GlossyTransmission, eta_p_hero);
    }

    return true;
}

void DielectricBxDF::Regularize()
{
    mf.Regularize();
}

} // namespace bulbit
