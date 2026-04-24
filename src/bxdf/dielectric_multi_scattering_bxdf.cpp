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
} // namespace

SpectrumSample DielectricMultiScatteringBxDF::f(Vec3 wo, Vec3 wi, TransportDirection direction) const
{
    SpectrumSample result(0);

    for (int32 i = 0; i < SpectrumSample::num_lanes; ++i)
    {
        Float eta_i_forward = eta[i];
        if (eta_i_forward == 1 || mf.EffectivelySmooth())
        {
            continue;
        }

        Float cos_theta_o = CosTheta(wo);
        Float cos_theta_wi = CosTheta(wi);

        Float eta_p = 1;
        bool reflect = cos_theta_wi * cos_theta_o > 0;
        if (!reflect)
        {
            eta_p = cos_theta_o > 0 ? eta_i_forward : (1 / eta_i_forward);
        }

        Float eta_o = cos_theta_o > 0 ? eta_i_forward : (1 / eta_i_forward);
        Float eta_i = cos_theta_o > 0 ? (1 / eta_i_forward) : eta_i_forward;

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

        Float ratio = ComputeScatteringRatio(eta_i_forward, eta_o);
        Float F = FresnelDielectric(Dot(wo, wm), eta_i_forward);
        if (reflect)
        {
            Float fr_ss = F * mf.D(wm) * mf.G(wo, wi) / (4 * CosTheta(wi) * CosTheta(wo));
            Float fr_ms = ratio * (1 - E(wi, eta_o)) * (1 - E(wo, eta_o)) / std::fmax(1e-4f, pi * (1 - E_avg(eta_o)));
            result[i] = fr_ss + fr_ms;
        }
        else
        {
            Float denom = Sqr(Dot(wi, wm) + Dot(wo, wm) / eta_p);
            Float ft_ss =
                (1 - F) * mf.D(wm) * mf.G(wo, wi) * std::abs(Dot(wi, wm) * Dot(wo, wm) / (CosTheta(wi) * CosTheta(wo) * denom));
            Float ft_ms = (1 - ratio) * (1 - E(wi, eta_i)) * (1 - E(wo, eta_o)) / std::fmax(1e-4f, pi * (1 - E_avg(eta_i)));

            if (direction == TransportDirection::ToLight)
            {
                ft_ss /= Sqr(eta_p);
            }
            else
            {
                ft_ms *= Sqr(eta_p);
            }

            result[i] = r[i] * (ft_ss + ft_ms);
        }
    }

    return result;
}

Float DielectricMultiScatteringBxDF::PDF(Vec3 wo, Vec3 wi, TransportDirection direction, BxDF_SamplingFlags flags) const
{
    BulbitNotUsed(direction);
    Float eta_hero = GetHeroEta(eta);
    if (eta_hero == 1 || mf.EffectivelySmooth())
    {
        return 0;
    }

    Float cos_theta_o = CosTheta(wo);
    Float cos_theta_i = CosTheta(wi);

    Float eta_p = 1;
    bool reflect = cos_theta_i * cos_theta_o > 0;
    if (!reflect)
    {
        eta_p = cos_theta_o > 0 ? eta_hero : (1 / eta_hero);
    }

    Float eta_o = cos_theta_o > 0 ? eta_hero : (1 / eta_hero);

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

    Float ratio = ComputeScatteringRatio(eta_hero, eta_o);
    Float E_o = Clamp(E(wo, eta_o), 1e-4f, 1 - 1e-4f);
    Float R = FresnelDielectric(Dot(wo, wm), eta_hero);
    Float T = 1 - R;

    Float pr_ss = R;
    Float pt_ss = T;
    Float pr_ms = ratio;
    Float pt_ms = 1 - ratio;
    if (!(flags & BxDF_SamplingFlags::Reflection)) pr_ss = pr_ms = 0;
    if (!(flags & BxDF_SamplingFlags::Transmission)) pt_ss = pt_ms = 0;
    if (pr_ss == 0 && pt_ss == 0 && pr_ms == 0 && pt_ms == 0)
    {
        return 0;
    }

    Float p_ss_sum = pr_ss + pt_ss;
    Float p_ms_sum = pr_ms + pt_ms;
    pr_ss /= p_ss_sum;
    pt_ss /= p_ss_sum;
    pr_ms /= p_ms_sum;
    pt_ms /= p_ms_sum;

    if (reflect)
    {
        Float pdf_ss = pr_ss * mf.PDF(wo, wm) / (4 * AbsDot(wo, wm));
        Float pdf_ms = pr_ms * CosineHemispherePDF(AbsCosTheta(wi));
        return Lerp(pdf_ms, pdf_ss, E_o);
    }

    Float dwm_dwi = AbsDot(wi, wm) / Sqr(Dot(wi, wm) + Dot(wo, wm) / eta_p);
    Float pdf_ss = pt_ss * mf.PDF(wo, wm) * dwm_dwi;
    Float pdf_ms = pt_ms * CosineHemispherePDF(AbsCosTheta(wi));
    return Lerp(pdf_ms, pdf_ss, E_o);
}

bool DielectricMultiScatteringBxDF::Sample_f(
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

    Float eta_o = CosTheta(wo) > 0 ? eta_hero : (1 / eta_hero);
    Float eta_i = CosTheta(wo) > 0 ? (1 / eta_hero) : eta_hero;

    Float ratio = ComputeScatteringRatio(eta_hero, eta_o);
    Float E_o = Clamp(E(wo, eta_o), 1e-4f, 1 - 1e-4f);
    if (u0 < E_o)
    {
        Vec3 wm = mf.Sample_Wm(wo, u12);
        wm.Normalize();
        if (Dot(wm, z_axis) < 0)
        {
            wm.Negate();
        }

        Float R = FresnelDielectric(Dot(wo, wm), eta_hero);
        Float T = 1 - R;

        Float pr_ss = R, pt_ss = T;
        Float pr_ms = ratio, pt_ms = 1 - ratio;
        if (!(flags & BxDF_SamplingFlags::Reflection)) pr_ss = pr_ms = 0;
        if (!(flags & BxDF_SamplingFlags::Transmission)) pt_ss = pt_ms = 0;
        if (pr_ss == 0 && pt_ss == 0)
        {
            return false;
        }

        Float p_ss_sum = pr_ss + pt_ss;
        Float p_ms_sum = pr_ms + pt_ms;
        pr_ss /= p_ss_sum;
        pt_ss /= p_ss_sum;
        pr_ms /= p_ms_sum;
        pt_ms /= p_ms_sum;

        u0 /= E_o;
        if (u0 < pr_ss)
        {
            Vec3 wi = Reflect(wo, wm);
            if (!SameHemisphere(wo, wi))
            {
                return false;
            }

            Float pdf_ss = pr_ss * mf.PDF(wo, wm) / (4 * AbsDot(wo, wm));
            Float pdf_ms = pr_ms * CosineHemispherePDF(AbsCosTheta(wi));
            Float pdf = Lerp(pdf_ms, pdf_ss, E_o);

            SpectrumSample fr_ss(R * mf.D(wm) * mf.G(wo, wi) / (4 * CosTheta(wi) * CosTheta(wo)));
            SpectrumSample fr_ms(ratio * (1 - E(wi, eta_o)) * (1 - E(wo, eta_o)) / std::fmax(1e-4f, pi * (1 - E_avg(eta_o))));
            SpectrumSample fs = fr_ss + fr_ms;
            *sample = BSDFSample(fs, wi, pdf, BxDF_Flags::GlossyReflection);
        }
        else
        {
            Vec3 wi;
            bool tir = !Refract(&wi, wo, wm, eta_hero);
            if (SameHemisphere(wo, wi) || wi.z == 0 || tir)
            {
                return false;
            }

            Float eta_p = CosTheta(wo) > 0 ? eta_hero : (1 / eta_hero);
            Float denom = Sqr(Dot(wi, wm) + Dot(wo, wm) / eta_p);
            Float dwm_dwi = AbsDot(wi, wm) / denom;
            Float pdf_ss = pt_ss * mf.PDF(wo, wm) * dwm_dwi;
            Float pdf_ms = pt_ms * CosineHemispherePDF(AbsCosTheta(wi));
            Float pdf = Lerp(pdf_ms, pdf_ss, E_o);

            SpectrumSample ft_ss(
                T * mf.D(wm) * mf.G(wo, wi) * std::abs(Dot(wi, wm) * Dot(wo, wm) / (CosTheta(wi) * CosTheta(wo) * denom))
            );
            SpectrumSample ft_ms(
                (1 - ratio) * (1 - E(wi, eta_i)) * (1 - E(wo, eta_o)) / std::fmax(1e-4f, pi * (1 - E_avg(eta_i)))
            );

            if (direction == TransportDirection::ToLight)
            {
                ft_ss /= Sqr(eta_p);
            }
            else
            {
                ft_ms *= Sqr(eta_p);
            }

            SpectrumSample fs = r * (ft_ss + ft_ms);
            *sample = BSDFSample(fs, wi, pdf, BxDF_Flags::GlossyTransmission, eta_p);
        }

        return true;
    }

    Float pr_ms = ratio;
    Float pt_ms = 1 - ratio;
    if (!(flags & BxDF_SamplingFlags::Reflection)) pr_ms = 0;
    if (!(flags & BxDF_SamplingFlags::Transmission)) pt_ms = 0;
    if (pr_ms == 0 && pt_ms == 0)
    {
        return false;
    }

    Float p_ms_sum = pr_ms + pt_ms;
    pr_ms /= p_ms_sum;
    pt_ms /= p_ms_sum;

    u0 = (u0 - E_o) / (1 - E_o);
    if (u0 < pr_ms)
    {
        Vec3 wi = SampleCosineHemisphere(u12);
        if (CosTheta(wo) < 0)
        {
            wi.Negate();
        }

        Vec3 wm = Normalize(wi + wo);
        Float R = FresnelDielectric(Dot(wo, wm), eta_hero);

        Float pr_ss = R;
        Float pt_ss = 1 - R;
        if (!(flags & BxDF_SamplingFlags::Reflection)) pr_ss = 0;
        if (!(flags & BxDF_SamplingFlags::Transmission)) pt_ss = 0;
        Float p_ss_sum = pr_ss + pt_ss;
        pr_ss /= p_ss_sum;
        pt_ss /= p_ss_sum;

        Float pdf_ss = pr_ss * mf.PDF(wo, wm) / (4 * AbsDot(wo, wm));
        Float pdf_ms = pr_ms * CosineHemispherePDF(AbsCosTheta(wi));
        Float pdf = Lerp(pdf_ms, pdf_ss, E_o);

        SpectrumSample fr_ss(R * mf.D(wm) * mf.G(wo, wi) / (4 * CosTheta(wi) * CosTheta(wo)));
        SpectrumSample fr_ms(ratio * (1 - E(wi, eta_o)) * (1 - E(wo, eta_o)) / std::fmax(1e-4f, pi * (1 - E_avg(eta_o))));
        SpectrumSample fs = fr_ss + fr_ms;
        *sample = BSDFSample(fs, wi, pdf, BxDF_Flags::DiffuseReflection);
    }
    else
    {
        Vec3 wi = SampleCosineHemisphere(u12);
        if (CosTheta(wo) > 0)
        {
            wi.Negate();
        }

        Vec3 wm = Normalize(wi * eta_o + wo);
        if (Dot(wm, z_axis) < 0)
        {
            wm.Negate();
        }

        Float R = FresnelDielectric(Dot(wo, wm), eta_hero);
        Float T = 1 - R;

        Float pr_ss = R;
        Float pt_ss = 1 - R;
        if (!(flags & BxDF_SamplingFlags::Reflection)) pr_ss = 0;
        if (!(flags & BxDF_SamplingFlags::Transmission)) pt_ss = 0;
        Float p_ss_sum = pr_ss + pt_ss;
        pr_ss /= p_ss_sum;
        pt_ss /= p_ss_sum;

        Float denom = Sqr(Dot(wi, wm) + Dot(wo, wm) / eta_o);
        Float dwm_dwi = AbsDot(wi, wm) / denom;

        Float pdf_ss = pt_ss * mf.PDF(wo, wm) * dwm_dwi;
        Float pdf_ms = pt_ms * CosineHemispherePDF(AbsCosTheta(wi));
        Float pdf = Lerp(pdf_ms, pdf_ss, E_o);

        SpectrumSample ft_ss(
            T * mf.D(wm) * mf.G(wo, wi) * std::abs(Dot(wi, wm) * Dot(wo, wm) / (CosTheta(wi) * CosTheta(wo) * denom))
        );
        SpectrumSample ft_ms((1 - ratio) * (1 - E(wi, eta_i)) * (1 - E(wo, eta_o)) / std::fmax(1e-4f, pi * (1 - E_avg(eta_i))));

        if (direction == TransportDirection::ToLight)
        {
            ft_ss /= Sqr(eta_o);
        }
        else
        {
            ft_ms *= Sqr(eta_o);
        }

        SpectrumSample fs = r * (ft_ss + ft_ms);
        *sample = BSDFSample(fs, wi, pdf, BxDF_Flags::DiffuseTransmission, eta_o);
    }

    return true;
}

void DielectricMultiScatteringBxDF::Regularize()
{
    mf.Regularize();
}

} // namespace bulbit
