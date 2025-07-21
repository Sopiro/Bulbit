#include "bulbit/bxdfs.h"
#include "bulbit/frame.h"
#include "bulbit/sampling.h"
#include "bulbit/scattering.h"

namespace bulbit
{

Spectrum DielectricMultiScatteringBxDF::f(Vec3 wo, Vec3 wi, TransportDirection direction) const
{
    if (eta == 1 || mf.EffectivelySmooth())
    {
        return Spectrum::black;
    }

    // Evaluate rough dielectric BSDF
    Float cos_theta_o = CosTheta(wo);
    Float cos_theta_i = CosTheta(wi);

    Float eta_p = 1;
    bool reflect = cos_theta_i * cos_theta_o > 0;
    if (!reflect)
    {
        // Flip interface
        eta_p = cos_theta_o > 0 ? eta : (1 / eta);
    }

    Float eta_o, eta_i;
    if (cos_theta_o > 0)
    {
        eta_o = eta;
        eta_i = 1 / eta;
    }
    else
    {
        eta_o = 1 / eta;
        eta_i = eta;
    }

    // Microfacet normal
    Vec3 wm = wi * eta_p + wo;
    if (cos_theta_i == 0 || cos_theta_o == 0 || Length2(wm) == 0)
    {
        return Spectrum::black;
    }

    wm.Normalize();
    if (Dot(wm, z_axis) < 0)
    {
        wm.Negate();
    }

    // Discard backfacing microfacets
    if (Dot(wm, wi) * cos_theta_i < 0 || Dot(wm, wo) * cos_theta_o < 0)
    {
        return Spectrum::black;
    }

    Float ratio = ComputeScatteringRatio(eta_o);

    Float F = FresnelDielectric(Dot(wo, wm), eta);
    if (reflect)
    {
        Spectrum fr_ss(F * mf.D(wm) * mf.G(wo, wi) / (4 * CosTheta(wi) * CosTheta(wo)));
        Spectrum fr_ms(ratio * (1 - E(wi, eta_o)) * (1 - E(wo, eta_o)) / std::fmax(1e-4f, pi * (1 - E_avg(eta_o))));

        return fr_ss + fr_ms;
    }
    else
    {
        Float denom = Sqr(Dot(wi, wm) + Dot(wo, wm) / eta_p);

        Spectrum ft_ss(
            (1 - F) * mf.D(wm) * mf.G(wo, wi) * std::abs(Dot(wi, wm) * Dot(wo, wm) / (CosTheta(wi) * CosTheta(wo) * denom))
        );
        Spectrum ft_ms((1 - ratio) * (1 - E(wi, eta_i)) * (1 - E(wo, eta_o)) / std::fmax(1e-4f, pi * (1 - E_avg(eta_i))));

        // Handle solid angle squeezing
        if (direction == TransportDirection::ToLight)
        {
            ft_ss /= Sqr(eta_p);
        }
        else
        {
            ft_ms *= Sqr(eta_p);
        }

        return r * (ft_ss + ft_ms);
    }
}

Float DielectricMultiScatteringBxDF::PDF(Vec3 wo, Vec3 wi, TransportDirection direction, BxDF_SamplingFlags flags) const
{
    BulbitNotUsed(direction);

    if (eta == 1 || mf.EffectivelySmooth())
    {
        return 0;
    }

    // Evaluate sampling PDF of rough dielectric BSDF
    // Compute generalized half vector wm
    Float cos_theta_o = CosTheta(wo);
    Float cos_theta_i = CosTheta(wi);

    Float eta_p = 1;
    bool reflect = cos_theta_i * cos_theta_o > 0;
    if (!reflect)
    {
        eta_p = cos_theta_o > 0 ? eta : (1 / eta);
    }

    Float eta_o;
    if (cos_theta_o > 0)
    {
        eta_o = eta;
    }
    else
    {
        eta_o = 1 / eta;
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

    // Discard backfacing microfacets
    if (Dot(wm, wi) * cos_theta_i < 0 || Dot(wm, wo) * cos_theta_o < 0)
    {
        return 0;
    }

    Float ratio = ComputeScatteringRatio(eta_o);

    Float E_o = Clamp(E(wo, eta_o), 1e-4f, 1 - 1e-4f);

    // Determine Fresnel F of rough dielectric boundary
    Float R = FresnelDielectric(Dot(wo, wm), eta);
    Float T = 1 - R;

    // Compute sampling probabilities for reflection and transmission
    Float pr_ss = R, pt_ss = T;
    Float pr_ms = ratio, pt_ms = 1 - ratio;
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

    Float pdf;
    if (reflect)
    {
        // Compute PDF of rough dielectric reflection
        Float pdf_ss = pr_ss * mf.PDF(wo, wm) / (4 * AbsDot(wo, wm));
        Float pdf_ms = pr_ms * CosineHemispherePDF(AbsCosTheta(wi));

        pdf = Lerp(pdf_ms, pdf_ss, E_o);
    }
    else
    {
        // Compute PDF of rough dielectric transmission
        Float dwm_dwi = AbsDot(wi, wm) / Sqr(Dot(wi, wm) + Dot(wo, wm) / eta_p);
        Float pdf_ss = pt_ss * mf.PDF(wo, wm) * dwm_dwi;
        Float pdf_ms = pt_ms * CosineHemispherePDF(AbsCosTheta(wi));

        pdf = Lerp(pdf_ms, pdf_ss, E_o);
    }

    return pdf;
}

bool DielectricMultiScatteringBxDF::Sample_f(
    BSDFSample* sample, Vec3 wo, Float u0, Point2 u12, TransportDirection direction, BxDF_SamplingFlags flags
) const
{
    if (eta == 1 || mf.EffectivelySmooth())
    {
        // Sample perfect specular dielectric BSDF
        Float R = FresnelDielectric(CosTheta(wo), eta);
        Float T = 1 - R;

        // Compute sampling probabilities for reflection and transmission
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
            // Sample perfect specular dielectric BRDF
            Vec3 wi(-wo.x, -wo.y, wo.z);

            Spectrum fr(R / AbsCosTheta(wi));
            *sample = BSDFSample(fr, wi, pr / (pr + pt), BxDF_Flags::SpecularReflection);
        }
        else
        {
            // Sample perfect specular dielectric BTDF
            // Compute ray direction for specular transmission
            Vec3 wi;
            Float eta_p;
            if (!Refract(&wi, wo, z_axis, eta, &eta_p))
            {
                return false;
            }

            Spectrum ft(T / AbsCosTheta(wi));

            // Handle solid angle squeezing
            if (direction == TransportDirection::ToLight)
            {
                ft /= Sqr(eta_p);
            }

            *sample = BSDFSample(r * ft, wi, pt / (pr + pt), BxDF_Flags::SpecularTransmission, eta_p);
        }

        return true;
    }

    Float eta_o, eta_i;
    if (CosTheta(wo) > 0)
    {
        eta_o = eta;
        eta_i = 1 / eta;
    }
    else
    {
        eta_o = 1 / eta;
        eta_i = eta;
    }

    Float ratio = ComputeScatteringRatio(eta_o);

    Float E_o = Clamp(E(wo, eta_o), 1e-4f, 1 - 1e-4f);
    if (u0 < E_o)
    {
        // Sample single-scattering rough dielectric BSDF
        Vec3 wm = mf.Sample_Wm(wo, u12);

        wm.Normalize();
        if (Dot(wm, z_axis) < 0)
        {
            wm.Negate();
        }

        Float R = FresnelDielectric(Dot(wo, wm), eta);
        Float T = 1 - R;

        // Compute sampling probabilities for reflection and transmission
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

        // Renormalize
        u0 /= E_o;
        if (u0 < pr_ss)
        {
            // Sample reflection at rough dielectric interface
            Vec3 wi = Reflect(wo, wm);
            if (!SameHemisphere(wo, wi))
            {
                return false;
            }

            // Compute PDF of rough dielectric reflection
            Float pdf_ss = pr_ss * mf.PDF(wo, wm) / (4 * AbsDot(wo, wm));
            Float pdf_ms = pr_ms * CosineHemispherePDF(AbsCosTheta(wi));
            Float pdf = Lerp(pdf_ms, pdf_ss, E_o);

            Spectrum fr_ss(R * mf.D(wm) * mf.G(wo, wi) / (4 * CosTheta(wi) * CosTheta(wo)));
            Spectrum fr_ms(ratio * (1 - E(wi, eta_o)) * (1 - E(wo, eta_o)) / std::fmax(1e-4f, pi * (1 - E_avg(eta_o))));

            *sample = BSDFSample(fr_ss + fr_ms, wi, pdf, BxDF_Flags::GlossyReflection);
        }
        else
        {
            // Sample transmission at rough dielectric interface
            Vec3 wi;
            bool tir = !Refract(&wi, wo, wm, eta);
            if (SameHemisphere(wo, wi) || wi.z == 0 || tir)
            {
                return false;
            }

            Float eta_p = CosTheta(wo) > 0 ? eta : (1 / eta);

            // Compute PDF of rough dielectric transmission
            Float denom = Sqr(Dot(wi, wm) + Dot(wo, wm) / eta_p);
            Float dwm_dwi = AbsDot(wi, wm) / denom;
            Float pdf_ss = pt_ss * mf.PDF(wo, wm) * dwm_dwi;
            Float pdf_ms = pt_ms * CosineHemispherePDF(AbsCosTheta(wi));
            Float pdf = Lerp(pdf_ms, pdf_ss, E_o);

            Spectrum ft_ss(
                T * mf.D(wm) * mf.G(wo, wi) * std::abs(Dot(wi, wm) * Dot(wo, wm) / (CosTheta(wi) * CosTheta(wo) * denom))
            );
            Spectrum ft_ms((1 - ratio) * (1 - E(wi, eta_i)) * (1 - E(wo, eta_o)) / std::fmax(1e-4f, pi * (1 - E_avg(eta_i))));

            // Handle solid angle squeezing
            if (direction == TransportDirection::ToLight)
            {
                ft_ss /= Sqr(eta_p);
            }
            else
            {
                ft_ms *= Sqr(eta_p);
            }

            *sample = BSDFSample(r * (ft_ss + ft_ms), wi, pdf, BxDF_Flags::GlossyTransmission, eta_p);
        }

        return true;
    }
    else
    {
        // Sample multi-scattering lobe

        // Compute sampling probabilities for reflection and transmission
        Float pr_ms = ratio, pt_ms = 1 - ratio;
        if (!(flags & BxDF_SamplingFlags::Reflection)) pr_ms = 0;
        if (!(flags & BxDF_SamplingFlags::Transmission)) pt_ms = 0;
        if (pr_ms == 0 && pt_ms == 0)
        {
            return false;
        }

        Float p_ms_sum = pr_ms + pt_ms;
        pr_ms /= p_ms_sum;
        pt_ms /= p_ms_sum;

        // Renormalize
        u0 = (u0 - E_o) / (1 - E_o);
        if (u0 < pr_ms)
        {
            // Sample diffuse reflection
            Vec3 wi = SampleCosineHemisphere(u12);
            if (CosTheta(wo) < 0)
            {
                wi.Negate();
            }

            Vec3 wm = Normalize(wi + wo);

            Float R = FresnelDielectric(Dot(wo, wm), eta);

            Float pr_ss = R, pt_ss = 1 - R;
            if (!(flags & BxDF_SamplingFlags::Reflection)) pr_ss = 0;
            if (!(flags & BxDF_SamplingFlags::Transmission)) pt_ss = 0;
            Float p_ss_sum = pr_ss + pt_ss;
            pr_ss /= p_ss_sum;
            pt_ss /= p_ss_sum;

            Float pdf_ss = pr_ss * mf.PDF(wo, wm) / (4 * AbsDot(wo, wm));
            Float pdf_ms = pr_ms * CosineHemispherePDF(AbsCosTheta(wi));
            Float pdf = Lerp(pdf_ms, pdf_ss, E_o);

            Spectrum fr_ss(R * mf.D(wm) * mf.G(wo, wi) / (4 * CosTheta(wi) * CosTheta(wo)));
            Spectrum fr_ms(ratio * (1 - E(wi, eta_o)) * (1 - E(wo, eta_o)) / std::fmax(1e-4f, pi * (1 - E_avg(eta_o))));

            *sample = BSDFSample(fr_ss + fr_ms, wi, pdf, BxDF_Flags::DiffuseReflection);
        }
        else
        {
            // Sample diffuse transmission

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

            Float R = FresnelDielectric(Dot(wo, wm), eta);
            Float T = 1 - R;

            Float pr_ss = R, pt_ss = 1 - R;
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

            Spectrum ft_ss(
                T * mf.D(wm) * mf.G(wo, wi) * std::abs(Dot(wi, wm) * Dot(wo, wm) / (CosTheta(wi) * CosTheta(wo) * denom))
            );
            Spectrum ft_ms((1 - ratio) * (1 - E(wi, eta_i)) * (1 - E(wo, eta_o)) / std::fmax(1e-4f, pi * (1 - E_avg(eta_i))));

            // Handle solid angle squeezing
            if (direction == TransportDirection::ToLight)
            {
                ft_ss /= Sqr(eta_o);
            }
            else
            {
                ft_ms *= Sqr(eta_o);
            }

            *sample = BSDFSample(r * (ft_ss + ft_ms), wi, pdf, BxDF_Flags::DiffuseTransmission, eta_o);
        }

        return true;
    }
}

void DielectricMultiScatteringBxDF::Regularize()
{
    mf.Regularize();
}

} // namespace bulbit
