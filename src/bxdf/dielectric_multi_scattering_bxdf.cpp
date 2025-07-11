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

    Float eta_i = eta >= 1 ? eta : 1 / eta;
    Float eta_t = 1 / eta_i;
    Float a = (1 - FresnelDielectricAverage(eta_i)) / (1 - E_avg(eta_t));
    Float b = Sqr(eta_i) * (1 - FresnelDielectricAverage(eta_t)) / (1 - E_avg(eta_i));
    Float x = b / (a + b);

    Float ratio = 1 - (eta >= 1 ? x : (1 - x)) * (1 - FresnelDielectricAverage(eta));

    Float F = FresnelDielectric(Dot(wo, wm), eta);
    if (reflect)
    {
        Spectrum fr_ss(F * mf.D(wm) * mf.G(wo, wi) / (4 * CosTheta(wi) * CosTheta(wo)));
        Spectrum fr_ms(ratio * (1 - E(wi, eta)) * (1 - E(wo, eta)) / std::fmax(1e-4f, pi * (1 - E_avg(eta))));

        return fr_ss + fr_ms;
    }
    else
    {
        Float denom = Sqr(Dot(wi, wm) + Dot(wo, wm) / eta);

        Spectrum ft_ss(
            (1 - F) * mf.D(wm) * mf.G(wo, wi) * std::abs(Dot(wi, wm) * Dot(wo, wm) / (CosTheta(wi) * CosTheta(wo) * denom))
        );
        Spectrum ft_ms((1 - ratio) * (1 - E(wi, 1 / eta)) * (1 - E(wo, eta)) / std::fmax(1e-4f, pi * (1 - E_avg(1 / eta))));

        // Handle solid angle squeezing
        if (direction == TransportDirection::ToLight)
        {
            ft_ss /= Sqr(eta);
        }
        else
        {
            ft_ms *= Sqr(eta);
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

    // Determine Fresnel F of rough dielectric boundary
    Float R = FresnelDielectric(Dot(wo, wm), eta);
    Float T = 1 - R;

    // Compute sampling probabilities for reflection and transmission
    Float pr = R;
    Float pt = T;
    if (!(flags & BxDF_SamplingFlags::Reflection)) pr = 0;
    if (!(flags & BxDF_SamplingFlags::Transmission)) pt = 0;
    if (pr == 0 && pt == 0)
    {
        return 0;
    }

    Float eta_i = eta >= 1 ? eta : 1 / eta;
    Float eta_t = 1 / eta_i;
    Float a = (1 - FresnelDielectricAverage(eta_i)) / (1 - E_avg(eta_t));
    Float b = Sqr(eta_i) * (1 - FresnelDielectricAverage(eta_t)) / (1 - E_avg(eta_i));
    Float x = b / (a + b);

    Float ratio = 1 - (eta >= 1 ? x : (1 - x)) * (1 - FresnelDielectricAverage(eta));

    Float reflectance = std::fmin(E(wo, eta), 1 - 1e-4f);

    Float pdf;
    if (reflect)
    {
        // Compute PDF of rough dielectric reflection
        Float pdf_ss = mf.PDF(wo, wm) / (4 * AbsDot(wo, wm)) * pr / (pr + pt);
        Float pdf_ms = ratio * CosineHemispherePDF(AbsCosTheta(wi));

        pdf = Lerp(pdf_ms, pdf_ss, reflectance);
    }
    else
    {
        // Compute PDF of rough dielectric transmission
        Float dwm_dwi = AbsDot(wi, wm) / Sqr(Dot(wi, wm) + Dot(wo, wm) / eta_p);
        Float pdf_ss = mf.PDF(wo, wm) * dwm_dwi * pt / (pr + pt);
        Float pdf_ms = (1 - ratio) * CosineHemispherePDF(AbsCosTheta(wi));

        pdf = Lerp(pdf_ms, pdf_ss, reflectance);
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

    Float eta_i = eta >= 1 ? eta : 1 / eta;
    Float eta_t = 1 / eta_i;
    Float a = (1 - FresnelDielectricAverage(eta_i)) / (1 - E_avg(eta_t));
    Float b = Sqr(eta_i) * (1 - FresnelDielectricAverage(eta_t)) / (1 - E_avg(eta_i));
    Float x = b / (a + b);

    Float ratio = 1 - (eta >= 1 ? x : (1 - x)) * (1 - FresnelDielectricAverage(eta));

    Float reflectance = std::fmin(E(wo, eta), 1 - 1e-4f);
    if (u0 < reflectance)
    {
        // Sample rough dielectric BSDF
        Vec3 wm = mf.Sample_Wm(wo, u12);

        Float R = FresnelDielectric(Dot(wo, wm), eta);
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
            // Sample reflection at rough dielectric interface
            Vec3 wi = Reflect(wo, wm);
            if (!SameHemisphere(wo, wi))
            {
                return false;
            }

            // Compute PDF of rough dielectric reflection
            Float pdf_ss = mf.PDF(wo, wm) / (4 * AbsDot(wo, wm)) * pr / (pr + pt);
            Float pdf_ms = ratio * CosineHemispherePDF(AbsCosTheta(wi));
            Float pdf = Lerp(pdf_ms, pdf_ss, reflectance);

            Spectrum fr_ss(R * mf.D(wm) * mf.G(wo, wi) / (4 * CosTheta(wi) * CosTheta(wo)));
            Spectrum fr_ms(ratio * (1 - E(wi, eta)) * (1 - E(wo, eta)) / std::fmax(1e-4f, pi * (1 - E_avg(eta))));

            *sample = BSDFSample(fr_ss + fr_ms, wi, pdf, BxDF_Flags::GlossyReflection);
        }
        else
        {
            // Sample transmission at rough dielectric interface
            Float eta_p;
            Vec3 wi;

            // Total internal reflection
            bool tir = !Refract(&wi, wo, wm, eta, &eta_p);
            if (SameHemisphere(wo, wi) || wi.z == 0 || tir)
            {
                return false;
            }

            // Compute PDF of rough dielectric transmission
            Float denom = Sqr(Dot(wi, wm) + Dot(wo, wm) / eta_p);
            Float dwm_dwi = AbsDot(wi, wm) / denom;
            Float pdf_ss = mf.PDF(wo, wm) * dwm_dwi * pt / (pr + pt);
            Float pdf_ms = (1 - ratio) * CosineHemispherePDF(AbsCosTheta(wi));
            Float pdf = Lerp(pdf_ms, pdf_ss, reflectance);

            Spectrum ft_ss(
                T * mf.D(wm) * mf.G(wo, wi) * std::abs(Dot(wi, wm) * Dot(wo, wm) / (CosTheta(wi) * CosTheta(wo) * denom))
            );
            Spectrum ft_ms((1 - ratio) * (1 - E(wi, 1 / eta)) * (1 - E(wo, eta)) / std::fmax(1e-4f, pi * (1 - E_avg(1 / eta))));

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

        // Compute sampling probabilities for reflection and transmission of diffuse multi-scattering lobe
        Float pr = ratio;
        Float pt = 1 - ratio;
        if (!(flags & BxDF_SamplingFlags::Reflection)) pr = 0;
        if (!(flags & BxDF_SamplingFlags::Transmission)) pt = 0;
        if (pr == 0 && pt == 0)
        {
            return false;
        }

        Vec3 wi = SampleCosineHemisphere(u12);
        Vec3 wm;

        // Renormalize
        u0 = (u0 - reflectance) / (1 - reflectance);
        if (u0 < pr / (pr + pt))
        {
            // Sample diffuse reflection

            wm = Normalize(wi + wo);

            Float R = FresnelDielectric(Dot(wo, wm), eta);

            Float pdf_ss = R * mf.PDF(wo, wm) / (4 * AbsDot(wo, wm));
            Float pdf_ms = CosineHemispherePDF(AbsCosTheta(wi)) * pr / (pr + pt);
            Float pdf = Lerp(pdf_ms, pdf_ss, reflectance);

            Spectrum fr_ss(R * mf.D(wm) * mf.G(wo, wi) / (4 * CosTheta(wi) * CosTheta(wo)));
            Spectrum fr_ms(ratio * (1 - E(wi, eta)) * (1 - E(wo, eta)) / std::fmax(1e-4f, pi * (1 - E_avg(eta))));

            *sample = BSDFSample(fr_ss + fr_ms, wi, pdf, BxDF_Flags::DiffuseReflection);
        }
        else
        {
            // Sample diffuse transmission

            wi.Negate();

            wm = Normalize(wi * eta + wo);
            if (!SameHemisphere(wo, wm))
            {
                wm.Negate();
            }

            Float T = 1 - FresnelDielectric(Dot(wo, wm), eta);

            Float denom = Sqr(Dot(wi, wm) + Dot(wo, wm) / eta);
            Float dwm_dwi = AbsDot(wi, wm) / denom;

            Float pdf_ss = T * mf.PDF(wo, wm) * dwm_dwi;
            Float pdf_ms = CosineHemispherePDF(AbsCosTheta(wi)) * pt / (pr + pt);
            Float pdf = Lerp(pdf_ms, pdf_ss, reflectance);

            Spectrum ft_ss(
                T * mf.D(wm) * mf.G(wo, wi) * std::abs(Dot(wi, wm) * Dot(wo, wm) / (CosTheta(wi) * CosTheta(wo) * denom))
            );
            Spectrum ft_ms((1 - ratio) * (1 - E(wi, 1 / eta)) * (1 - E(wo, eta)) / std::fmax(1e-4f, pi * (1 - E_avg(1 / eta))));

            // Handle solid angle squeezing
            if (direction == TransportDirection::ToLight)
            {
                ft_ss /= Sqr(eta);
            }
            else
            {
                ft_ms *= Sqr(eta);
            }

            *sample = BSDFSample(r * (ft_ss + ft_ms), wi, pdf, BxDF_Flags::DiffuseTransmission, eta);
        }

        return true;
    }
}

void DielectricMultiScatteringBxDF::Regularize()
{
    mf.Regularize();
}

} // namespace bulbit
