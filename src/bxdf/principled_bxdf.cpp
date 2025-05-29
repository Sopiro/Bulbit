#include "bulbit/bxdfs.h"
#include "bulbit/frame.h"
#include "bulbit/hash.h"
#include "bulbit/sampling.h"
#include "bulbit/scattering.h"

namespace bulbit
{

Spectrum PrincipledBxDF::f(const Vec3& wo, const Vec3& wi) const
{
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

    Float F_cc = clearcoat * FresnelDielectric(Dot(wo, wm), PrincipledBxDF::default_clearcoat_ior);
    Float T_cc = 1 - F_cc;

    Spectrum F_d = Spectrum(FresnelDielectric(Dot(wo, wm), eta));
    Spectrum F_c = F_Schlick(color, Dot(wi, wm));

    Spectrum F = Lerp(F_d, F_c, metallic);
    Spectrum T = Spectrum(1) - F;

    Spectrum f(0);

    if (reflect)
    {
        // Add clearcoat reflection
        Float denom = std::abs(4 * cos_theta_i * cos_theta_o);
        f += F_cc * clearcoat_color * mf_clearcoat.D(wm) * mf_clearcoat.G(wo, wi) / denom;

#if 1
        // Revisiting Physically Based Shading at Imageworks (Christopher Kulla and Alejandro Conty)
        // https://blog.selfshadow.com/publications/s2017-shading-course/imageworks/s2017_pbs_imageworks_slides_v2.pdf
        Float E_avg = mf.rho_avg();

        Spectrum f0 = F0(eta, color, metallic);
        Spectrum fresnel_avg = F_avg_Schlick(f0);
        Spectrum fresnel_ms = fresnel_avg * fresnel_avg * E_avg / (Spectrum(1) - fresnel_avg * (1 - E_avg));

        Spectrum f_ms = fresnel_ms * ((1 - mf.rho(wo)) * (1 - mf.rho(wi))) / (pi * (1 - E_avg));

        // Add dielectric reflection and metal reflection
        f += T_cc * (F * mf.D(wm) * mf.G(wo, wi) / denom + f_ms);
#else
        // Practical multiple scattering compensation for microfacet models (Emmanuel Turquin)
        // https://blog.selfshadow.com/publications/turquin/ms_comp_final.pdf
        Spectrum f0 = F0(eta, color, metallic);

        // Add dielectric reflection and metal reflection
        Spectrum rho = T_cc * F * mf.D(wm) * mf.G(wo, wi) / denom;
        f += (Spectrum(1) + f0 * (1 / mf.rho(wo) - 1)) * rho;
#endif

        if (sheen > 0)
        {
            // Add sheen reflection
            f += sheen * (1 - transmission) * (1 - metallic) * T_cc * T * sheen_color * mf_sheen.D(wm) * mf_sheen.G(wo, wi) /
                 denom;

            Float sheen_reflectance = mf_sheen.rho(wo);

            // Add diffuse reflection
            f += (1 - sheen * sheen_reflectance) * (1 - transmission) * (1 - metallic) * T_cc * T * color * inv_pi;
        }
        else
        {
            // Add diffuse reflection
            f += (1 - transmission) * (1 - metallic) * T_cc * T * color * inv_pi;
        }
    }
    else
    {
        // Add dielectric transmission
        Float denom = Sqr(Dot(wi, wm) + Dot(wo, wm) / eta_p) * cos_theta_i * cos_theta_o;
        f += transmission * T_cc * T * Sqrt(color) * mf.D(wm) * mf.G(wo, wi) * std::abs(Dot(wi, wm) * Dot(wo, wm) / denom);

        // Handle solid angle squeezing
        f /= Sqr(eta_p);
    }

    return f;
}

Float PrincipledBxDF::PDF(Vec3 wo, Vec3 wi, BxDF_SamplingFlags flags) const
{
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

    Float p_sum = pr + pt;
    pr /= p_sum;
    pt /= p_sum;

    Float cc = clearcoat * FresnelDielectric(Dot(wo, wm), PrincipledBxDF::default_clearcoat_ior);

    Float pdf = 0;

    if (reflect)
    {
        // Add clearcoat BRDF
        pdf += cc * mf_clearcoat.PDF(wo, wm) / (4 * AbsDot(wo, wm));

        // Add dielectric BRDF and metal BRDF
        pdf += (1 - cc) * (metallic + (1 - metallic) * pr) * mf.PDF(wo, wm) / (4 * AbsDot(wo, wm));

        // Add diffuse BRDF
        pdf += (1 - cc) * (1 - metallic) * pt * (1 - transmission) * AbsCosTheta(wi) * inv_pi;
    }
    else
    {
        // Add dielectric BTDF
        Float dwm_dwi = AbsDot(wi, wm) / Sqr(Dot(wi, wm) + Dot(wo, wm) / eta_p);
        pdf += (1 - cc) * (1 - metallic) * pt * transmission * mf.PDF(wo, wm) * dwm_dwi;
    }

    return pdf;
}

bool PrincipledBxDF::Sample_f(BSDFSample* sample, Vec3 wo, Float u0, Point2 u12, BxDF_SamplingFlags flags) const
{
    BxDF_Flags flag;
    Vec3 wi;
    Float eta_p = 1;

    Vec3 wm = mf_clearcoat.Sample_Wm(wo, u12);

    Float cc = clearcoat * FresnelDielectric(Dot(wo, wm), PrincipledBxDF::default_clearcoat_ior);

    if (u0 < cc)
    {
        // Sample clearcoat
        wi = Reflect(wo, wm);
        if (!SameHemisphere(wo, wi))
        {
            return false;
        }

        flag = BxDF_Flags::GlossyReflection;
    }
    else
    {
        // Renormalize
        u0 = (u0 - cc) / (1 - cc);
        u12[0] = (u12[0] - cc) / (1 - cc);
        u12[1] = (u12[1] - cc) / (1 - cc);

        wm = mf.Sample_Wm(wo, u12);

        if (u0 < metallic)
        {
            // Sample metal
            wi = Reflect(wo, wm);
            if (!SameHemisphere(wo, wi))
            {
                return false;
            }

            flag = BxDF_Flags::GlossyReflection;
        }
        else
        {
            // Sample dielectric
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

            Float p_sum = pr + pt;
            pr /= p_sum;
            pt /= p_sum;

            // Renormalize
            u0 = (u0 - metallic) / (1 - metallic);

            if (u0 < pr)
            {
                // Sample glossy reflection
                wi = Reflect(wo, wm);
                if (!SameHemisphere(wo, wi))
                {
                    return false;
                }

                flag = BxDF_Flags::GlossyReflection;
            }
            else
            {
                // Renormalize
                u0 = (u0 - pr) / (pt);

                if (u0 < transmission)
                {
                    // Sample glossy transmission

                    // Total internal reflection
                    bool tir = !Refract(&wi, wo, wm, eta, &eta_p);
                    if (SameHemisphere(wo, wi) || wi.z == 0 || tir)
                    {
                        return false;
                    }

                    flag = BxDF_Flags::GlossyTransmission;
                }
                else
                {
                    // Sample diffuse reflection
                    wi = SampleCosineHemisphere(u12);
                    flag = BxDF_Flags::DiffuseReflection;
                }
            }
        }
    }

    *sample = BSDFSample(f(wo, wi), wi, PDF(wo, wi), flag, eta_p);
    return true;
}

void PrincipledBxDF::Regularize()
{
    mf.Regularize();
}

} // namespace bulbit
