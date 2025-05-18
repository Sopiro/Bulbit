#include "bulbit/bxdfs.h"
#include "bulbit/frame.h"
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

    Spectrum F_d = Spectrum(FresnelDielectric(Dot(wo, wm), eta));
    Spectrum F_c = F_Schlick(color, Dot(wi, wm));

    Spectrum F = Lerp(F_d, F_c, metallic);
    Spectrum T = Spectrum(1) - F;

    if (reflect)
    {
        // Add dielectric reflection and metal reflection
        Float denom = std::abs(4 * cos_theta_i * cos_theta_o);
        Spectrum f = F * mf.D(wm) * mf.G(wo, wi) / denom;

        // Add diffuse reflection
        f += (1 - transmission) * (1 - metallic) * T * color * inv_pi;

        return f;
    }
    else
    {
        // Add dielectric transmission
        Float denom = Sqr(Dot(wi, wm) + Dot(wo, wm) / eta_p) * cos_theta_i * cos_theta_o;
        Spectrum f = transmission * Sqrt(color) * T * mf.D(wm) * mf.G(wo, wi) * std::abs(Dot(wi, wm) * Dot(wo, wm) / denom);

        // Handle solid angle squeezing
        f /= Sqr(eta_p);

        return f;
    }
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
    Float pt = T * (1 - metallic);
    if (!(flags & BxDF_SamplingFlags::Reflection)) pr = 0;
    if (!(flags & BxDF_SamplingFlags::Transmission)) pt = 0;
    if (pr == 0 && pt == 0)
    {
        return 0;
    }

    Float p_sum = pr + pt;
    pr /= p_sum;
    pt /= p_sum;

    if (reflect)
    {
        // Add dielectric BRDF and metal BRDF
        Float pdf = pr * mf.PDF(wo, wm) / (4 * AbsDot(wo, wm));

        // Add diffuse BRDF
        pdf += pt * (1 - metallic) * (1 - transmission) * AbsCosTheta(wi) * inv_pi;

        return pdf;
    }
    else
    {
        // Add dielectric BTDF
        Float dwm_dwi = AbsDot(wi, wm) / Sqr(Dot(wi, wm) + Dot(wo, wm) / eta_p);
        Float pdf = pt * (1 - metallic) * transmission * mf.PDF(wo, wm) * dwm_dwi;

        return pdf;
    }
}

bool PrincipledBxDF::Sample_f(BSDFSample* sample, Vec3 wo, Float u0, Point2 u12, BxDF_SamplingFlags flags) const
{
    BxDF_Flags flag;
    Vec3 wi;
    Float eta_p = 1;

    if (u0 < metallic)
    {
        Vec3 wm = mf.Sample_Wm(wo, u12);
        wi = Reflect(wo, wm);
        flag = BxDF_Flags::GlossyReflection;
    }
    else
    {
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

    *sample = BSDFSample(f(wo, wi), wi, PDF(wo, wi), flag, eta_p);
    return true;
}

void PrincipledBxDF::Regularize()
{
    mf.Regularize();
}

} // namespace bulbit
