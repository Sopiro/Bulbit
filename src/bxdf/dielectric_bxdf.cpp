#include "bulbit/bxdfs.h"
#include "bulbit/frame.h"
#include "bulbit/sampling.h"
#include "bulbit/scattering.h"

namespace bulbit
{

Spectrum DielectricBxDF::f(Vec3 wo, Vec3 wi, TransportDirection direction) const
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

    Float F = FresnelDielectric(Dot(wo, wm), eta);
    if (reflect)
    {
        // Compute reflection at rough dielectric interface
        Spectrum fr(F * mf.D(wm) * mf.G(wo, wi) / std::abs(4 * cos_theta_i * cos_theta_o));
        return fr;
    }
    else
    {
        // Compute transmission at rough dielectric interface
        Float denom = Sqr(Dot(wi, wm) + Dot(wo, wm) / eta_p) * cos_theta_i * cos_theta_o;
        Float ft = (1 - F) * mf.D(wm) * mf.G(wo, wi) * std::abs(Dot(wi, wm) * Dot(wo, wm) / denom);

        // Handle solid angle squeezing
        if (direction == TransportDirection::ToLight)
        {
            ft /= Sqr(eta_p);
        }

        return r * ft;
    }
}

Float DielectricBxDF::PDF(Vec3 wo, Vec3 wi, TransportDirection direction, BxDF_SamplingFlags flags) const
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

    // Return PDF for rough dielectric
    Float pdf;
    if (reflect)
    {
        // Compute PDF of rough dielectric reflection
        pdf = mf.PDF(wo, wm) / (4 * AbsDot(wo, wm)) * pr / (pr + pt);
    }
    else
    {
        // Compute PDF of rough dielectric transmission
        Float dwm_dwi = AbsDot(wi, wm) / Sqr(Dot(wi, wm) + Dot(wo, wm) / eta_p);
        pdf = mf.PDF(wo, wm) * dwm_dwi * pt / (pr + pt);
    }
    return pdf;
}

bool DielectricBxDF::Sample_f(
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
        Float pdf = mf.PDF(wo, wm) / (4 * AbsDot(wo, wm)) * pr / (pr + pt);

        Spectrum fr(R * mf.D(wm) * mf.G(wo, wi) / (4 * CosTheta(wi) * CosTheta(wo)));
        *sample = BSDFSample(fr, wi, pdf, BxDF_Flags::GlossyReflection);
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
        Float pdf = mf.PDF(wo, wm) * dwm_dwi * pt / (pr + pt);

        Spectrum ft(T * mf.D(wm) * mf.G(wo, wi) * std::abs(Dot(wi, wm) * Dot(wo, wm) / (CosTheta(wi) * CosTheta(wo) * denom)));

        // Handle solid angle squeezing
        if (direction == TransportDirection::ToLight)
        {
            ft /= Sqr(eta_p);
        }

        *sample = BSDFSample(r * ft, wi, pdf, BxDF_Flags::GlossyTransmission, eta_p);
    }

    return true;
}

void DielectricBxDF::Regularize()
{
    mf.Regularize();
}

} // namespace bulbit
