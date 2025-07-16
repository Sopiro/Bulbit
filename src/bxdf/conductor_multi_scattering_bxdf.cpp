#include "bulbit/bxdfs.h"
#include "bulbit/scattering.h"

namespace bulbit
{

Spectrum ConductorMultiScatteringBxDF::f(Vec3 wo, Vec3 wi, TransportDirection direction) const
{
    BulbitNotUsed(direction);

    if (!SameHemisphere(wo, wi) || mf.EffectivelySmooth())
    {
        return Spectrum::black;
    }

    Float cos_theta_o = AbsCosTheta(wo);
    Float cos_theta_i = AbsCosTheta(wi);
    if (cos_theta_i == 0 || cos_theta_o == 0)
    {
        return Spectrum::black;
    }

    Vec3 wm = wo + wi;
    if (Length2(wm) == 0)
    {
        return Spectrum::black;
    }
    wm.Normalize();

    Spectrum F = FresnelComplex(AbsDot(wo, wm), eta, k);
    Spectrum f_ss = F * mf.D(wm) * mf.G(wo, wi) / (4 * cos_theta_i * cos_theta_o);

    Float E_avg = mf.E_avg();
    Spectrum fresnel_avg = FresnelConductorAverage(eta, k);
    Spectrum fresnel_ms = fresnel_avg * fresnel_avg * E_avg / (Spectrum(1) - fresnel_avg * (1 - E_avg));

    Spectrum f_ms = fresnel_ms * ((1 - mf.E(wo)) * (1 - mf.E(wi))) / (pi * (1 - E_avg));

    return f_ss + f_ms;
}

Float ConductorMultiScatteringBxDF::PDF(Vec3 wo, Vec3 wi, TransportDirection direction, BxDF_SamplingFlags flags) const
{
    BulbitNotUsed(direction);

    if (!(flags & BxDF_SamplingFlags::Reflection))
    {
        return 0;
    }

    if (!SameHemisphere(wo, wi) || mf.EffectivelySmooth())
    {
        return 0;
    }

    Vec3 wm = wo + wi;
    if (Length2(wm) == 0)
    {
        return 0;
    }
    wm.Normalize();

    if (Dot(wm, z_axis) < 0)
    {
        wm.Negate();
    }

    Float pdf_ss = mf.PDF(wo, wm) / (4 * AbsDot(wo, wm));
    Float pdf_ms = CosineHemispherePDF(wi.z);
    Float pdf = Lerp(pdf_ms, pdf_ss, mf.E(wo));

    return pdf;
}

bool ConductorMultiScatteringBxDF::Sample_f(
    BSDFSample* sample, Vec3 wo, Float u0, Point2 u12, TransportDirection direction, BxDF_SamplingFlags flags
) const
{
    BulbitNotUsed(u0);
    BulbitNotUsed(direction);

    if (!(flags & BxDF_SamplingFlags::Reflection))
    {
        return false;
    }

    if (mf.EffectivelySmooth())
    {
        // Sample perfect specular conductor BRDF
        Vec3 wi(-wo.x, -wo.y, wo.z);
        Spectrum f = FresnelComplex(AbsCosTheta(wi), eta, k) / AbsCosTheta(wi);
        *sample = BSDFSample(f, wi, 1, BxDF_Flags::SpecularReflection);
        return true;
    }

    if (wo.z == 0)
    {
        return false;
    }

    Vec3 wm;
    Vec3 wi;
    BxDF_Flags flag;

    Float E = mf.E(wo);
    if (u0 < E)
    {
        // Sample rough conductor BRDF
        wm = mf.Sample_Wm(wo, u12);
        wi = Reflect(wo, wm);
        flag = BxDF_Flags::GlossyReflection;
    }
    else
    {
        // Sample multi-scattering lobe
        wi = SampleCosineHemisphere(u12);
        wm = Normalize(wo + wi);
        flag = BxDF_Flags::DiffuseReflection;
    }

    if (!SameHemisphere(wo, wi))
    {
        return false;
    }

    Float cos_theta_o = AbsCosTheta(wo);
    Float cos_theta_i = AbsCosTheta(wi);
    if (cos_theta_i == 0 || cos_theta_o == 0)
    {
        return false;
    }

    Spectrum F = FresnelComplex(AbsDot(wo, wm), eta, k);
    Spectrum f_ss = F * mf.D(wm) * mf.G(wo, wi) / (4 * cos_theta_i * cos_theta_o);

    Float E_avg = mf.E_avg();
    Spectrum fresnel_avg = FresnelConductorAverage(eta, k);
    Spectrum fresnel_ms = fresnel_avg * fresnel_avg * E_avg / (Spectrum(1) - fresnel_avg * (1 - E_avg));

    Spectrum f_ms = fresnel_ms * ((1 - mf.E(wo)) * (1 - mf.E(wi))) / (pi * (1 - E_avg));

    Float pdf_ss = mf.PDF(wo, wm) / (4 * AbsDot(wo, wm));
    Float pdf_ms = CosineHemispherePDF(wi.z);
    Float pdf = Lerp(pdf_ms, pdf_ss, E);

    *sample = BSDFSample(f_ss + f_ms, wi, pdf, flag);
    return true;
}

void ConductorMultiScatteringBxDF::Regularize()
{
    mf.Regularize();
}

} // namespace bulbit
