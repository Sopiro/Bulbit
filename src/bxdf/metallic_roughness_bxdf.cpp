#include "bulbit/bxdfs.h"
#include "bulbit/frame.h"
#include "bulbit/sampling.h"

namespace bulbit
{

Spectrum MetallicRoughnessBxDF::f(Vec3 wo, Vec3 wi, TransportDirection direction) const
{
    BulbitNotUsed(direction);

    if (!SameHemisphere(wo, wi))
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

    Spectrum f0 = F0(color, metallic);
    Spectrum F = F_Schlick(f0, Dot(wi, wm));

    Spectrum f_s = F * mf.D(wm) * mf.G(wo, wi) / (4 * cos_theta_i * cos_theta_o);
    Spectrum f_d = (1 - metallic) * (Spectrum(1) - F) * (color * inv_pi);

    return f_d + f_s;
}

Float MetallicRoughnessBxDF::PDF(Vec3 wo, Vec3 wi, TransportDirection direction, BxDF_SamplingFlags flags) const
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

    Float R = FresnelDielectric(Dot(wo, wm), detault_ior);
    Float T = 1 - R;

    Float pr = R;
    Float pt = T * (1 - metallic);
    if (pr == 0 && pt == 0)
    {
        return 0;
    }

    Float p_sum = pr + pt;
    pr /= p_sum;
    pt /= p_sum;

    Float pdf_r = mf.PDF(wo, wm) / (4 * AbsDot(wo, wm));
    Float pdf_d = AbsCosTheta(wi) * inv_pi;

    return pr * pdf_r + pt * pdf_d;
}

bool MetallicRoughnessBxDF::Sample_f(
    BSDFSample* sample, Vec3 wo, Float u0, Point2 u12, TransportDirection direction, BxDF_SamplingFlags flags
) const
{
    BulbitNotUsed(direction);

    if (!(flags & BxDF_SamplingFlags::Reflection))
    {
        return false;
    }

    BulbitAssert(!mf.EffectivelySmooth());

    if (wo.z == 0)
    {
        return false;
    }

    // Sample half vector
    Vec3 wm = mf.Sample_Wm(wo, u12);
    Vec3 wi = Reflect(wo, wm);

    Float R = FresnelDielectric(Dot(wo, wm), detault_ior);
    Float T = 1 - R;

    Float pr = R;
    Float pt = T * (1 - metallic);
    if (pr == 0 && pt == 0)
    {
        return false;
    }

    Float p_sum = pr + pt;
    pr /= p_sum;
    pt /= p_sum;

    BxDF_Flags flag;
    if (u0 < pr)
    {
        // Sample glossy reflection
        if (!SameHemisphere(wo, wi))
        {
            return false;
        }

        flag = BxDF_Flags::GlossyReflection;
    }
    else
    {
        // Sample diffuse reflection
        wi = SampleCosineHemisphere(u12);
        wm = Normalize(wi + wo);

        flag = BxDF_Flags::DiffuseReflection;
    }

    Float cos_theta_o = AbsCosTheta(wo);
    Float cos_theta_i = AbsCosTheta(wi);
    if (cos_theta_i == 0 || cos_theta_o == 0)
    {
        return false;
    }

    Spectrum f0 = F0(color, metallic);
    Spectrum F = F_Schlick(f0, Dot(wi, wm));

    Spectrum f_r = F * mf.D(wm) * mf.G(wo, wi) / (4 * cos_theta_i * cos_theta_o);
    Spectrum f_d = (1 - metallic) * (Spectrum(1) - F) * (color * inv_pi);

    Float pdf_r = mf.PDF(wo, wm) / (4 * AbsDot(wo, wm));
    Float pdf_d = cos_theta_i * inv_pi;

    *sample = BSDFSample(f_r + f_d, wi, pr * pdf_r + pt * pdf_d, flag);

    return true;
}

void MetallicRoughnessBxDF::Regularize()
{
    mf.Regularize();
}

} // namespace bulbit
