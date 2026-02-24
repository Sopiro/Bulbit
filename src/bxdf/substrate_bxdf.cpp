#include "bulbit/bxdfs.h"
#include "bulbit/frame.h"
#include "bulbit/sampling.h"
#include "bulbit/scattering.h"

namespace bulbit
{

Float SubstrateBxDF::GlossyProbability(Vec3 wo) const
{
    // Sampling is done with a two-lobe mixture.
    // This returns p(glossy | wo); the diffuse lobe uses 1 - p.
    Float glossy = FresnelDielectric(AbsCosTheta(wo), eta);
    Float substrate = reflectance.Average() * avg_transmittance * (1 - glossy);

    Float denominator = glossy + substrate;
    if (denominator <= 0)
    {
        return 1;
    }

    return Clamp(glossy / denominator, 0, 1);
}

Spectrum SubstrateBxDF::f(Vec3 wo, Vec3 wi, TransportDirection direction) const
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
    if (Dot(wm, z_axis) < 0)
    {
        wm.Negate();
    }

    // Fresnel on both directions for the diffuse substrate term:
    // incoming light must enter the coating (1 - F_i),
    // outgoing light must also exit the coating (1 - F_o).
    Float F_i = FresnelDielectric(cos_theta_i, eta);
    Float F_o = FresnelDielectric(cos_theta_o, eta);

    // Diffuse substrate BRDF
    //
    // Model used here (position-free approximation):
    //   f_d ~= (1 - F_i) * (1 - F_o) * eta_t^2 * (rho_d / (1 - rho_d * Fdr)) * (1 / pi)
    //
    // where:
    //   eta_t = 1 / eta
    //   rho_d = reflectance
    //   F_dr   = fresnel_avg (hemispherical-average Fresnel reflectance)
    //
    // The factor rho_d / (1 - rho_d * F_dr) comes from a geometric-series
    // approximation of internal diffuse re-bounces under the coating:
    //   rho_d * [1 + rho_d * F_dr + (rho_d * F_dr)^2 + ...]
    //
    // This keeps the model local (BRDF-only), while approximating multi-bounce
    // energy that would otherwise require explicit sub-surface path simulation.
    // Integrators multiply by AbsDot(n, wi) outside.
    Spectrum denom = Max(Spectrum(1) - reflectance * fresnel_avg, 1e-4f);
    Spectrum diffuse = ((1 - F_i) * (1 - F_o) * Sqr(1 / eta) * inv_pi) * (reflectance / denom);

    if (sigma_a_dt.MaxComponent() > 0)
    {
        // Beer-Lambert attenuation for the path inside the substrate:
        // exp(-sigma_a * (1/cos_i + 1/cos_o) * thickness)
        const Float dz_i = 1 / cos_theta_i;
        const Float dz_o = 1 / cos_theta_o;
        diffuse *= Exp(-sigma_a_dt * (dz_i + dz_o));
    }

    // Glossy top-coat reflection (GGX microfacet dielectric BRDF).
    Float F = FresnelDielectric(Dot(wo, wm), eta);
    Spectrum glossy(F * mf.D(wm) * mf.G(wo, wi) / (4 * cos_theta_i * cos_theta_o));

    return glossy + diffuse;
}

Float SubstrateBxDF::PDF(Vec3 wo, Vec3 wi, TransportDirection direction, BxDF_SamplingFlags flags) const
{
    BulbitNotUsed(direction);

    if (!(flags & BxDF_SamplingFlags::Reflection))
    {
        return 0;
    }

    if (!SameHemisphere(wo, wi) || wo.z == 0 || wi.z == 0)
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

    Float p_glossy = GlossyProbability(wo);
    Float p_diffuse = 1 - p_glossy;

    Float pdf_glossy = 0;
    if (Float abs_dot_wo_wm = AbsDot(wo, wm); abs_dot_wo_wm > 0)
    {
        pdf_glossy = mf.PDF(wo, wm) / (4 * abs_dot_wo_wm);
    }
    Float pdf_diffuse = CosineHemispherePDF(AbsCosTheta(wi));

    return p_glossy * pdf_glossy + p_diffuse * pdf_diffuse;
}

bool SubstrateBxDF::Sample_f(
    BSDFSample* sample, Vec3 wo, Float u0, Point2 u12, TransportDirection direction, BxDF_SamplingFlags flags
) const
{
    if (!(flags & BxDF_SamplingFlags::Reflection))
    {
        return false;
    }

    if (wo.z == 0)
    {
        return false;
    }

    Float p_specular = GlossyProbability(wo);

    Vec3 wi;
    BxDF_Flags sample_flags;
    if (u0 < p_specular)
    {
        // Sample glossy top interface
        Vec3 wm = mf.Sample_Wm(wo, u12);
        wi = Reflect(wo, wm);
        if (!SameHemisphere(wo, wi))
        {
            return false;
        }

        sample_flags = BxDF_Flags::GlossyReflection;
    }
    else
    {
        // Sample diffuse substrate
        wi = SampleCosineHemisphere(u12);
        if (wo.z < 0)
        {
            wi.z = -wi.z;
        }

        sample_flags = BxDF_Flags::DiffuseReflection;
    }

    // Evaluate the full BSDF and divide by the matched mixture PDF.
    Float pdf = PDF(wo, wi, direction, flags);
    if (pdf == 0)
    {
        return false;
    }

    Spectrum r = f(wo, wi, direction);
    if (r.IsBlack())
    {
        return false;
    }

    *sample = BSDFSample(r, wi, pdf, sample_flags);
    return true;
}

void SubstrateBxDF::Regularize()
{
    mf.Regularize();
}

} // namespace bulbit
