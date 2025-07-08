#include "bulbit/bxdfs.h"

namespace bulbit
{

Spectrum SheenBxDF::f(const Vec3& wo, const Vec3& wi, TransportDirection direction) const
{
    BulbitNotUsed(direction);

    if (!SameHemisphere(wo, wi))
    {
        return Spectrum::black;
    }

    Vec3 wm = Normalize(wi + wo);

    Float cos_theta_o = CosTheta(wo);
    Float cos_theta_i = CosTheta(wi);

    Float denom = std::abs(4 * cos_theta_i * cos_theta_o);

    // Combine with lambertian BRDF by albedo normalization technique
    Spectrum f_diffuse = (1 - mf.E(wo)) * base * inv_pi;
    Spectrum f_sheen = sheen * mf.D(wm) * mf.G(wo, wi) / denom;

    return f_diffuse + f_sheen;
}

Float SheenBxDF::PDF(Vec3 wo, Vec3 wi, TransportDirection direction, BxDF_SamplingFlags flags) const
{
    BulbitNotUsed(direction);
    BulbitNotUsed(flags);
    if (!SameHemisphere(wo, wi))
    {
        return 0;
    }

    return CosineHemispherePDF(wi.z);
}

bool SheenBxDF::Sample_f(
    BSDFSample* sample, Vec3 wo, Float u0, Point2 u12, TransportDirection direction, BxDF_SamplingFlags flags
) const
{
    BulbitNotUsed(u0);
    BulbitNotUsed(direction);

    if (!(flags & BxDF_SamplingFlags::Reflection))
    {
        return false;
    }

    Vec3 wi = SampleCosineHemisphere(u12);
    Float pdf = CosineHemispherePDF(wi.z);

    Vec3 wm = Normalize(wi + wo);

    Float cos_theta_o = CosTheta(wo);
    Float cos_theta_i = CosTheta(wi);

    Float denom = std::abs(4 * cos_theta_i * cos_theta_o);

    Spectrum f_diffuse = (1 - mf.E(wo)) * base * inv_pi;
    Spectrum f_sheen = sheen * mf.D(wm) * mf.G(wo, wi) / denom;

    *sample = BSDFSample(f_diffuse + f_sheen, wi, pdf, BxDF_Flags::DiffuseReflection);
    return true;
}

} // namespace bulbit
