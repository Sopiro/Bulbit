#include "bulbit/bxdfs.h"

namespace bulbit
{

Spectrum NormalizedFresnelBxDF::f(const Vec3& wo, const Vec3& wi, TransportDirection direction) const
{
    if (!SameHemisphere(wo, wi))
    {
        return Spectrum::black;
    }

    // Normalization constant
    Float c = 1 / (pi * (1 - 2 * FresnelMoment1(1 / eta)));
    Spectrum f(c * (1 - FresnelDielectric(CosTheta(wi), eta)));

    // Handle solid angle squeezing for BSSRDF transmission
    if (direction == TransportDirection::ToLight)
    {
        f *= Sqr(eta);
    }

    return f;
}

Float NormalizedFresnelBxDF::PDF(Vec3 wo, Vec3 wi, TransportDirection direction, BxDF_SamplingFlags flags) const
{
    BulbitNotUsed(direction);

    if (!(flags & BxDF_SamplingFlags::Reflection))
    {
        return 0;
    }

    if (SameHemisphere(wo, wi))
    {
        return CosineHemispherePDF(AbsCosTheta(wi));
    }
    else
    {
        return 0;
    }
}

bool NormalizedFresnelBxDF::Sample_f(
    BSDFSample* sample, Vec3 wo, Float u0, Point2 u12, TransportDirection direction, BxDF_SamplingFlags flags
) const
{
    BulbitNotUsed(u0);

    // Same as the Lambertian BRDF
    if (!(flags & BxDF_SamplingFlags::Reflection))
    {
        return false;
    }

    Vec3 wi = SampleCosineHemisphere(u12);
    Float pdf = CosineHemispherePDF(AbsCosTheta(wi));
    if (wo.z < 0)
    {
        wi.z = -wi.z;
    }

    *sample = BSDFSample(f(wo, wi, direction), wi, pdf, BxDF_Flags::DiffuseReflection);
    return true;
}

void NormalizedFresnelBxDF::Regularize() {}

} // namespace bulbit
