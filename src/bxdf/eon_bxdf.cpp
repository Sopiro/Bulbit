#include "bulbit/bxdfs.h"
#include "bulbit/frame.h"
#include "bulbit/sampling.h"

namespace bulbit
{

// Listing 1.
// https://jcgt.org/published/0014/01/06/

static const Float constant1_FON = 0.5f - 2.0f / (3.0f * pi);
static const Float constant2_FON = 2.0f / 3.0f - 28.0f / (15.0f * pi);

static Float E_FON_exact(Float mu, Float r)
{
    Float AF = 1.0f / (1.0f + constant1_FON * r); // FON A coefficient
    Float BF = r * AF;                            // FON B coefficient
    Float Si = std::sqrt(1.0f - (mu * mu));
    Float G = Si * (std::acos(mu) - Si * mu) + (2.0f / 3.0f) * ((Si / mu) * (1.0f - (Si * Si * Si)) - Si);
    return AF + (BF * inv_pi) * G;
}

static Float E_FON_approx(Float mu, Float r)
{
    Float mucomp = 1.0f - mu;

    const Float g1 = 0.0571085289f;
    const Float g2 = 0.491881867f;
    const Float g3 = -0.332181442f;
    const Float g4 = 0.0714429953f;

    Float GoverPi = mucomp * (g1 + mucomp * (g2 + mucomp * (g3 + mucomp * g4)));

    return (1.0f + r * GoverPi) / (1.0f + constant1_FON * r);
}

// Evaluates EON BRDF value, given inputs:
//  rho = single-scattering albedo
//  r = roughness in [0,1]
//  exact = flag to select exact or fast approx. version
Spectrum EONBxDF::f(const Vec3& wo, const Vec3& wi, TransportDirection direction) const
{
    BulbitNotUsed(direction);

    if (!SameHemisphere(wo, wi))
    {
        return Spectrum::black;
    }

    Float mu_i = wi.z;
    Float mu_o = wo.z;

    Float s = Dot(wi, wo) - mu_i * mu_o;
    Float sovertF = s > 0 ? s / std::max(mu_i, mu_o) : s;
    Float AF = 1.0f / (1.0f + constant1_FON * r);

    Spectrum f_ss = (rho * inv_pi) * AF * (1.0f + r * sovertF);

    Float EFo = exact ? E_FON_exact(mu_o, r) : E_FON_approx(mu_o, r);
    Float EFi = exact ? E_FON_exact(mu_i, r) : E_FON_approx(mu_i, r);

    Float avgEF = AF * (1.0f + constant2_FON * r);

    Spectrum rho_ms = (rho * rho) * avgEF / (Spectrum(1.0f) - rho * (1.0f - avgEF));
    constexpr Float eps = 1.0e-7f;

    Spectrum f_ms = (rho_ms * inv_pi) * std::max(eps, 1.0f - EFo) * std::max(eps, 1.0f - EFi) / std::max(eps, 1.0f - avgEF);

    return f_ss + f_ms;
}

Float EONBxDF::PDF(Vec3 wo, Vec3 wi, TransportDirection direction, BxDF_SamplingFlags flags) const
{
    BulbitNotUsed(direction);

    if (!(flags & BxDF_SamplingFlags::Reflection) || !SameHemisphere(wo, wi))
    {
        return 0;
    }

    return CosineHemispherePDF(AbsCosTheta(wi));
}

bool EONBxDF::Sample_f(
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
    Float pdf = CosineHemispherePDF(CosTheta(wi));
    if (wo.z < 0)
    {
        wi.z = -wi.z;
    }

    *sample = BSDFSample(f(wo, wi, direction), wi, pdf, BxDF_Flags::DiffuseReflection);

    return true;
}

} // namespace bulbit
