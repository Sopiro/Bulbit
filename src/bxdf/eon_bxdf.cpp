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

// Listing 2.
void ltc_coeffs(Float mu, Float r, Float* a, Float* b, Float* c, Float* d)
{
    *a = 1.0f + r * (0.303392f + (-0.518982f + 0.111709f * mu) * mu + (-0.276266f + 0.335918f * mu) * r);
    *b = r * (-1.16407f + 1.15859f * mu + (0.150815f - 0.150105f * mu) * r) / (mu * mu * mu - 1.43545f);
    *c = 1.0f + (0.20013f + (-0.506373f + 0.261777f * mu) * mu) * r;
    *d = ((0.540852f + (-1.01625f + 0.475392f * mu) * mu) * r) / (-1.0743f + mu * (0.0725628f + mu));
}

// Listing 3.
Mat3 orthonormal_basis_ltc(Vec3 w)
{

    Float lenSqr = Length2(Vec2(w.x, w.y));
    Vec3 X = lenSqr > 0.0f ? Vec3(w.x, w.y, 0.0f) * 1 / std::sqrt(lenSqr) : Vec3(1, 0, 0);
    Vec3 Y = Vec3(-X.y, X.x, 0.0f); // cross(Z, X)

    return Mat3(X, Y, Vec3(0, 0, 1));
}

Vec3 cltc_sample(Vec3 wo, Float r, Float u1, Float u2, Float* pdf)
{
    Float a, b, c, d;
    ltc_coeffs(wo.z, r, &a, &b, &c, &d);

    Float R = sqrt(u1);
    Float phi = two_pi * u2;

    Float x = R * cos(phi);
    Float y = R * sin(phi);

    Float vz = 1.0f / sqrt(d * d + 1.0f);
    Float s = 0.5f * (1.0f + vz);
    x = -Lerp(sqrt(1.0f - y * y), x, s);

    Vec3 wh = Vec3(x, y, sqrt(std::max(1.0f - (x * x + y * y), 0.0f))); // ωH sample via CLTC
    Float pdf_wh = wh.z / (pi * s);

    Vec3 wi = Vec3(a * wh.x + b * wh.z, c * wh.y, d * wh.x + wh.z);
    Float len = Length(wi);
    Float detM = c * (a - b * d);
    Float pdf_wi = pdf_wh * len * len * len / detM;
    Mat3 fromLTC = orthonormal_basis_ltc(wo);

    wi = Normalize(Mul(fromLTC, wi));

    *pdf = pdf_wi;

    return wi;
}

Float cltc_pdf(Vec3 wo_local, Vec3 wi_local, Float r)
{
    Mat3 toLTC = orthonormal_basis_ltc(wo_local).GetTranspose();
    Vec3 wi = Mul(toLTC, wi_local);

    Float a, b, c, d;
    ltc_coeffs(wo_local.z, r, &a, &b, &c, &d);

    Float detM = c * (a - b * d);
    Vec3 wh = Vec3(c * (wi.x - b * wi.z), (a - b * d) * wi.y, -c * (d * wi.x - a * wi.z));
    Float lenSqr = Dot(wh, wh);
    Float vz = 1.0f / sqrt(d * d + 1.0f);
    Float s = 0.5f * (1.0f + vz);
    Float pdf = detM * detM / (lenSqr * lenSqr) * std::max(wh.z, 0.0f) / (pi * s);

    return pdf;
}

// Evaluates EON BRDF value, given inputs:
//  rho = single-scattering albedo
//  r = roughness in [0,1]
//  exact = flag to select exact or fast approx. version
Spectrum EONBxDF::f(Vec3 wo, Vec3 wi, TransportDirection direction) const
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

    Float mu = wo.z;
    Float P_u = pow(r, 0.1f) * (0.162925f + mu * (-0.372058f + (0.538233f - 0.290822f * mu) * mu));
    Float P_c = 1.0f - P_u;

    Float pdf_c = cltc_pdf(wo, wi, r);
    Float pdf_u = UniformHemispherePDF();

    return P_u * pdf_u + P_c * pdf_c;
}

bool EONBxDF::Sample_f(
    BSDFSample* sample, Vec3 wo, Float u0, Point2 u12, TransportDirection direction, BxDF_SamplingFlags flags
) const
{
    BulbitNotUsed(direction);

    if (!(flags & BxDF_SamplingFlags::Reflection))
    {
        return false;
    }

    Float mu = wo.z;
    Float P_u = pow(r, 0.1f) * (0.162925f + mu * (-0.372058f + (0.538233f - 0.290822f * mu) * mu));
    Float P_c = 1.0f - P_u; // probability of CLTC sample

    Vec3 wi;
    Float pdf_c;
    if (u0 <= P_u)
    {
        wi = SampleUniformHemisphere(u12); // sample wi from uniform lobe
        pdf_c = cltc_pdf(wo, wi, r);
    }
    else
    {
        wi = cltc_sample(wo, r, u12[0], u12[1], &pdf_c); // sample wi from CLTC lobe
    }

    const Float pdf_u = UniformHemispherePDF();
    Float pdf = P_u * pdf_u + P_c * pdf_c; // MIS PDF of wi

    *sample = BSDFSample(f(wo, wi, direction), wi, pdf, BxDF_Flags::DiffuseReflection);

    return true;
}

} // namespace bulbit
