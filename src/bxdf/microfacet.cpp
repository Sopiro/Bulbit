#include "bulbit/microfacet.h"
#include "bulbit/parallel_for.h"

namespace bulbit
{

static const int32 rho_texture_size = 32;

static const int32 rho_samples = 16;
static const Point2 u_rho[rho_samples] = {
    Point2(0.855985f, 0.570367f), Point2(0.381823f, 0.851844f), Point2(0.285328f, 0.764262f), Point2(0.733380f, 0.114073f),
    Point2(0.542663f, 0.344465f), Point2(0.127274f, 0.414848f), Point2(0.964700f, 0.947162f), Point2(0.594089f, 0.643463f),
    Point2(0.095109f, 0.170369f), Point2(0.825444f, 0.263359f), Point2(0.429467f, 0.454469f), Point2(0.244460f, 0.816459f),
    Point2(0.756135f, 0.731258f), Point2(0.516165f, 0.152852f), Point2(0.180888f, 0.214174f), Point2(0.898579f, 0.503897f)
};

void PrecomputeMicrofacetReflectanceTextures()
{
    if (TrowbridgeReitzDistribution::rho_texture || TrowbridgeReitzDistribution::rho_avg_texture ||
        CharlieSheenDistribution::rho_texture)
    {
        return;
    }

    // TrowbridgeReitzDistribution
    {
        Image3f image_e(rho_texture_size, rho_texture_size);
        Image1f image_e_avg(rho_texture_size, 1);

        Float d = 1.0f / rho_texture_size;
        for (int32 j = 0; j < rho_texture_size; ++j)
        {
            Float a = d / 2 + d * j;

            Float r_sum = 0;

            for (int32 i = 0; i < rho_texture_size; ++i)
            {
                Float cos_theta = d / 2 + d * i;
                Float sin_theta = std::sqrt(1 - Sqr(cos_theta));

                Vec3 wo(sin_theta, 0, cos_theta);

                Float r = TrowbridgeReitzDistribution::rho(a, a, wo, u_rho);

                r_sum += r * cos_theta * d;

                image_e(i, j) = { r, r, r };
            }

            image_e_avg(j, 0) = 2 * r_sum;
        }

        WriteImage(image_e, "r_tr.hdr");
        WriteImage(image_e_avg, "r_avg_tr.hdr");

        TrowbridgeReitzDistribution::rho_texture =
            std::make_unique<SpectrumImageTexture>(std::move(image_e), TexCoordFilter::clamp);
        TrowbridgeReitzDistribution::rho_avg_texture =
            std::make_unique<FloatImageTexture>(std::move(image_e_avg), TexCoordFilter::clamp);
    }

    // CharlieSheenDistribution
    {
        Image3f image(rho_texture_size, rho_texture_size);

        Float d = 1.0f / rho_texture_size;
        for (int32 j = 0; j < rho_texture_size; ++j)
        {
            Float a = d / 2 + d * j;

            for (int32 i = 0; i < rho_texture_size; ++i)
            {
                Float cos_theta = d / 2 + d * i;
                Float sin_theta = std::sqrt(1 - Sqr(cos_theta));

                Vec3 wo(sin_theta, 0, cos_theta);

                Float r = CharlieSheenDistribution::rho(a, wo, u_rho);

                image(i, j) = { r, r, r };
            }
        }

        WriteImage(image, "r_cs.hdr");

        CharlieSheenDistribution::rho_texture = std::make_unique<SpectrumImageTexture>(std::move(image), TexCoordFilter::clamp);
    }
}

// Hemispherical reflectance
Float TrowbridgeReitzDistribution::rho(Float alpha_x, Float alpha_y, const Vec3& wo, std::span<const Point2> u)
{
    Float r = 0;
    for (size_t i = 0; i < u.size(); ++i)
    {
        Vec3 wm = Sample_Wm(alpha_x, alpha_y, wo, u[i]);
        Vec3 wi = Reflect(wo, wm);

        if (!SameHemisphere(wo, wi))
        {
            continue;
        }

        Float pdf = PDF(alpha_x, alpha_y, wo, wm) / (4 * AbsDot(wo, wm));

        Float f = D(alpha_x, alpha_y, wm) * G(alpha_x, alpha_y, wo, wi) / (4 * AbsCosTheta(wo) * AbsCosTheta(wi));

        r += f * AbsCosTheta(wi) / pdf;
    }

    return r / u.size();
}

Float TrowbridgeReitzDistribution::D(Float alpha_x, Float alpha_y, const Vec3& wm)
{
    Float tan2_theta = Tan2Theta(wm);
    if (std::isinf(tan2_theta))
    {
        return 0;
    }

    Float cos4_theta = Sqr(Cos2Theta(wm));

    if (cos4_theta < 1e-16f)
    {
        return 0;
    }
    Float e = tan2_theta * (Sqr(CosPhi(wm) / alpha_x) + Sqr(SinPhi(wm) / alpha_y));

    return 1 / (pi * alpha_x * alpha_y * cos4_theta * Sqr(1 + e));
}

Float TrowbridgeReitzDistribution::D(Float alpha_x, Float alpha_y, const Vec3& w, const Vec3& wm)
{
    return G1(alpha_x, alpha_y, w) / AbsCosTheta(w) * D(alpha_x, alpha_y, wm) * AbsDot(w, wm);
}

Float TrowbridgeReitzDistribution::PDF(Float alpha_x, Float alpha_y, const Vec3& w, const Vec3& wm)
{
    return D(alpha_x, alpha_y, w, wm);
}

Float TrowbridgeReitzDistribution::G1(Float alpha_x, Float alpha_y, const Vec3& w)
{
    return 1 / (1 + Lambda(alpha_x, alpha_y, w));
}

Float TrowbridgeReitzDistribution::G(Float alpha_x, Float alpha_y, const Vec3& wo, const Vec3& wi)
{
    return 1 / (1 + Lambda(alpha_x, alpha_y, wo) + Lambda(alpha_x, alpha_y, wi));
}

Float TrowbridgeReitzDistribution::Lambda(Float alpha_x, Float alpha_y, const Vec3& w)
{
    Float tan2_theta = Tan2Theta(w);
    if (std::isinf(tan2_theta))
    {
        return 0;
    }

    Float alpha2 = Sqr(CosPhi(w) * alpha_x) + Sqr(SinPhi(w) * alpha_y);
    return (std::sqrt(1 + alpha2 * tan2_theta) - 1) / 2;
}

Vec3 TrowbridgeReitzDistribution::Sample_Wm(Float alpha_x, Float alpha_y, const Vec3& w, Point2 u)
{
#if 1
    Vec3 wm = Sample_GGX_VNDF_Dupuy_Benyoub(w, alpha_x, alpha_y, u);
#else
    Vec3 wm = Sample_GGX_VNDF_Heitz(w, alpha_x, alpha_y, u);
#endif
    return wm;
}

// Hemispherical reflectance
Float CharlieSheenDistribution::rho(Float alpha, const Vec3& wo, std::span<const Point2> u)
{
    Float r = 0;
    for (size_t i = 0; i < u.size(); ++i)
    {
        Vec3 wi = SampleCosineHemisphere(u[i]);
        Vec3 h = Normalize(wo + wi);

        Float f = D(alpha, h) * G(alpha, wo, wi) / (4 * AbsCosTheta(wo) * AbsCosTheta(wi));
        Float pdf = CosineHemispherePDF(AbsCosTheta(wi));

        r += f * AbsCosTheta(wi) / pdf;
    }

    return r / u.size();
}

Float CharlieSheenDistribution::D(Float alpha, Vec3& wm)
{
    // Eq. (2)
    Float inv_r = 1 / alpha;

    Float cos2_theta = CosTheta(wm);
    Float sin2_theta = 1 - cos2_theta;

    return (2.0f + inv_r) * std::pow(sin2_theta, 0.5f * inv_r) * inv_two_pi;
}

Float CharlieSheenDistribution::G(Float alpha, const Vec3& wo, const Vec3& wi)
{
#if 1
    return 1 / (1 + Lambda(alpha, wo) + Lambda(alpha, wi));
#else
    return 1 / (1 + Lambda2(alpha, wo) + Lambda2(alpha, wi));
#endif
}

Float CharlieSheenDistribution::Lambda(Float alpha, const Vec3& w)
{
    // Eq. 3
    Float cos_theta = CosTheta(w);
    if (cos_theta < 0.5f)
    {
        return std::exp(L(alpha, cos_theta));
    }
    else
    {
        return std::exp(2.0f * L(alpha, 0.5f) - L(alpha, 1.0f - cos_theta));
    }
}

Float CharlieSheenDistribution::Lambda2(Float alpha, const Vec3& w)
{
    // 4 Terminator Softening
    return std::pow(Lambda(alpha, w), 1.0f + 2.0f * std::pow(1.0f - CosTheta(w), 8.0f));
}

Float CharlieSheenDistribution::L(Float alpha, Float x)
{
    Float t = Sqr(1.0f - alpha);

    Float a = Lerp(21.5473f, 25.3245f, t);
    Float b = Lerp(3.82987f, 3.32435f, t);
    Float c = Lerp(0.19823f, 0.16801f, t);
    Float d = Lerp(-1.97760f, -1.27393f, t);
    Float e = Lerp(-4.32054f, -4.85967f, t);

    // 3 Shadowing Term
    return a / (1.0f + b * std::pow(x, c)) + d * x + e;
}

} // namespace bulbit
