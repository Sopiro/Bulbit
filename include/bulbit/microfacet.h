#pragma once

#include "common.h"
#include "frame.h"
#include "image.h"
#include "sampling.h"
#include "spectrum.h"
#include "textures.h"

namespace bulbit
{

class TrowbridgeReitzDistribution
{
public:
    TrowbridgeReitzDistribution(Float alpha_x, Float alpha_y)
        : alpha_x{ alpha_x }
        , alpha_y{ alpha_y }
    {
        if (!EffectivelySmooth())
        {
            alpha_x = std::max(alpha_x, Float(1e-4));
            alpha_y = std::max(alpha_y, Float(1e-4));
        }
    }

    bool EffectivelySmooth() const
    {
        return std::max(alpha_x, alpha_y) < 1e-3f;
    }

    Float D(const Vec3& wm) const
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

    Float D(const Vec3& w, const Vec3& wm) const
    {
        return G1(w) / AbsCosTheta(w) * D(wm) * AbsDot(w, wm);
    }

    Float PDF(const Vec3& w, const Vec3& wm) const
    {
        return D(w, wm);
    }

    Float G1(const Vec3& w) const
    {
        return 1 / (1 + Lambda(w));
    }

    Float G(const Vec3& wo, const Vec3& wi) const
    {
        return 1 / (1 + Lambda(wo) + Lambda(wi));
    }

    Float Lambda(const Vec3& w) const
    {
        Float tan2_theta = Tan2Theta(w);
        if (std::isinf(tan2_theta))
        {
            return 0;
        }

        Float alpha2 = Sqr(CosPhi(w) * alpha_x) + Sqr(SinPhi(w) * alpha_y);
        return (std::sqrt(1 + alpha2 * tan2_theta) - 1) / 2;
    }

    Vec3 Sample_Wm(const Vec3& w, Point2 u) const
    {
#if 1
        Vec3 wm = Sample_GGX_VNDF_Dupuy_Benyoub(w, alpha_x, alpha_y, u);
#else
        Vec3 wm = Sample_GGX_VNDF_Heitz(w, alpha_x, alpha_y, u);
#endif
        return wm;
    }

    static Float RoughnessToAlpha(Float roughness)
    {
        return Sqr(roughness);
    }

    void Regularize()
    {
        if (alpha_x < 0.3f) alpha_x = Clamp(2 * alpha_x, 0.1f, 0.3f);
        if (alpha_y < 0.3f) alpha_y = Clamp(2 * alpha_y, 0.1f, 0.3f);
    }

private:
    Float alpha_x, alpha_y;
};

// https://blog.selfshadow.com/publications/s2017-shading-course/imageworks/s2017_pbs_imageworks_sheen.pdf
class CharlieSheenDistribution
{
public:
    CharlieSheenDistribution(Float alpha)
        : alpha{ std::max(alpha, 1e-4f) }
    {
        // Precompute hemispherical-directional reflectance and store in a texture
        std::call_once(rho_init_flag, [&] {
            Image3f image(rho_texture_size, rho_texture_size);

            const int32 rho_samples = 16;
            const Point2 u_rho[rho_samples] = { Point2(0.855985f, 0.570367f), Point2(0.381823f, 0.851844f),
                                                Point2(0.285328f, 0.764262f), Point2(0.733380f, 0.114073f),
                                                Point2(0.542663f, 0.344465f), Point2(0.127274f, 0.414848f),
                                                Point2(0.964700f, 0.947162f), Point2(0.594089f, 0.643463f),
                                                Point2(0.095109f, 0.170369f), Point2(0.825444f, 0.263359f),
                                                Point2(0.429467f, 0.454469f), Point2(0.244460f, 0.816459f),
                                                Point2(0.756135f, 0.731258f), Point2(0.516165f, 0.152852f),
                                                Point2(0.180888f, 0.214174f), Point2(0.898579f, 0.503897f) };

            Float d = rho_texture_size - 1;
            for (int32 j = 0; j < rho_texture_size; ++j)
            {
                Float a = std::max(j / d, 1e-4f);
                for (int32 i = 0; i < rho_texture_size; ++i)
                {
                    Float cos_theta = std::max(i / d, 1e-4f);
                    Float sin_theta = std::sqrt(1 - Sqr(cos_theta));

                    Vec3 wo(sin_theta, 0, cos_theta);

                    Float r = rho(a, wo, u_rho);

                    image(i, j) = { r, r, r };
                }
            }

            rho_texture = std::make_unique<SpectrumImageTexture>(std::move(image), TexCoordFilter::clamp);
        });
    }

    Float D(Vec3& wm) const
    {
        // Eq. (2)
        Float inv_r = 1 / alpha;

        Float cos2_theta = CosTheta(wm);
        Float sin2_theta = 1 - cos2_theta;

        return (2.0f + inv_r) * std::pow(sin2_theta, 0.5f * inv_r) * inv_two_pi;
    }

    Float G(const Vec3& wo, const Vec3& wi) const
    {
#if 1
        return 1 / (1 + Lambda(wo) + Lambda(wi));
#else
        return 1 / (1 + Lambda2(wo) + Lambda2(wi));
#endif
    }

    Float Lambda(const Vec3& w) const
    {
        // Eq. 3
        Float cos_theta = CosTheta(w);
        if (cos_theta < 0.5f)
        {
            return std::exp(L(cos_theta));
        }
        else
        {
            return std::exp(2.0f * L(0.5f) - L(1.0f - cos_theta));
        }
    }

    Float Lambda2(const Vec3& w) const
    {
        // 4 Terminator Softening
        return std::pow(Lambda(w), 1 + 2 * std::pow(1 - CosTheta(w), 8));
    }

    Float L(Float x) const
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

    // Hemispherical reflectance
    Float rho(const Vec3& wo) const
    {
        return rho_texture->Evaluate({ wo.z * rho_texture_size, alpha * rho_texture_size })[0];
    }

private:
    static inline std::once_flag rho_init_flag;
    static inline const int32 rho_texture_size = 64;
    static inline std::unique_ptr<SpectrumImageTexture> rho_texture;

    // Hemispherical reflectance
    static Float rho(Float alpha, const Vec3& wo, std::span<const Point2> u)
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

    static Float D(Float alpha, Vec3& wm)
    {
        // Eq. (2)
        Float inv_r = 1 / alpha;

        Float cos2_theta = CosTheta(wm);
        Float sin2_theta = 1 - cos2_theta;

        return (2.0f + inv_r) * std::pow(sin2_theta, 0.5f * inv_r) * inv_two_pi;
    }

    static Float G(Float alpha, const Vec3& wo, const Vec3& wi)
    {
#if 1
        return 1 / (1 + Lambda(alpha, wo) + Lambda(alpha, wi));
#else
        return 1 / (1 + Lambda2(alpha, wo) + Lambda2(alpha, wi));
#endif
    }

    static Float Lambda(Float alpha, const Vec3& w)
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

    static Float Lambda2(Float alpha, const Vec3& w)
    {
        // 4 Terminator Softening
        return std::pow(Lambda(alpha, w), 1 + 2 * std::pow(1 - CosTheta(w), 8));
    }

    static Float L(Float alpha, Float x)
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

    Float alpha;
};

} // namespace bulbit
