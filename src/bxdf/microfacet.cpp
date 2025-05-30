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

} // namespace bulbit
