#include "bulbit/microfacet.h"
#include "bulbit/async_job.h"
#include "bulbit/bxdfs.h"
#include "bulbit/image3d.h"
#include "bulbit/parallel_for.h"
#include "bulbit/samplers.h"
#include "bulbit/texture3d.h"

#define WRITE_REFLECTANCE_TEXTURE 0

namespace bulbit
{

void TrowbridgeReitzDistribution::ComputeReflectanceTexture(int32 texture_size, std::span<Float> uc, std::span<Point2> u)
{
    BulbitNotUsed(uc);

    if (E_texture && E_avg_texture)
    {
        return;
    }

    Image1f image_e(texture_size, texture_size);
    Image1f image_e_avg(texture_size, 1);

    const Float d = 1.0f / texture_size;

    ParallelFor(0, texture_size, [&](int32 j) {
        Float a = d / 2 + d * j;

        Float r_sum = 0;

        for (int32 i = 0; i < texture_size; ++i)
        {
            Float cos_theta = d / 2 + d * i;
            Float sin_theta = std::sqrt(1 - Sqr(cos_theta));

            Vec3 wo(sin_theta, 0, cos_theta);

            TrowbridgeReitzDistribution distribution(a, a);
            Float r = distribution.rho(wo, u);

            r_sum += r * cos_theta * d;

            image_e(i, j) = r;
        }

        image_e_avg(j, 0) = 2 * r_sum;
    });

#if WRITE_REFLECTANCE_TEXTURE
    WriteImage(image_e, "E_tr.hdr");
    WriteImage(image_e_avg, "E_avg_tr.hdr");
#endif

    TrowbridgeReitzDistribution::E_texture = std::make_unique<FloatImageTexture>(std::move(image_e), TexCoordFilter::clamp);
    TrowbridgeReitzDistribution::E_avg_texture =
        std::make_unique<FloatImageTexture>(std::move(image_e_avg), TexCoordFilter::clamp);
}

void CharlieSheenDistribution::ComputeReflectanceTexture(int32 texture_size, std::span<Float> uc, std::span<Point2> u)
{
    BulbitNotUsed(uc);

    if (CharlieSheenDistribution::E_texture)
    {
        return;
    }

    Image1f image(texture_size, texture_size);

    const Float d = 1.0f / texture_size;
    ParallelFor(0, texture_size, [&](int32 j) {
        Float a = d / 2 + d * j;

        for (int32 i = 0; i < texture_size; ++i)
        {
            Float cos_theta = d / 2 + d * i;
            Float sin_theta = std::sqrt(1 - Sqr(cos_theta));

            Vec3 wo(sin_theta, 0, cos_theta);

            CharlieSheenDistribution distribution(a);
            Float r = distribution.rho(wo, u);

            image(i, j) = r;
        }
    });

#if WRITE_REFLECTANCE_TEXTURE
    WriteImage(image, "E_cs.hdr");
#endif

    CharlieSheenDistribution::E_texture = std::make_unique<FloatImageTexture>(std::move(image), TexCoordFilter::clamp);
}

void DielectricBxDF::ComputeReflectanceTexture(int32 texture_size, std::span<Float> uc, std::span<Point2> u)
{
    if (E_texture && E_inv_texture)
    {
        return;
    }

    Image3D1f image_e(texture_size, texture_size, texture_size);
    Image3D1f image_e_inv(texture_size, texture_size, texture_size);
    Image1f image_e_avg(texture_size, texture_size);
    Image1f image_e_inv_avg(texture_size, texture_size);

    const Float d = 1.0f / texture_size;

    ParallelFor(0, texture_size, [&](int32 k) {
        Float f = d / 2 + d * k;
        Float ior = MapF0toIOR(f);
        // std::cout << k << ": " << f << ", " << ior << std::endl;

        for (int32 j = 0; j < texture_size; ++j)
        {
            Float a = d / 2 + d * j;

            Float r_sum = 0;
            Float r_inv_sum = 0;

            for (int32 i = 0; i < texture_size; ++i)
            {
                Float cos_theta = d / 2 + d * i;
                Float sin_theta = std::sqrt(1 - Sqr(cos_theta));

                Vec3 wo(sin_theta, 0, cos_theta);

                DielectricBxDF bsdf_i(ior, Spectrum(1), TrowbridgeReitzDistribution(a, a), false);
                DielectricBxDF bsdf_t(1 / ior, Spectrum(1), TrowbridgeReitzDistribution(a, a), false);

                Float r = bsdf_i.rho(wo, uc, u, TransportDirection::ToLight).Average();
                Float r_inv = bsdf_t.rho(wo, uc, u, TransportDirection::ToLight).Average();

                r_sum += r * cos_theta * d;
                r_inv_sum += r_inv * cos_theta * d;

                image_e(k, i, j) = r;
                image_e_inv(k, i, j) = r_inv;
            }

            image_e_avg(k, j) = 2 * r_sum;
            image_e_inv_avg(k, j) = 2 * r_inv_sum;
        }
    });

#if WRITE_REFLECTANCE_TEXTURE
    WriteImage3D(image_e, "E_d.hdr");
    WriteImage3D(image_e_inv, "E_inv_d.hdr");
    WriteImage(image_e_avg, "E_avg_d.hdr");
    WriteImage(image_e_inv_avg, "E_inv_avg_d.hdr");
#endif

    DielectricBxDF::E_texture = std::make_unique<FloatImageTexture3D>(std::move(image_e), TexCoordFilter::clamp);
    DielectricBxDF::E_inv_texture = std::make_unique<FloatImageTexture3D>(std::move(image_e_inv), TexCoordFilter::clamp);
    DielectricBxDF::E_avg_texture = std::make_unique<FloatImageTexture>(std::move(image_e_avg), TexCoordFilter::clamp);
    DielectricBxDF::E_inv_avg_texture = std::make_unique<FloatImageTexture>(std::move(image_e_inv_avg), TexCoordFilter::clamp);
}

void ComoputeReflectanceTextures()
{
    int32 texture_size = 32;

    int32 x_samples = 16;
    int32 y_samples = x_samples;
    int32 samples = x_samples * y_samples;
    std::vector<Float> uc(samples);
    std::vector<Point2> u(samples);

    const uint32 hash_uc = 1;
    const uint32 hash_u = 2;
    RNG rng(3);

    for (int32 i = 0; i < samples; ++i)
    {
        {
            int32 stratum = PermutationElement(i, samples, hash_uc);
            uc[i] = (stratum + rng.NextFloat()) / samples;
        }

        {
            int32 stratum = PermutationElement(i, samples, hash_u);

            int32 x = stratum % x_samples;
            int32 y = stratum / x_samples;

            u[i] = { (x + rng.NextFloat()) / x_samples, (y + rng.NextFloat()) / y_samples };
        }
    }

    TrowbridgeReitzDistribution::ComputeReflectanceTexture(texture_size, uc, u);
    CharlieSheenDistribution::ComputeReflectanceTexture(texture_size, uc, u);
    DielectricBxDF::ComputeReflectanceTexture(texture_size, uc, u);
}

} // namespace bulbit
