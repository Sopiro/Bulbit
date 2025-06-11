#include "bulbit/microfacet.h"
#include "bulbit/async_job.h"
#include "bulbit/bxdfs.h"
#include "bulbit/image3d.h"
#include "bulbit/samplers.h"
#include "bulbit/texture3d.h"

#define WRITE_REFLECTANCE_TEXTURE 0

namespace bulbit
{

void TrowbridgeReitzDistribution::ComputeReflectanceTexture(int32 texture_size, std::span<Float> uc, std::span<Point2> u)
{
    BulbitNotUsed(uc);

    if (rho_texture && rho_avg_texture)
    {
        return;
    }

    Image1f image_e(texture_size, texture_size);
    Image1f image_e_avg(texture_size, 1);

    const Float d = 1.0f / texture_size;
    for (int32 j = 0; j < texture_size; ++j)
    {
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
    }

#if WRITE_REFLECTANCE_TEXTURE
    WriteImage(image_e, "r_tr.hdr");
    WriteImage(image_e_avg, "r_avg_tr.hdr");
#endif

    TrowbridgeReitzDistribution::rho_texture = std::make_unique<FloatImageTexture>(std::move(image_e), TexCoordFilter::clamp);
    TrowbridgeReitzDistribution::rho_avg_texture =
        std::make_unique<FloatImageTexture>(std::move(image_e_avg), TexCoordFilter::clamp);
}

void CharlieSheenDistribution::ComputeReflectanceTexture(int32 texture_size, std::span<Float> uc, std::span<Point2> u)
{
    BulbitNotUsed(uc);

    if (CharlieSheenDistribution::rho_texture)
    {
        return;
    }

    Image1f image(texture_size, texture_size);

    const Float d = 1.0f / texture_size;
    for (int32 j = 0; j < texture_size; ++j)
    {
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
    }

#if WRITE_REFLECTANCE_TEXTURE
    WriteImage(image, "r_cs.hdr");
#endif

    CharlieSheenDistribution::rho_texture = std::make_unique<FloatImageTexture>(std::move(image), TexCoordFilter::clamp);
}

void DielectricBxDF::ComputeReflectanceTexture(int32 texture_size, std::span<Float> uc, std::span<Point2> u)
{
    if (rho_texture && rho_inv_texture)
    {
        return;
    }

    Image3D1f image_e(texture_size, texture_size, texture_size);
    Image3D1f image_e_inv(texture_size, texture_size, texture_size);
    // Image1f image_e_avg(texture_size, 1);

    const Float d = 1.0f / texture_size;

    for (int32 k = 0; k < texture_size; ++k)
    {
        Float f = d / 2 + d * k;
        Float ior = MapF0toIOR(f);
        // std::cout << k << ": " << f << ", " << ior << std::endl;

        for (int32 j = 0; j < texture_size; ++j)
        {
            Float a = d / 2 + d * j;

            // Float r_sum = 0;

            for (int32 i = 0; i < texture_size; ++i)
            {
                Float cos_theta = d / 2 + d * i;
                Float sin_theta = std::sqrt(1 - Sqr(cos_theta));

                Vec3 wo(sin_theta, 0, cos_theta);

                DielectricBxDF bsdf_i(ior, Spectrum(1), TrowbridgeReitzDistribution(a, a), false);
                DielectricBxDF bsdf_t(1 / ior, Spectrum(1), TrowbridgeReitzDistribution(a, a), false);

                Spectrum r = bsdf_i.rho(wo, uc, u);
                Spectrum r_inv = bsdf_t.rho(wo, uc, u);

                // r_sum += r * cos_theta * d;

                image_e(k, i, j) = r.Average();
                image_e_inv(k, i, j) = r_inv.Average();
            }

            // image_e_avg(j, 0) = 2 * r_sum;
        }
    }

#if WRITE_REFLECTANCE_TEXTURE
    WriteImage3D(image_e, "r_d.hdr");
    WriteImage3D(image_e_inv, "r_d_inv.hdr");
#endif

    DielectricBxDF::rho_texture = std::make_unique<FloatImageTexture3D>(std::move(image_e), TexCoordFilter::clamp);
    DielectricBxDF::rho_inv_texture = std::make_unique<FloatImageTexture3D>(std::move(image_e_inv), TexCoordFilter::clamp);
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

    std::unique_ptr<AsyncJob<bool>> tr = RunAsync([&]() {
        TrowbridgeReitzDistribution::ComputeReflectanceTexture(texture_size, uc, u);
        return true;
    });

    std::unique_ptr<AsyncJob<bool>> cs = RunAsync([&]() {
        CharlieSheenDistribution::ComputeReflectanceTexture(texture_size, uc, u);
        return true;
    });

    std::unique_ptr<AsyncJob<bool>> d = RunAsync([&]() {
        DielectricBxDF::ComputeReflectanceTexture(texture_size, uc, u);
        return true;
    });

    tr->Wait();
    cs->Wait();
    d->Wait();
}

} // namespace bulbit
