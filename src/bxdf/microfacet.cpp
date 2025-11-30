#include "bulbit/microfacet.h"
#include "bulbit/async_job.h"
#include "bulbit/bxdfs.h"
#include "bulbit/image3d.h"
#include "bulbit/parallel_for.h"
#include "bulbit/samplers.h"
#include "bulbit/texture3d.h"

namespace fs = std::filesystem;

namespace bulbit
{

static const fs::path folder = "res/refs";

static const fs::path filename_E_tr = folder / "E_tr.hdr";
static const fs::path filename_E_avg_tr = folder / "E_avg_tr.hdr";

static const fs::path filename_E_cs = folder / "E_cs.hdr";

static const fs::path filename_E_d = folder / "E_d.hdr";
static const fs::path filename_E_inv_d = folder / "E_inv_d.hdr";

static const fs::path filename_E_dm = folder / "E_dm.hdr";
static const fs::path filename_E_inv_dm = folder / "E_inv_dm.hdr";
static const fs::path filename_E_avg_dm = folder / "E_avg_dm.hdr";
static const fs::path filename_E_inv_avg_dm = folder / "E_inv_avg_dm.hdr";

// TODO: Handle cases where the loaded texture sizes are different

void TrowbridgeReitzDistribution::PrepareReflectanceTexture(int32 texture_size, std::span<Float> u0, std::span<Point2> u12)
{
    BulbitNotUsed(u0);

    Image1f image_e, image_e_avg;

    if (fs::exists(filename_E_tr) && fs::exists(filename_E_avg_tr))
    {
        image_e = ReadImage1(filename_E_tr, 0, true);
        image_e_avg = ReadImage1(filename_E_avg_tr, 0, true);
    }
    else
    {
        image_e = Image1f(texture_size, texture_size);
        image_e_avg = Image1f(texture_size, 1);

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
                Float r = distribution.rho(wo, u12);

                r_sum += r * cos_theta * d;

                image_e(i, j) = r;
            }

            image_e_avg(j, 0) = 2 * r_sum;
        });

        WriteImage(image_e, filename_E_tr);
        WriteImage(image_e_avg, filename_E_avg_tr);
    }

    E_texture = std::make_unique<FloatImageTexture>(std::move(image_e), TexCoordFilter::clamp);
    E_avg_texture = std::make_unique<FloatImageTexture>(std::move(image_e_avg), TexCoordFilter::clamp);
}

void CharlieSheenDistribution::PrepareReflectanceTexture(int32 texture_size, std::span<Float> u0, std::span<Point2> u12)
{
    BulbitNotUsed(u0);

    Image1f image;
    if (fs::exists(filename_E_cs))
    {
        image = ReadImage1(filename_E_cs, 0, true);
    }
    else
    {
        image = Image1f(texture_size, texture_size);

        const Float d = 1.0f / texture_size;
        ParallelFor(0, texture_size, [&](int32 j) {
            Float a = d / 2 + d * j;

            for (int32 i = 0; i < texture_size; ++i)
            {
                Float cos_theta = d / 2 + d * i;
                Float sin_theta = std::sqrt(1 - Sqr(cos_theta));

                Vec3 wo(sin_theta, 0, cos_theta);

                CharlieSheenDistribution distribution(a);
                Float r = distribution.rho(wo, u12);

                image(i, j) = r;
            }
        });

        WriteImage(image, filename_E_cs);
    }

    E_texture = std::make_unique<FloatImageTexture>(std::move(image), TexCoordFilter::clamp);
}

void DielectricBxDF::PrepareReflectanceTexture(int32 texture_size, std::span<Float> u0, std::span<Point2> u12)
{
    Image3D1f image_e, image_e_inv;

    if (fs::exists(filename_E_d) && fs::exists(filename_E_inv_d))
    {
        image_e = ReadImage3D(filename_E_d, 0, Point3i(texture_size), true);
        image_e_inv = ReadImage3D(filename_E_inv_d, 0, Point3i(texture_size), true);
    }
    else
    {
        image_e = Image3D1f(texture_size, texture_size, texture_size);
        image_e_inv = Image3D1f(texture_size, texture_size, texture_size);

        const Float d = 1.0f / texture_size;

        ParallelFor(0, texture_size, [&](int32 k) {
            Float f = d / 2 + d * k;
            Float ior = MapF0toIOR(f);

            for (int32 j = 0; j < texture_size; ++j)
            {
                Float a = d / 2 + d * j;

                for (int32 i = 0; i < texture_size; ++i)
                {
                    Float cos_theta = d / 2 + d * i;
                    Float sin_theta = std::sqrt(1 - Sqr(cos_theta));

                    Vec3 wo(sin_theta, 0, cos_theta);

                    DielectricBxDF bsdf_i(ior, TrowbridgeReitzDistribution(a, a), Spectrum(1));
                    DielectricBxDF bsdf_t(1 / ior, TrowbridgeReitzDistribution(a, a), Spectrum(1));

                    Float r = bsdf_i.rho(wo, u0, u12, TransportDirection::ToLight).Average();
                    Float r_inv = bsdf_t.rho(wo, u0, u12, TransportDirection::ToLight).Average();

                    image_e(k, i, j) = r;
                    image_e_inv(k, i, j) = r_inv;
                }
            }
        });

        WriteImage3D(image_e, filename_E_d);
        WriteImage3D(image_e_inv, filename_E_inv_d);
    }

    E_texture = std::make_unique<FloatImageTexture3D>(std::move(image_e), TexCoordFilter::clamp);
    E_inv_texture = std::make_unique<FloatImageTexture3D>(std::move(image_e_inv), TexCoordFilter::clamp);
}

void DielectricMultiScatteringBxDF::PrepareReflectanceTexture(int32 texture_size, std::span<Float> u0, std::span<Point2> u12)
{
    Image3D1f image_e, image_e_inv;
    Image1f image_e_avg, image_e_inv_avg;

    if (fs::exists(filename_E_dm) && fs::exists(filename_E_inv_dm) && fs::exists(filename_E_avg_dm) &&
        fs::exists(filename_E_inv_avg_dm))
    {
        image_e = ReadImage3D(filename_E_dm, 0, Point3i(texture_size), true);
        image_e_inv = ReadImage3D(filename_E_inv_dm, 0, Point3i(texture_size), true);
        image_e_avg = ReadImage1(filename_E_avg_dm, 0, true);
        image_e_inv_avg = ReadImage1(filename_E_inv_avg_dm, 0, true);
    }
    else
    {
        image_e = Image3D1f(texture_size, texture_size, texture_size);
        image_e_inv = Image3D1f(texture_size, texture_size, texture_size);
        image_e_avg = Image1f(texture_size, texture_size);
        image_e_inv_avg = Image1f(texture_size, texture_size);

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

                    DielectricBxDF bsdf_i(ior, TrowbridgeReitzDistribution(a, a), Spectrum(1));
                    DielectricBxDF bsdf_t(1 / ior, TrowbridgeReitzDistribution(a, a), Spectrum(1));

                    Float r = bsdf_i.rho(wo, u0, u12, TransportDirection::ToCamera).Average();
                    Float r_inv = bsdf_t.rho(wo, u0, u12, TransportDirection::ToCamera).Average();

                    r_sum += r * cos_theta * d;
                    r_inv_sum += r_inv * cos_theta * d;

                    image_e(k, i, j) = r;
                    image_e_inv(k, i, j) = r_inv;
                }

                image_e_avg(k, j) = 2 * r_sum;
                image_e_inv_avg(k, j) = 2 * r_inv_sum;
            }
        });

        WriteImage3D(image_e, filename_E_dm);
        WriteImage3D(image_e_inv, filename_E_inv_dm);
        WriteImage(image_e_avg, filename_E_avg_dm);
        WriteImage(image_e_inv_avg, filename_E_inv_avg_dm);
    }

    E_texture = std::make_unique<FloatImageTexture3D>(std::move(image_e), TexCoordFilter::clamp);
    E_inv_texture = std::make_unique<FloatImageTexture3D>(std::move(image_e_inv), TexCoordFilter::clamp);
    E_avg_texture = std::make_unique<FloatImageTexture>(std::move(image_e_avg), TexCoordFilter::clamp);
    E_inv_avg_texture = std::make_unique<FloatImageTexture>(std::move(image_e_inv_avg), TexCoordFilter::clamp);
}

void PrepareReflectanceTextures(int32 texture_size, int32 num_samples)
{
    if (!fs::exists(folder))
    {
        fs::create_directories(folder);
    }

    int32 x_samples = int32(std::sqrt(num_samples));
    int32 y_samples = x_samples;

    int32 samples = x_samples * y_samples;
    BulbitAssert(num_samples == samples);

    std::vector<Float> u0(samples);
    std::vector<Point2> u12(samples);

    const uint32 hash_uc = 1;
    const uint32 hash_u = 2;
    RNG rng(3);

    for (int32 i = 0; i < samples; ++i)
    {
        {
            int32 stratum = PermutationElement(i, samples, hash_uc);
            u0[i] = (stratum + rng.NextFloat()) / samples;
        }

        {
            int32 stratum = PermutationElement(i, samples, hash_u);

            int32 x = stratum % x_samples;
            int32 y = stratum / x_samples;

            u12[i] = { (x + rng.NextFloat()) / x_samples, (y + rng.NextFloat()) / y_samples };
        }
    }

    TrowbridgeReitzDistribution::PrepareReflectanceTexture(texture_size, u0, u12);
    CharlieSheenDistribution::PrepareReflectanceTexture(texture_size, u0, u12);
    DielectricBxDF::PrepareReflectanceTexture(texture_size, u0, u12);
    DielectricMultiScatteringBxDF::PrepareReflectanceTexture(texture_size, u0, u12);
}

} // namespace bulbit
