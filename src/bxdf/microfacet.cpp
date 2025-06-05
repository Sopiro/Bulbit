#include "bulbit/microfacet.h"
#include "bulbit/async_job.h"

#define WRITE_REFLECTANCE_TEXTURE 0

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

Float IORtoF0(Float ior)
{
    return Sqr((ior - 1) / (ior + 1));
}

Float F0toIOR(Float f0)
{
    Float sqrt_f0 = std::sqrt(f0);
    return (1 + sqrt_f0) / (1 - sqrt_f0);
}

float MapF0toIOR(Float f0)
{
    return F0toIOR(Sqr(Sqr(f0)));
}

void TrowbridgeReitzDistribution::ComputeReflectanceTexture()
{
    if (rho_texture && rho_avg_texture)
    {
        return;
    }

    Image1f image_e(rho_texture_size, rho_texture_size);
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

            TrowbridgeReitzDistribution distribution(a, a);
            Float r = distribution.rho(wo, u_rho);

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

void CharlieSheenDistribution::ComputeReflectanceTexture()
{
    if (CharlieSheenDistribution::rho_texture)
    {
        return;
    }

    Image1f image(rho_texture_size, rho_texture_size);

    Float d = 1.0f / rho_texture_size;
    for (int32 j = 0; j < rho_texture_size; ++j)
    {
        Float a = d / 2 + d * j;

        for (int32 i = 0; i < rho_texture_size; ++i)
        {
            Float cos_theta = d / 2 + d * i;
            Float sin_theta = std::sqrt(1 - Sqr(cos_theta));

            Vec3 wo(sin_theta, 0, cos_theta);

            CharlieSheenDistribution distribution(a);
            Float r = distribution.rho(wo, u_rho);

            image(i, j) = r;
        }
    }

#if WRITE_REFLECTANCE_TEXTURE
    WriteImage(image, "r_cs.hdr");
#endif

    CharlieSheenDistribution::rho_texture = std::make_unique<FloatImageTexture>(std::move(image), TexCoordFilter::clamp);
}

void ComoputeReflectanceTextures()
{
    std::unique_ptr<AsyncJob<bool>> tr = RunAsync([]() {
        TrowbridgeReitzDistribution::ComputeReflectanceTexture();
        return true;
    });

    std::unique_ptr<AsyncJob<bool>> cs = RunAsync([]() {
        CharlieSheenDistribution::ComputeReflectanceTexture();
        return true;
    });

    tr->Wait();
    cs->Wait();
}

} // namespace bulbit
