#pragma once

#include "bxdf.h"
#include "media.h"
#include "microfacet.h"
#include "scattering.h"
#include "textures3d.h"

namespace bulbit
{

class LambertianBxDF : public BxDF
{
public:
    LambertianBxDF(const SpectrumSample& reflectance)
        : r{ reflectance }
    {
    }

    virtual BxDF_Flags Flags() const override
    {
        return !r.IsBlack() ? BxDF_Flags::DiffuseReflection : BxDF_Flags::Unset;
    }

    virtual SpectrumSample f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const override;
    virtual Float PDF(
        Vec3 wo,
        Vec3 wi,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

    virtual bool Sample_f(
        BSDFSample* sample,
        Vec3 wo,
        Float u0,
        Point2 u12,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

private:
    // Reflectance [0, 1]
    SpectrumSample r;
};

class SpecularReflectionBxDF : public BxDF
{
public:
    SpecularReflectionBxDF(const SpectrumSample& reflectance)
        : r{ reflectance }
    {
    }

    virtual BxDF_Flags Flags() const override
    {
        return !r.IsBlack() ? BxDF_Flags::SpecularReflection : BxDF_Flags::Unset;
    }

    virtual SpectrumSample f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const override;
    virtual Float PDF(
        Vec3 wo,
        Vec3 wi,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

    virtual bool Sample_f(
        BSDFSample* sample,
        Vec3 wo,
        Float u0,
        Point2 u12,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

private:
    // Reflectance [0, 1]
    SpectrumSample r;
};

class DielectricBxDF : public BxDF
{
public:
    DielectricBxDF(Float eta, TrowbridgeReitzDistribution mf, SpectrumSample r)
        : DielectricBxDF(SpectrumSample(eta), mf, r)
    {
    }

    DielectricBxDF(SpectrumSample eta, TrowbridgeReitzDistribution mf, SpectrumSample r)
        : eta{ eta }
        , mf{ mf }
        , r{ Sqrt(r) }
    {
    }

    virtual BxDF_Flags Flags() const override
    {
        BxDF_Flags flags = HeroEta() == 1 ? BxDF_Flags::Transmission : (BxDF_Flags::Reflection | BxDF_Flags::Transmission);
        return flags | (mf.EffectivelySmooth() ? BxDF_Flags::Specular : BxDF_Flags::Glossy);
    }

    virtual SpectrumSample f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const override;
    virtual Float PDF(
        Vec3 wo,
        Vec3 wi,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

    virtual bool Sample_f(
        BSDFSample* sample,
        Vec3 wo,
        Float u0,
        Point2 u12,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;
    virtual void Regularize() override;

    static void PrepareReflectanceTexture(int32 texture_size, std::span<Float> uc, std::span<Point2> u);

    static Float E(Vec3 wo, Float eta, Float alpha)
    {
        BulbitAssert(E_texture != nullptr);
        BulbitAssert(E_inv_texture != nullptr);

        if (eta >= 1)
        {
            Float f0 = MapIORtoF0(eta);
            return E_texture->Evaluate({ f0, std::abs(wo.z), alpha });
        }
        else
        {
            Float f0 = MapIORtoF0(1 / eta);
            return E_inv_texture->Evaluate({ f0, std::abs(wo.z), alpha });
        }
    }

private:
    Float HeroEta() const
    {
        return eta[WavelengthSample::hero_lane];
    }

    static inline std::unique_ptr<FloatImageTexture3D> E_texture = nullptr;
    static inline std::unique_ptr<FloatImageTexture3D> E_inv_texture = nullptr;

    SpectrumSample eta;
    TrowbridgeReitzDistribution mf;
    SpectrumSample r;
};

class DielectricMultiScatteringBxDF : public BxDF
{
public:
    DielectricMultiScatteringBxDF(Float eta, TrowbridgeReitzDistribution mf, SpectrumSample r)
        : DielectricMultiScatteringBxDF(SpectrumSample(eta), mf, r)
    {
    }

    DielectricMultiScatteringBxDF(SpectrumSample eta, TrowbridgeReitzDistribution mf, SpectrumSample r)
        : eta{ eta }
        , mf{ mf }
        , r{ Sqrt(r) }
    {
    }

    virtual BxDF_Flags Flags() const override
    {
        BxDF_Flags flags = HeroEta() == 1 ? BxDF_Flags::Transmission : (BxDF_Flags::Reflection | BxDF_Flags::Transmission);
        return flags | (mf.EffectivelySmooth() ? BxDF_Flags::Specular : BxDF_Flags::Glossy);
    }

    virtual SpectrumSample f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const override;
    virtual Float PDF(
        Vec3 wo,
        Vec3 wi,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

    virtual bool Sample_f(
        BSDFSample* sample,
        Vec3 wo,
        Float u0,
        Point2 u12,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;
    virtual void Regularize() override;

    Float E(Vec3 wo, Float eta) const
    {
        BulbitAssert(E_texture != nullptr);
        BulbitAssert(E_inv_texture != nullptr);

        if (eta >= 1)
        {
            Float f0 = MapIORtoF0(eta);
            return E_texture->Evaluate({ f0, std::abs(wo.z), mf.GetMeanAlpha() });
        }
        else
        {
            Float f0 = MapIORtoF0(1 / eta);
            return E_inv_texture->Evaluate({ f0, std::abs(wo.z), mf.GetMeanAlpha() });
        }
    }

    Float E_avg(Float eta) const
    {
        BulbitAssert(E_avg_texture != nullptr);
        BulbitAssert(E_inv_avg_texture != nullptr);

        if (eta >= 1)
        {
            Float f0 = MapIORtoF0(eta);
            return E_avg_texture->Evaluate({ f0, mf.GetMeanAlpha() });
        }
        else
        {
            Float f0 = MapIORtoF0(1 / eta);
            return E_inv_avg_texture->Evaluate({ f0, mf.GetMeanAlpha() });
        }
    }

    static void PrepareReflectanceTexture(int32 texture_size, std::span<Float> uc, std::span<Point2> u);

private:
    static inline std::unique_ptr<FloatImageTexture3D> E_texture = nullptr;
    static inline std::unique_ptr<FloatImageTexture3D> E_inv_texture = nullptr;
    static inline std::unique_ptr<FloatImageTexture> E_avg_texture = nullptr;
    static inline std::unique_ptr<FloatImageTexture> E_inv_avg_texture = nullptr;

    Float ComputeScatteringRatio(Float eta_forward, Float eta_o) const
    {
        Float eta_i = eta_forward >= 1 ? eta_forward : 1 / eta_forward;
        Float eta_t = 1 / eta_i;

        Float a = (1 - FresnelDielectricAverage(eta_i)) / (1 - E_avg(eta_t));
        Float b = Sqr(eta_i) * (1 - FresnelDielectricAverage(eta_t)) / (1 - E_avg(eta_i));
        Float x = 1 / (a / b + 1);

        Float ratio;
        if (eta_o >= 1)
        {
            ratio = 1 - x * (1 - FresnelDielectricAverage(eta_o));
        }
        else
        {
            ratio = 1 - (1 - x) * (1 - FresnelDielectricAverage(eta_o));
        }

        return ratio;
    }

    Float HeroEta() const
    {
        return eta[WavelengthSample::hero_lane];
    }

    SpectrumSample eta;
    TrowbridgeReitzDistribution mf;
    SpectrumSample r;
};

class ConductorBxDF : public BxDF
{
    // Physically correct BRDF assumes reflectance = 1
public:
    ConductorBxDF(
        SpectrumSample eta, SpectrumSample k, TrowbridgeReitzDistribution mf, SpectrumSample reflectance = SpectrumSample(1)
    )
        : mf{ mf }
        , eta{ eta }
        , k{ k }
        , reflectance{ reflectance }
    {
    }

    ConductorBxDF(SpectrumSample R, TrowbridgeReitzDistribution mf, SpectrumSample reflectance = SpectrumSample(1))
        : mf{ mf }
        , eta{ 1 }
        , reflectance{ reflectance }
    {
        // The reflectance R for a conductor is:
        // R = \frac{(\eta - 1)^2 + k^2}{(\eta + 1)^2 + k^2}
        // Assume \eta = 1 and solve for k

        SpectrumSample r = Clamp(R, 0, 1 - 1e-4f);
        k = 2 * Sqrt(r) / Sqrt(Max(SpectrumSample(1) - r, 0));
    }

    virtual BxDF_Flags Flags() const override
    {
        return mf.EffectivelySmooth() ? BxDF_Flags::SpecularReflection : BxDF_Flags::GlossyReflection;
    }

    virtual SpectrumSample f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const override;
    virtual Float PDF(
        Vec3 wo,
        Vec3 wi,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

    virtual bool Sample_f(
        BSDFSample* sample,
        Vec3 wo,
        Float u0,
        Point2 u12,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

    virtual void Regularize() override;

private:
    TrowbridgeReitzDistribution mf;
    SpectrumSample eta, k, reflectance;
};

class ConductorMultiScatteringBxDF : public BxDF
{
    // Physically correct BRDF assumes reflectance = 1
public:
    ConductorMultiScatteringBxDF(
        SpectrumSample eta, SpectrumSample k, TrowbridgeReitzDistribution mf, SpectrumSample reflectance = SpectrumSample(1)
    )
        : mf{ mf }
        , eta{ eta }
        , k{ k }
        , reflectance{ reflectance }
    {
    }

    ConductorMultiScatteringBxDF(SpectrumSample R, TrowbridgeReitzDistribution mf, SpectrumSample reflectance = SpectrumSample(1))
        : mf{ mf }
        , eta{ 1 }
        , reflectance{ reflectance }
    {
        // The reflectance R for a conductor is:
        // R = \frac{(\eta - 1)^2 + k^2}{(\eta + 1)^2 + k^2}
        // Assume \eta = 1 and solve for k

        SpectrumSample r = Clamp(R, 0, 1 - 1e-4f);
        k = 2 * Sqrt(r) / Sqrt(Max(SpectrumSample(1) - r, 0));
    }

    virtual BxDF_Flags Flags() const override
    {
        return mf.EffectivelySmooth() ? BxDF_Flags::SpecularReflection : BxDF_Flags::GlossyReflection;
    }

    virtual SpectrumSample f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const override;
    virtual Float PDF(
        Vec3 wo,
        Vec3 wi,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

    virtual bool Sample_f(
        BSDFSample* sample,
        Vec3 wo,
        Float u0,
        Point2 u12,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

    virtual void Regularize() override;

private:
    TrowbridgeReitzDistribution mf;
    SpectrumSample eta, k, reflectance;
};

class ThinDielectricBxDF : public BxDF
{
public:
    ThinDielectricBxDF(Float eta, SpectrumSample r)
        : ThinDielectricBxDF(SpectrumSample(eta), r)
    {
    }

    ThinDielectricBxDF(SpectrumSample eta, SpectrumSample r)
        : eta{ eta }
        , r{ Sqrt(r) }
    {
    }

    virtual BxDF_Flags Flags() const override
    {
        return BxDF_Flags::Specular | BxDF_Flags::Reflection | BxDF_Flags::Transmission;
    }

    virtual SpectrumSample f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const override;
    virtual Float PDF(
        Vec3 wo,
        Vec3 wi,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

    virtual bool Sample_f(
        BSDFSample* sample,
        Vec3 wo,
        Float u0,
        Point2 u12,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

private:
    SpectrumSample eta;
    SpectrumSample r;
};

class SheenBxDF : public BxDF
{
public:
    SheenBxDF(SpectrumSample base, SpectrumSample sheen, CharlieSheenDistribution mf)
        : base{ base }
        , sheen{ sheen }
        , mf{ mf }
    {
    }

    virtual BxDF_Flags Flags() const override
    {
        return BxDF_Flags::Diffuse | BxDF_Flags::Reflection;
    }

    virtual SpectrumSample f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const override;
    virtual Float PDF(
        Vec3 wo,
        Vec3 wi,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

    virtual bool Sample_f(
        BSDFSample* sample,
        Vec3 wo,
        Float u0,
        Point2 u12,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

    virtual void Regularize() override {}

private:
    SpectrumSample base, sheen;
    CharlieSheenDistribution mf;
};

class MetallicRoughnessBxDF : public BxDF
{
public:
    MetallicRoughnessBxDF(SpectrumSample color, Float metallic, TrowbridgeReitzDistribution mf)
        : color{ color }
        , metallic{ metallic }
        , mf{ mf }
    {
    }

    virtual BxDF_Flags Flags() const override
    {
        return BxDF_Flags::Diffuse | BxDF_Flags::Glossy | BxDF_Flags::Reflection;
    }

    virtual SpectrumSample f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const override;
    virtual Float PDF(
        Vec3 wo,
        Vec3 wi,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

    virtual bool Sample_f(
        BSDFSample* sample,
        Vec3 wo,
        Float u0,
        Point2 u12,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

    virtual void Regularize() override;

    static const inline Float detault_ior = 1.5f;
    static const inline SpectrumSample default_dielectric_f0 = SpectrumSample(0.04f);
    static const inline Float min_alpha = 1e-3f;

    static Float RoughnessToAlpha(Float roughness)
    {
        return std::fmax(TrowbridgeReitzDistribution::RoughnessToAlpha(roughness), min_alpha);
    }

    static SpectrumSample F0(Float ior, SpectrumSample color, Float metallic)
    {
        SpectrumSample f0(Sqr((ior - 1) / (ior + 1)));
        return Lerp(f0, color, metallic);
    }

    static SpectrumSample F0(SpectrumSample color, Float metallic)
    {
        return Lerp(default_dielectric_f0, color, metallic);
    }

    static SpectrumSample F_Schlick(SpectrumSample f0, Float cosine_theta)
    {
        return f0 + (/*f90*/ SpectrumSample(1) - f0) * std::pow(1 - cosine_theta, 5.0f);
    }

private:
    SpectrumSample color;
    Float metallic;
    TrowbridgeReitzDistribution mf;
};

class SubstrateBxDF : public BxDF
{
    // Rough dielectric coating + diffuse substrate model.
    // The reflected BSDF is a sum of:
    // 1) glossy microfacet reflection at the top interface
    // 2) diffuse substrate reflection attenuated by in/out Fresnel terms
public:
    SubstrateBxDF(
        SpectrumSample reflectance, Float ior, TrowbridgeReitzDistribution mf, SpectrumSample sigma_a, Float thickness = 1.0f
    )
        : reflectance{ Clamp(reflectance, 0, 1) }
        , eta{ std::max(ior, 1.0001f) }
        , mf{ mf }
        , sigma_a_dt{ Max(sigma_a, 0) * std::max(thickness, 0.0f) }
        , fresnel_avg{ FresnelDielectricAverage(1 / eta) }
        , avg_transmittance{ std::exp(-2 * sigma_a_dt.Average()) }
    {
    }

    virtual BxDF_Flags Flags() const override
    {
        BxDF_Flags flags = BxDF_Flags::GlossyReflection;
        if (!reflectance.IsBlack())
        {
            flags = flags | BxDF_Flags::DiffuseReflection;
        }
        return flags;
    }

    virtual SpectrumSample f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const override;
    virtual Float PDF(
        Vec3 wo,
        Vec3 wi,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

    virtual bool Sample_f(
        BSDFSample* sample,
        Vec3 wo,
        Float u0,
        Point2 u12,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

    virtual void Regularize() override;

    static const inline Float min_alpha = 1e-3f;

    static Float RoughnessToAlpha(Float roughness)
    {
        return std::fmax(TrowbridgeReitzDistribution::RoughnessToAlpha(roughness), min_alpha);
    }

private:
    Float GlossyProbability(Vec3 wo) const;

    SpectrumSample reflectance;     // Substrate diffuse albedo (rho_d)

    Float eta;                      // Relative IOR for the top dielectric interface
    TrowbridgeReitzDistribution mf; // Microfacet distribution for the coating

    SpectrumSample sigma_a_dt;      // sigma_a * thickness, used for Beer-Lambert attenuation

    Float fresnel_avg;              // Hemispherical-average Fresnel term for diffuse normalization
    Float avg_transmittance;        // Average round-trip transmittance through the coating
};

class PrincipledBxDF : public BxDF
{
public:
    PrincipledBxDF(
        SpectrumSample color,
        Float metallic,
        TrowbridgeReitzDistribution mf,
        Float eta,
        Float transmission,
        Float clearcoat,
        TrowbridgeReitzDistribution mf_clearcoat,
        SpectrumSample clearcoat_color,
        Float sheen,
        CharlieSheenDistribution mf_sheen,
        SpectrumSample sheen_color
    )
        : color{ color }
        , metallic{ metallic }
        , mf{ mf }
        , eta{ eta }
        , transmission{ transmission }
        , clearcoat{ clearcoat }
        , mf_clearcoat{ mf_clearcoat }
        , clearcoat_color{ clearcoat_color }
        , sheen{ sheen }
        , mf_sheen{ mf_sheen }
        , sheen_color{ sheen_color }
    {
    }

    virtual BxDF_Flags Flags() const override
    {
        return BxDF_Flags::Diffuse | BxDF_Flags::Glossy | BxDF_Flags::Reflection | BxDF_Flags::Transmission;
    }

    virtual SpectrumSample f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const override;
    virtual Float PDF(
        Vec3 wo,
        Vec3 wi,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

    virtual bool Sample_f(
        BSDFSample* sample,
        Vec3 wo,
        Float u0,
        Point2 u12,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

    virtual void Regularize() override;

    static const inline Float default_clearcoat_ior = 1.5f;
    static const inline SpectrumSample default_dielectric_f0 = SpectrumSample(0.04f);
    static const inline Float min_alpha = 1e-3f;

    static Float RoughnessToAlpha(Float roughness)
    {
        return std::fmax(TrowbridgeReitzDistribution::RoughnessToAlpha(roughness), min_alpha);
    }

    static Point2 RoughnessToAlpha(Float roughness, Float anisotropy)
    {
        Float alpha = RoughnessToAlpha(roughness);
        Float ratio = std::sqrt(1 - 0.9f * anisotropy);

        Float alpha_x = alpha / ratio;
        Float alpha_y = alpha * ratio;

        return { alpha_x, alpha_y };
    }

    static SpectrumSample F0(Float ior, SpectrumSample color, Float metallic)
    {
        SpectrumSample f0(Sqr((ior - 1) / (ior + 1)));
        return Lerp(f0, color, metallic);
    }

    static SpectrumSample F_Schlick(SpectrumSample f0, Float cos_theta)
    {
        return f0 + (/*f90*/ SpectrumSample(1) - f0) * std::pow(1 - cos_theta, 5.0f);
    }

    static SpectrumSample F_avg_Schlick(SpectrumSample f0)
    {
        return (SpectrumSample(1) + 20 * f0) / 21;
    }

private:
    SpectrumSample color;
    Float metallic;
    TrowbridgeReitzDistribution mf;
    Float eta;
    Float transmission;
    Float clearcoat;
    TrowbridgeReitzDistribution mf_clearcoat;
    SpectrumSample clearcoat_color;
    Float sheen;
    CharlieSheenDistribution mf_sheen;
    SpectrumSample sheen_color;
};

class NormalizedFresnelBxDF : public BxDF
{
public:
    NormalizedFresnelBxDF(Float eta)
        : eta{ eta }
    {
    }

    virtual BxDF_Flags Flags() const override
    {
        return BxDF_Flags(BxDF_Flags::Diffuse | BxDF_Flags::Reflection);
    }

    virtual SpectrumSample f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const override;
    virtual Float PDF(
        Vec3 wo,
        Vec3 wi,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

    virtual bool Sample_f(
        BSDFSample* sample,
        Vec3 wo,
        Float u0,
        Point2 u12,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

    virtual void Regularize() override;

private:
    Float eta;
};

class EONBxDF : public BxDF
{
    // Energy-preserving Oren?밡ayar BRDF
    // EON: APractical Energy-Preserving Rough Diffuse BRDF (Portsmouth et al., 2025)
    // https://jcgt.org/published/0014/01/06/
public:
    EONBxDF(const SpectrumSample& reflectance, Float roughness, bool exact = true)
        : rho{ reflectance }
        , r{ roughness }
        , exact{ exact }
    {
    }

    virtual BxDF_Flags Flags() const override
    {
        return !rho.IsBlack() ? BxDF_Flags::DiffuseReflection : BxDF_Flags::Unset;
    }

    virtual SpectrumSample f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const override;
    virtual Float PDF(
        Vec3 wo,
        Vec3 wi,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

    virtual bool Sample_f(
        BSDFSample* sample,
        Vec3 wo,
        Float u0,
        Point2 u12,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

private:
    SpectrumSample rho;
    Float r;
    bool exact;
};

class LayeredBxDF : public BxDF
{
public:
    LayeredBxDF(
        BxDF* top,
        BxDF* bottom,
        bool two_sided,
        const SpectrumSample& albedo,
        Float thickness,
        Float g = 0,
        int32 max_bounces = 16,
        int32 samples = 1
    )
        : top{ top }
        , bottom{ bottom }
        , two_sided{ two_sided }
        , albedo{ albedo }
        , thickness{ thickness }
        , g{ g }
        , max_bounces{ max_bounces }
        , samples{ samples }
    {
    }

    virtual BxDF_Flags Flags() const override
    {
        BxDF_Flags top_flags = top->Flags();
        BxDF_Flags bottom_flags = bottom->Flags();

        BxDF_Flags flags = BxDF_Flags::Reflection;

        if (IsSpecular(top_flags))
        {
            flags = flags | BxDF_Flags::Specular;
        }

        if (IsDiffuse(top_flags) || IsDiffuse(bottom_flags) || !albedo.IsBlack())
        {
            flags = flags | BxDF_Flags::Diffuse;
        }
        else if (IsGlossy(top_flags) || IsGlossy(bottom_flags))
        {
            flags = flags | BxDF_Flags::Glossy;
        }

        if (IsTransmissive(top_flags) && IsTransmissive(bottom_flags))
        {
            flags = flags | BxDF_Flags::Transmission;
        }

        return flags;
    }

    virtual SpectrumSample f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const override;
    virtual Float PDF(
        Vec3 wo,
        Vec3 wi,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

    virtual bool Sample_f(
        BSDFSample* sample,
        Vec3 wo,
        Float u0,
        Point2 u12,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

    virtual void Regularize() override
    {
        top->Regularize();
        bottom->Regularize();
    }

private:
    static Float Tr(Float dz, Vec3 w)
    {
        return std::exp(-std::abs(dz / w.z));
    }

    BxDF* top;
    BxDF* bottom;
    bool two_sided;

    SpectrumSample albedo; // single scattering albedo
    Float thickness, g;

    int32 max_bounces, samples;
};

constexpr size_t max_bxdf_size = std::max(
    { sizeof(LambertianBxDF), sizeof(SpecularReflectionBxDF), sizeof(DielectricBxDF), sizeof(ConductorBxDF),
      sizeof(DielectricMultiScatteringBxDF), sizeof(SubstrateBxDF), sizeof(ConductorMultiScatteringBxDF),
      sizeof(ThinDielectricBxDF), sizeof(SheenBxDF), sizeof(MetallicRoughnessBxDF), sizeof(PrincipledBxDF),
      sizeof(NormalizedFresnelBxDF), sizeof(EONBxDF), sizeof(LayeredBxDF) }
);

} // namespace bulbit
