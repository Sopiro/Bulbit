#pragma once

#include "bxdf.h"
#include "microfacet.h"
#include "scattering.h"
#include "textures3d.h"

namespace bulbit
{

class LambertianBxDF : public BxDF
{
public:
    LambertianBxDF(const Spectrum& reflectance)
        : r{ reflectance }
    {
    }

    virtual BxDF_Flags Flags() const override
    {
        return !r.IsBlack() ? BxDF_Flags::DiffuseReflection : BxDF_Flags::Unset;
    }

    virtual Spectrum f(const Vec3& wo, const Vec3& wi, TransportDirection direction = TransportDirection::ToLight) const override;
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
    Spectrum r;
};

class SpecularReflectionBxDF : public BxDF
{
public:
    SpecularReflectionBxDF(const Spectrum& reflectance)
        : r{ reflectance }
    {
    }

    virtual BxDF_Flags Flags() const override
    {
        return !r.IsBlack() ? BxDF_Flags::SpecularReflection : BxDF_Flags::Unset;
    }

    virtual Spectrum f(const Vec3& wo, const Vec3& wi, TransportDirection direction = TransportDirection::ToLight) const override;
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
    Spectrum r;
};

class DielectricBxDF : public BxDF
{
public:
    DielectricBxDF(Float eta, Spectrum r, TrowbridgeReitzDistribution mf, bool ms)
        : eta{ eta }
        , r{ Sqrt(r) }
        , mf{ mf }
        , ms{ ms }
    {
    }

    virtual BxDF_Flags Flags() const override
    {
        BxDF_Flags flags = (eta == 1) ? BxDF_Flags::Transmission : (BxDF_Flags::Reflection | BxDF_Flags::Transmission);
        return flags | (mf.EffectivelySmooth() ? BxDF_Flags::Specular : BxDF_Flags::Glossy);
    }

    virtual Spectrum f(const Vec3& wo, const Vec3& wi, TransportDirection direction = TransportDirection::ToLight) const override;
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

    Float E(Vec3 wo) const
    {
        BulbitAssert(E_texture != nullptr);
        BulbitAssert(E_inv_texture != nullptr);

        Float alpha = std::sqrt(mf.alpha_x * mf.alpha_y);
        if (eta >= 1.0f)
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

    Float E_inv(Vec3 wi) const
    {
        BulbitAssert(E_texture != nullptr);
        BulbitAssert(E_inv_texture != nullptr);

        Float alpha = std::sqrt(mf.alpha_x * mf.alpha_y);
        if (eta >= 1.0f)
        {
            Float f0 = MapIORtoF0(eta);
            return E_inv_texture->Evaluate({ f0, std::abs(wi.z), alpha });
        }
        else
        {
            Float f0 = MapIORtoF0(1 / eta);
            return E_texture->Evaluate({ f0, std::abs(wi.z), alpha });
        }
    }

    Float E_avg() const
    {
        BulbitAssert(E_avg_texture != nullptr);
        BulbitAssert(E_inv_avg_texture != nullptr);

        Float alpha = std::sqrt(mf.alpha_x * mf.alpha_y);
        if (eta >= 1.0f)
        {
            Float f0 = MapIORtoF0(eta);
            return E_avg_texture->Evaluate({ f0, alpha });
        }
        else
        {
            Float f0 = MapIORtoF0(1 / eta);
            return E_inv_avg_texture->Evaluate({ f0, alpha });
        }
    }

    Float E_avg_inv() const
    {
        BulbitAssert(E_avg_texture != nullptr);
        BulbitAssert(E_inv_avg_texture != nullptr);

        Float alpha = std::sqrt(mf.alpha_x * mf.alpha_y);
        if (eta >= 1.0f)
        {
            Float f0 = MapIORtoF0(eta);
            return E_inv_avg_texture->Evaluate({ f0, alpha });
        }
        else
        {
            Float f0 = MapIORtoF0(1 / eta);
            return E_avg_texture->Evaluate({ f0, alpha });
        }
    }

    static void ComputeReflectanceTexture(int32 texture_size, std::span<Float> uc, std::span<Point2> u);

private:
    static inline std::unique_ptr<FloatImageTexture3D> E_texture = nullptr;
    static inline std::unique_ptr<FloatImageTexture3D> E_inv_texture = nullptr;
    static inline std::unique_ptr<FloatImageTexture> E_avg_texture = nullptr;
    static inline std::unique_ptr<FloatImageTexture> E_inv_avg_texture = nullptr;

    Float eta;

    Spectrum r;
    TrowbridgeReitzDistribution mf;
    bool ms;
};

class ConductorBxDF : public BxDF
{
public:
    ConductorBxDF(Spectrum eta, Spectrum k, TrowbridgeReitzDistribution mf, bool ms)
        : mf{ mf }
        , eta{ eta }
        , k{ k }
        , ms{ ms }
    {
    }

    virtual BxDF_Flags Flags() const override
    {
        return mf.EffectivelySmooth() ? BxDF_Flags::SpecularReflection : BxDF_Flags::GlossyReflection;
    }

    virtual Spectrum f(const Vec3& wo, const Vec3& wi, TransportDirection direction = TransportDirection::ToLight) const override;
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
    Spectrum eta, k;
    bool ms;
};

class ThinDielectricBxDF : public BxDF
{
public:
    ThinDielectricBxDF(Float eta)
        : eta{ eta }
    {
    }

    virtual BxDF_Flags Flags() const override
    {
        return BxDF_Flags::Specular | BxDF_Flags::Reflection | BxDF_Flags::Transmission;
    }

    virtual Spectrum f(const Vec3& wo, const Vec3& wi, TransportDirection direction = TransportDirection::ToLight) const override;
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
    Float eta;
};

class MetallicRoughnessBxDF : public BxDF
{
public:
    MetallicRoughnessBxDF(Spectrum color, Float metallic, TrowbridgeReitzDistribution mf)
        : color{ color }
        , metallic{ metallic }
        , mf{ mf }
    {
    }

    virtual BxDF_Flags Flags() const override
    {
        return BxDF_Flags::Diffuse | BxDF_Flags::Glossy | BxDF_Flags::Reflection;
    }

    virtual Spectrum f(const Vec3& wo, const Vec3& wi, TransportDirection direction = TransportDirection::ToLight) const override;
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
    static const inline Spectrum default_dielectric_f0 = Spectrum(0.04f);
    static const inline Float min_alpha = 1e-3f;

    static Float RoughnessToAlpha(Float roughness)
    {
        return std::fmax(TrowbridgeReitzDistribution::RoughnessToAlpha(roughness), min_alpha);
    }

    static Spectrum F0(Float ior, Spectrum color, Float metallic)
    {
        Spectrum f0(Sqr((ior - 1) / (ior + 1)));
        return Lerp(f0, color, metallic);
    }

    static Spectrum F0(Spectrum color, Float metallic)
    {
        return Lerp(default_dielectric_f0, color, metallic);
    }

    static Spectrum F_Schlick(Spectrum f0, Float cosine_theta)
    {
        return f0 + (/*f90*/ Spectrum(1) - f0) * std::pow(1 - cosine_theta, 5.0f);
    }

private:
    Spectrum color;
    Float metallic;
    TrowbridgeReitzDistribution mf;
};

class PrincipledBxDF : public BxDF
{
public:
    PrincipledBxDF(
        Spectrum color,
        Float metallic,
        TrowbridgeReitzDistribution mf,
        Float eta,
        Float transmission,
        Float clearcoat,
        TrowbridgeReitzDistribution mf_clearcoat,
        Spectrum clearcoat_color,
        Float sheen,
        CharlieSheenDistribution mf_sheen,
        Spectrum sheen_color
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

    virtual Spectrum f(const Vec3& wo, const Vec3& wi, TransportDirection direction = TransportDirection::ToLight) const override;
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
    static const inline Spectrum default_dielectric_f0 = Spectrum(0.04f);
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

    static Spectrum F0(Float ior, Spectrum color, Float metallic)
    {
        Spectrum f0(Sqr((ior - 1) / (ior + 1)));
        return Lerp(f0, color, metallic);
    }

    static Spectrum F_Schlick(Spectrum f0, Float cos_theta)
    {
        return f0 + (/*f90*/ Spectrum(1) - f0) * std::pow(1 - cos_theta, 5.0f);
    }

    static Spectrum F_avg_Schlick(Spectrum f0)
    {
        return (Spectrum(1) + 20 * f0) / 21;
    }

private:
    Spectrum color;
    Float metallic;
    TrowbridgeReitzDistribution mf;
    Float eta;
    Float transmission;
    Float clearcoat;
    TrowbridgeReitzDistribution mf_clearcoat;
    Spectrum clearcoat_color;
    Float sheen;
    CharlieSheenDistribution mf_sheen;
    Spectrum sheen_color;
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

    virtual Spectrum f(const Vec3& wo, const Vec3& wi, TransportDirection direction = TransportDirection::ToLight) const override;
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

constexpr size_t max_bxdf_size = std::max(
    { sizeof(LambertianBxDF), sizeof(SpecularReflectionBxDF), sizeof(DielectricBxDF), sizeof(ConductorBxDF),
      sizeof(ThinDielectricBxDF), sizeof(MetallicRoughnessBxDF), sizeof(PrincipledBxDF), sizeof(NormalizedFresnelBxDF) }
);

} // namespace bulbit
