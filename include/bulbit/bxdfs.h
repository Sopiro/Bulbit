#pragma once

#include "bxdf.h"
#include "microfacet.h"
#include "scattering.h"

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

    virtual Spectrum f(const Vec3& wo, const Vec3& wi) const override;
    virtual Float PDF(Vec3 wo, Vec3 wi, BxDF_SamplingFlags flags = BxDF_SamplingFlags::All) const override;

    virtual bool Sample_f(
        BSDFSample* sample, Vec3 wo, Float u0, Point2 u12, BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
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

    virtual Spectrum f(const Vec3& wo, const Vec3& wi) const override;
    virtual Float PDF(Vec3 wo, Vec3 wi, BxDF_SamplingFlags flags = BxDF_SamplingFlags::All) const override;

    virtual bool Sample_f(
        BSDFSample* sample, Vec3 wo, Float u0, Point2 u12, BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

private:
    // Reflectance [0, 1]
    Spectrum r;
};

class DielectricBxDF : public BxDF
{
public:
    DielectricBxDF(Float eta, Spectrum r, TrowbridgeReitzDistribution mf)
        : eta{ eta }
        , r{ Sqrt(r) }
        , mf{ mf }
    {
    }

    virtual BxDF_Flags Flags() const override
    {
        BxDF_Flags flags = (eta == 1) ? BxDF_Flags::Transmission : (BxDF_Flags::Reflection | BxDF_Flags::Transmission);
        return flags | (mf.EffectivelySmooth() ? BxDF_Flags::Specular : BxDF_Flags::Glossy);
    }

    virtual Spectrum f(const Vec3& wo, const Vec3& wi) const override;
    virtual Float PDF(Vec3 wo, Vec3 wi, BxDF_SamplingFlags flags = BxDF_SamplingFlags::All) const override;

    virtual bool Sample_f(
        BSDFSample* sample, Vec3 wo, Float u0, Point2 u12, BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

    virtual void Regularize() override;

private:
    Float eta;

    Spectrum r;
    TrowbridgeReitzDistribution mf;
};

class ConductorBxDF : public BxDF
{
public:
    ConductorBxDF(Spectrum eta, Spectrum k, TrowbridgeReitzDistribution mf)
        : mf{ mf }
        , eta{ eta }
        , k{ k }
    {
    }

    virtual BxDF_Flags Flags() const override
    {
        return mf.EffectivelySmooth() ? BxDF_Flags::SpecularReflection : BxDF_Flags::GlossyReflection;
    }

    virtual Spectrum f(const Vec3& wo, const Vec3& wi) const override;
    virtual Float PDF(Vec3 wo, Vec3 wi, BxDF_SamplingFlags flags = BxDF_SamplingFlags::All) const override;

    virtual bool Sample_f(
        BSDFSample* sample, Vec3 wo, Float u0, Point2 u12, BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override;

    virtual void Regularize() override;

private:
    TrowbridgeReitzDistribution mf;
    Spectrum eta, k;
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

    virtual Spectrum f(const Vec3& wo, const Vec3& wi) const override;
    virtual Float PDF(Vec3 wo, Vec3 wi, BxDF_SamplingFlags flags = BxDF_SamplingFlags::All) const override;

    virtual bool Sample_f(
        BSDFSample* sample, Vec3 wo, Float u0, Point2 u12, BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
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

    virtual Spectrum f(const Vec3& wo, const Vec3& wi) const override;
    virtual Float PDF(Vec3 wo, Vec3 wi, BxDF_SamplingFlags flags = BxDF_SamplingFlags::All) const override;

    virtual bool Sample_f(
        BSDFSample* sample, Vec3 wo, Float u0, Point2 u12, BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
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
        TrowbridgeReitzDistribution mf_clearcoat
    )
        : color{ color }
        , metallic{ metallic }
        , mf{ mf }
        , mf_clearcoat{ mf_clearcoat }
        , eta{ eta }
        , transmission{ transmission }
        , clearcoat{ clearcoat }
    {
    }

    virtual BxDF_Flags Flags() const override
    {
        return BxDF_Flags::Diffuse | BxDF_Flags::Glossy | BxDF_Flags::Reflection | BxDF_Flags::Transmission;
    }

    virtual Spectrum f(const Vec3& wo, const Vec3& wi) const override;
    virtual Float PDF(Vec3 wo, Vec3 wi, BxDF_SamplingFlags flags = BxDF_SamplingFlags::All) const override;

    virtual bool Sample_f(
        BSDFSample* sample, Vec3 wo, Float u0, Point2 u12, BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
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
    TrowbridgeReitzDistribution mf, mf_clearcoat;
    Float eta;
    Float transmission;
    Float clearcoat;
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

    virtual Spectrum f(const Vec3& wo, const Vec3& wi) const override;
    virtual Float PDF(Vec3 wo, Vec3 wi, BxDF_SamplingFlags flags = BxDF_SamplingFlags::All) const override;

    virtual bool Sample_f(
        BSDFSample* sample, Vec3 wo, Float u0, Point2 u12, BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
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
