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
    LambertianBxDF(const Spectrum& reflectance)
        : r{ reflectance }
    {
    }

    virtual BxDF_Flags Flags() const override
    {
        return !r.IsBlack() ? BxDF_Flags::DiffuseReflection : BxDF_Flags::Unset;
    }

    virtual Spectrum f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const override;
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

    virtual Spectrum f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const override;
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
    DielectricBxDF(Float eta, TrowbridgeReitzDistribution mf, Spectrum r)
        : eta{ eta }
        , mf{ mf }
        , r{ Sqrt(r) }
    {
    }

    virtual BxDF_Flags Flags() const override
    {
        BxDF_Flags flags = (eta == 1) ? BxDF_Flags::Transmission : (BxDF_Flags::Reflection | BxDF_Flags::Transmission);
        return flags | (mf.EffectivelySmooth() ? BxDF_Flags::Specular : BxDF_Flags::Glossy);
    }

    virtual Spectrum f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const override;
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

    static void ComputeReflectanceTexture(int32 texture_size, std::span<Float> uc, std::span<Point2> u);

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
    static inline std::unique_ptr<FloatImageTexture3D> E_texture = nullptr;
    static inline std::unique_ptr<FloatImageTexture3D> E_inv_texture = nullptr;

    Float eta;
    TrowbridgeReitzDistribution mf;
    Spectrum r;
};

class DielectricMultiScatteringBxDF : public BxDF
{
public:
    DielectricMultiScatteringBxDF(Float eta, TrowbridgeReitzDistribution mf, Spectrum r)
        : eta{ eta }
        , mf{ mf }
        , r{ Sqrt(r) }
    {
    }

    virtual BxDF_Flags Flags() const override
    {
        BxDF_Flags flags = (eta == 1) ? BxDF_Flags::Transmission : (BxDF_Flags::Reflection | BxDF_Flags::Transmission);
        return flags | (mf.EffectivelySmooth() ? BxDF_Flags::Specular : BxDF_Flags::Glossy);
    }

    virtual Spectrum f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const override;
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

    static void ComputeReflectanceTexture(int32 texture_size, std::span<Float> uc, std::span<Point2> u);

private:
    static inline std::unique_ptr<FloatImageTexture3D> E_texture = nullptr;
    static inline std::unique_ptr<FloatImageTexture3D> E_inv_texture = nullptr;
    static inline std::unique_ptr<FloatImageTexture> E_avg_texture = nullptr;
    static inline std::unique_ptr<FloatImageTexture> E_inv_avg_texture = nullptr;

    Float ComputeScatteringRatio(Float eta_o) const
    {
        Float eta_i = eta >= 1 ? eta : 1 / eta;
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

    Float eta;
    TrowbridgeReitzDistribution mf;
    Spectrum r;
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

    ConductorBxDF(Spectrum reflectance, TrowbridgeReitzDistribution mf)
        : mf{ mf }
        , eta{ 1 }
    {
        // The reflectance R for a conductor is:
        // R = \frac{(\eta - 1)^2 + k^2}{(\eta + 1)^2 + k^2}
        // Assume \eta = 1 and solve for k

        Spectrum r = Clamp(reflectance, 0, 1 - 1e-4f);
        k = 2 * Sqrt(r) / Sqrt(Max(Spectrum(1) - r, 0));
    }

    virtual BxDF_Flags Flags() const override
    {
        return mf.EffectivelySmooth() ? BxDF_Flags::SpecularReflection : BxDF_Flags::GlossyReflection;
    }

    virtual Spectrum f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const override;
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
};

class ConductorMultiScatteringBxDF : public BxDF
{
public:
    ConductorMultiScatteringBxDF(Spectrum eta, Spectrum k, TrowbridgeReitzDistribution mf)
        : mf{ mf }
        , eta{ eta }
        , k{ k }
    {
    }

    ConductorMultiScatteringBxDF(Spectrum reflectance, TrowbridgeReitzDistribution mf)
        : mf{ mf }
        , eta{ 1 }
    {
        // The reflectance R for a conductor is:
        // R = \frac{(\eta - 1)^2 + k^2}{(\eta + 1)^2 + k^2}
        // Assume \eta = 1 and solve for k

        Spectrum r = Clamp(reflectance, 0, 1 - 1e-4f);
        k = 2 * Sqrt(r) / Sqrt(Max(Spectrum(1) - r, 0));
    }

    virtual BxDF_Flags Flags() const override
    {
        return mf.EffectivelySmooth() ? BxDF_Flags::SpecularReflection : BxDF_Flags::GlossyReflection;
    }

    virtual Spectrum f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const override;
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
};

class ThinDielectricBxDF : public BxDF
{
public:
    ThinDielectricBxDF(Float eta, Spectrum r)
        : eta{ eta }
        , r{ Sqrt(r) }
    {
    }

    virtual BxDF_Flags Flags() const override
    {
        return BxDF_Flags::Specular | BxDF_Flags::Reflection | BxDF_Flags::Transmission;
    }

    virtual Spectrum f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const override;
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
    Spectrum r;
};

class SheenBxDF : public BxDF
{
public:
    SheenBxDF(Spectrum base, Spectrum sheen, CharlieSheenDistribution mf)
        : base{ base }
        , sheen{ sheen }
        , mf{ mf }
    {
    }

    virtual BxDF_Flags Flags() const override
    {
        return BxDF_Flags::Diffuse | BxDF_Flags::Reflection;
    }

    virtual Spectrum f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const override;
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
    Spectrum base, sheen;
    CharlieSheenDistribution mf;
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

    virtual Spectrum f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const override;
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

    virtual Spectrum f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const override;
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

    virtual Spectrum f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const override;
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
    // Energy-preserving Oren–Nayar BRDF
    // EON: APractical Energy-Preserving Rough Diffuse BRDF (Portsmouth et al., 2025)
    // https://jcgt.org/published/0014/01/06/
public:
    EONBxDF(const Spectrum& reflectance, Float roughness, bool exact = true)
        : rho{ reflectance }
        , r{ roughness }
        , exact{ exact }
    {
    }

    virtual BxDF_Flags Flags() const override
    {
        return !rho.IsBlack() ? BxDF_Flags::DiffuseReflection : BxDF_Flags::Unset;
    }

    virtual Spectrum f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const override;
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
    Spectrum rho;
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
        const Spectrum& albedo,
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

        if (IsDiffuse(top_flags) || IsDiffuse(bottom_flags) || albedo != Spectrum::black)
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

    virtual Spectrum f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const override;
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

    Spectrum albedo; // single scattering albedo
    Float thickness, g;

    int32 max_bounces, samples;
};

constexpr size_t max_bxdf_size = std::max(
    { sizeof(LambertianBxDF), sizeof(SpecularReflectionBxDF), sizeof(DielectricBxDF), sizeof(ConductorBxDF),
      sizeof(DielectricMultiScatteringBxDF), sizeof(ConductorMultiScatteringBxDF), sizeof(ThinDielectricBxDF), sizeof(SheenBxDF),
      sizeof(MetallicRoughnessBxDF), sizeof(PrincipledBxDF), sizeof(NormalizedFresnelBxDF), sizeof(EONBxDF), sizeof(LayeredBxDF) }
);

} // namespace bulbit
