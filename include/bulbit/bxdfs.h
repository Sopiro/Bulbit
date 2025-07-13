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

    Spectrum r;
    TrowbridgeReitzDistribution mf;
};

class DielectricMultiScatteringBxDF : public BxDF
{
public:
    DielectricMultiScatteringBxDF(Float eta, Spectrum r, TrowbridgeReitzDistribution mf)
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
    ThinDielectricBxDF(Float eta)
        : eta{ eta }
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

template <typename TopBxDF, typename BottomBxDF>
class VariantBxDF
{
public:
    VariantBxDF() = default;

    VariantBxDF& operator=(const TopBxDF* bxdf)
    {
        top = bxdf;
        bottom = nullptr;
        return *this;
    }
    VariantBxDF& operator=(const BottomBxDF* bxdf)
    {
        bottom = bxdf;
        top = nullptr;
        return *this;
    }

    Spectrum f(Vec3 wo, Vec3 wi, TransportDirection direction) const
    {
        return top ? top->f(wo, wi, direction) : bottom->f(wo, wi, direction);
    }

    bool Sample_f(
        BSDFSample* sample,
        Vec3 wo,
        Float u0,
        Point2 u12,
        TransportDirection direction,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const
    {
        return top ? top->Sample_f(sample, wo, u0, u12, direction, flags)
                   : bottom->Sample_f(sample, wo, u0, u12, direction, flags);
    }

    Float PDF(Vec3 wo, Vec3 wi, TransportDirection direction, BxDF_SamplingFlags flags = BxDF_SamplingFlags::All) const
    {
        return top ? top->PDF(wo, wi, direction, flags) : bottom->PDF(wo, wi, direction, flags);
    }

    BxDF_Flags Flags() const
    {
        return top ? top->Flags() : bottom->Flags();
    }

private:
    const TopBxDF* top = nullptr;
    const BottomBxDF* bottom = nullptr;
};

template <typename TopBxDF, typename BottomBxDF, bool two_sided>
class LayeredBxDF : public BxDF
{
public:
    LayeredBxDF(
        TopBxDF top,
        BottomBxDF bottom,
        const Spectrum& albedo,
        Float thickness,
        Float g = 0,
        int32 max_bounces = 16,
        int32 samples = 1
    )
        : top{ top }
        , bottom{ bottom }
        , albedo{ albedo }
        , thickness{ thickness }
        , g{ g }
        , max_bounces{ max_bounces }
        , samples{ samples }
    {
    }

    virtual BxDF_Flags Flags() const override
    {
        BxDF_Flags top_flags = top.Flags();
        BxDF_Flags bottom_flags = bottom.Flags();

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

    virtual Spectrum f(Vec3 wo, Vec3 wi, TransportDirection direction = TransportDirection::ToLight) const override
    {
        Spectrum f(0);

        if (two_sided && wo.z < 0)
        {
            wo.Negate();
            wi.Negate();
        }

        VariantBxDF<TopBxDF, BottomBxDF> enter_interface;

        bool entered_top = two_sided || wo.z > 0;
        if (entered_top)
        {
            enter_interface = &top;
        }
        else
        {
            enter_interface = &bottom;
        }

        VariantBxDF<TopBxDF, BottomBxDF> exit_interface, non_exit_interface;
        bool exit_bottom = SameHemisphere(wo, wi) ^ entered_top;
        if (exit_bottom)
        {
            exit_interface = &bottom;
            non_exit_interface = &top;
        }
        else
        {
            exit_interface = &top;
            non_exit_interface = &bottom;
        }

        Float z_exit = exit_bottom ? 0 : thickness;

        // Part of BSDF is given by reflection
        if (SameHemisphere(wo, wi))
        {
            f = samples * enter_interface.f(wo, wi, direction);
        }

        // Prepare rng for stochastic BSDF evaluation
        RNG rng(Hash(wo), Hash(wi));

        // Estimate BSDF by unidirectional random walk
        for (int32 s = 0; s < samples; ++s)
        {
            // Sample transmission direction through entrance interface conditioned on wo
            // This is the initial direction of random walk
            BSDFSample wo_sample;
            if (!enter_interface.Sample_f(
                    &wo_sample, wo, rng.NextFloat(), { rng.NextFloat(), rng.NextFloat() }, direction,
                    BxDF_SamplingFlags::Transmission
                ))
            {
                continue;
            }

            if (wo_sample.f == Spectrum::black || wo_sample.pdf == 0 || wo_sample.wi.z == 0)
            {
                continue;
            }

            // Sample transmission direction through entrance interface conditioned on wi
            // This is the virtual light direction used for NEE contribution
            BSDFSample wi_sample;
            if (!exit_interface.Sample_f(
                    &wi_sample, wi, rng.NextFloat(), { rng.NextFloat(), rng.NextFloat() }, !direction,
                    BxDF_SamplingFlags::Transmission
                ))
            {
                continue;
            }

            if (wi_sample.f == Spectrum::black || wi_sample.pdf == 0 || wi_sample.wi.z == 0)
            {
                continue;
            }

            // Path states for random walk BSDF evaluation
            Spectrum beta = wo_sample.f * AbsCosTheta(wo_sample.wi) / wo_sample.pdf;
            Float z = entered_top ? thickness : 0;
            Vec3 w = wo_sample.wi;
            HenyeyGreensteinPhaseFunction phase_function(g);

            constexpr Float rr_min = 0.25f;
            for (int32 bounce = 0; bounce < max_bounces; ++bounce)
            {
                // Possibly terminate random walk with russian roulette
                if (bounce > 3)
                {
                    if (Float p = beta.MaxComponent(); p < rr_min)
                    {
                        if (rng.NextFloat() > p)
                        {
                            break;
                        }
                        else
                        {
                            beta /= p;
                        }
                    }
                }

                if (albedo == Spectrum::black)
                {
                    // No medium scattering, advance to next layer boundary
                    z = (z == thickness) ? 0 : thickness;

                    // Update beta for transmittance
                    beta *= Tr(thickness, w);
                }
                else
                {
                    constexpr Float sigma_t = 1;
                    Float dz = SampleExponential(rng.NextFloat(), sigma_t / std::abs(w.z));

                    // Sampled position by homogenious medium scattering
                    Float z_p = w.z > 0 ? (z + dz) : (z - dz);

                    // It's still inside the medium
                    if (0 < z_p && z_p < thickness)
                    {
                        // Add MIS combined NEE contiribution from virtual light through exit interface
                        Float w_mis = 1;
                        if (!IsSpecular(exit_interface.Flags()))
                        {
                            w_mis = PowerHeuristic(wi_sample.pdf, phase_function.PDF(-w, -wi_sample.wi));
                        }

                        f += beta * w_mis * albedo * phase_function.p(-w, -wi_sample.wi) * Tr(z_p - z_exit, wi_sample.wi) *
                             wi_sample.f / wi_sample.pdf;

                        // Sample phase function for next path vertex
                        PhaseFunctionSample phase_sample;
                        if (!phase_function.Sample_p(&phase_sample, -w, { rng.NextFloat(), rng.NextFloat() }))
                        {
                            continue;
                        }

                        if (phase_sample.pdf == 0 || phase_sample.wi.z == 0)
                        {
                            continue;
                        }

                        beta *= albedo * phase_sample.p / phase_sample.pdf;
                        w = phase_sample.wi;
                        z = z_p;

                        // Add MIS combined phase function contribution
                        if (!IsSpecular(exit_interface.Flags()))
                        {
                            if ((w.z > 0 && z < z_exit) || (w.z < 0 && z > z_exit))
                            {
                                // Incorporate contribution from exit interface
                                Spectrum f_exit = exit_interface.f(-w, wi, direction);
                                if (f_exit != Spectrum::black)
                                {
                                    Float pdf_exit = exit_interface.PDF(-w, wi, direction, BxDF_SamplingFlags::Transmission);
                                    Float w_mis = PowerHeuristic(phase_sample.pdf, pdf_exit);
                                    f += beta * w_mis * Tr(z_p - z_exit, phase_sample.wi) * f_exit;
                                }
                            }
                        }

                        // Proceed to sample next path vertex
                        continue;
                    }

                    // Sampled surface scattering, L_o
                    z = Clamp(z_p, 0, thickness);
                }

                // Handle interface sampling
                if (z == z_exit)
                {
                    // No light contribution is added since we are doing NEE
                    BSDFSample exit_sample;
                    if (!exit_interface.Sample_f(
                            &exit_sample, -w, rng.NextFloat(), { rng.NextFloat(), rng.NextFloat() }, direction,
                            BxDF_SamplingFlags::Reflection
                        ))
                    {
                        break;
                    }

                    if (exit_sample.f == Spectrum::black || exit_sample.pdf == 0 || exit_sample.wi.z == 0)
                    {
                        break;
                    }

                    beta *= exit_sample.f * AbsCosTheta(exit_sample.wi) / exit_sample.pdf;
                    w = exit_sample.wi;
                }
                else
                {
                    if (!IsSpecular(non_exit_interface.Flags()))
                    {
                        // Add NEE contribution
                        Float w_mis = 1;
                        if (!IsSpecular((exit_interface.Flags())))
                        {
                            w_mis = PowerHeuristic(wi_sample.pdf, non_exit_interface.PDF(-w, -wi_sample.wi, direction));
                        }

                        f += beta * w_mis * non_exit_interface.f(-w, -wi_sample.wi, direction) * AbsCosTheta(wi_sample.wi) *
                             Tr(thickness, wi_sample.wi) * wi_sample.f / wi_sample.pdf;
                    }

                    // Sample next reflection direction
                    BSDFSample ref_sample;
                    if (!non_exit_interface.Sample_f(
                            &ref_sample, -w, rng.NextFloat(), { rng.NextFloat(), rng.NextFloat() }, direction,
                            BxDF_SamplingFlags::Reflection
                        ))
                    {
                        break;
                    }

                    if (ref_sample.f == Spectrum::black || ref_sample.pdf == 0 || ref_sample.wi.z == 0)
                    {
                        break;
                    }

                    // Update path state
                    beta *= ref_sample.f * AbsCosTheta(ref_sample.wi) / ref_sample.pdf;
                    w = ref_sample.wi;

                    if (!IsSpecular(exit_interface.Flags()))
                    {
                        // Add light contribution from BSDF sampling
                        Spectrum f_exit = exit_interface.f(-w, wi, direction);
                        if (f_exit != Spectrum::black)
                        {
                            Float w_mis = 1;
                            if (!IsSpecular(non_exit_interface.Flags()))
                            {
                                Float pdf_exit = exit_interface.PDF(-w, wi, direction, BxDF_SamplingFlags::Transmission);
                                w_mis = PowerHeuristic(ref_sample.pdf, pdf_exit);
                            }

                            f += beta * w_mis * Tr(thickness, ref_sample.wi) * f_exit;
                        }
                    }
                }
            }
        }

        return f / samples;
    }

    virtual Float PDF(
        Vec3 wo,
        Vec3 wi,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override
    {
        Float pdf = 0;

        if (two_sided && wo.z < 0)
        {
            wo.Negate();
            wi.Negate();
        }

        bool entered_top = two_sided || wo.z > 0;
        bool reflection = SameHemisphere(wo, wi);

        if (reflection && (flags & BxDF_SamplingFlags::Reflection))
        {
            // Add the first R pdf
            pdf += samples * (entered_top ? top.PDF(wo, wi, direction, BxDF_SamplingFlags::Reflection)
                                          : bottom.PDF(wo, wi, direction, BxDF_SamplingFlags::Reflection));
        }

        RNG rng(Hash(wi), Hash(wo));

        for (int32 s = 0; s < samples; ++s)
        {
            if (reflection && (flags & BxDF_SamplingFlags::Reflection))
            {
                // Estimate the first TRT pdf
                VariantBxDF<TopBxDF, BottomBxDF> r_interface, t_interface;
                if (entered_top)
                {
                    r_interface = &bottom;
                    t_interface = &top;
                }
                else
                {
                    r_interface = &top;
                    t_interface = &bottom;
                }

                BSDFSample wo_sample, wi_sample;
                if (t_interface.Sample_f(
                        &wo_sample, wo, rng.NextFloat(), { rng.NextFloat(), rng.NextFloat() }, direction,
                        BxDF_SamplingFlags::Transmission
                    ) &&
                    t_interface.Sample_f(
                        &wi_sample, wi, rng.NextFloat(), { rng.NextFloat(), rng.NextFloat() }, !direction,
                        BxDF_SamplingFlags::Transmission
                    ))
                {
                    if (wo_sample.f == Spectrum::black || wo_sample.pdf == 0)
                    {
                        continue;
                    }

                    if (wi_sample.f == Spectrum::black || wi_sample.pdf == 0)
                    {
                        continue;
                    }

                    if (IsSpecular(t_interface.Flags()))
                    {
                        pdf += r_interface.PDF(-wo_sample.wi, -wi_sample.wi, direction);
                    }
                    else
                    {
                        BSDFSample r_sample;
                        if (r_interface.Sample_f(
                                &r_sample, -wo_sample.wi, rng.NextFloat(), { rng.NextFloat(), rng.NextFloat() }, direction
                            ))
                        {
                            if (r_sample.f == Spectrum::black || r_sample.pdf == 0)
                            {
                                continue;
                            }

                            if (IsSpecular(r_interface.Flags()))
                            {
                                pdf += t_interface.PDF(-r_sample.wi, wi, direction);
                            }
                            else
                            {
                                Float pdf_r = r_interface.PDF(-wo_sample.wi, -wi_sample.wi, direction);
                                Float w_mis = PowerHeuristic(wi_sample.pdf, pdf_r);
                                pdf += w_mis * pdf_r;

                                Float pdf_t = t_interface.PDF(-r_sample.wi, wi, direction);
                                w_mis = PowerHeuristic(r_sample.pdf, pdf_t);
                                pdf += w_mis * pdf_t;
                            }
                        }
                    }
                }
            }
            else if (!reflection && (flags & BxDF_SamplingFlags::Transmission))
            {
                // Estimate the first TT pdf
                VariantBxDF<TopBxDF, BottomBxDF> to_interface, ti_interface;
                if (entered_top)
                {
                    to_interface = &top;
                    ti_interface = &bottom;
                }
                else
                {
                    to_interface = &bottom;
                    ti_interface = &top;
                }

                BSDFSample wo_sample;
                if (!to_interface.Sample_f(&wo_sample, wo, rng.NextFloat(), { rng.NextFloat(), rng.NextFloat() }, direction))
                {
                    continue;
                }

                if (wo_sample.f == Spectrum::black || wo_sample.pdf == 0 || wo_sample.wi.z == 0 || wo_sample.IsReflection())
                {
                    continue;
                }

                BSDFSample wi_sample;
                if (!ti_interface.Sample_f(&wi_sample, wi, rng.NextFloat(), { rng.NextFloat(), rng.NextFloat() }, !direction))
                {
                    continue;
                }

                if (wi_sample.f == Spectrum::black || wi_sample.pdf == 0 || wi_sample.wi.z == 0 || wi_sample.IsReflection())
                {
                    continue;
                }

                if (IsSpecular(to_interface.Flags()))
                {
                    pdf += ti_interface.PDF(-wo_sample.wi, wi, direction);
                }
                else if (IsSpecular(ti_interface.Flags()))
                {
                    pdf += to_interface.PDF(wo, -wi_sample.wi, direction);
                }
                else
                {
                    // Combine two sampling strategy with constant weights
                    Float pdf_ti = ti_interface.PDF(-wo_sample.wi, wi, direction);
                    Float pdf_to = to_interface.PDF(wo, -wi_sample.wi, direction);

                    pdf += 0.5f * (pdf_to + pdf_ti);
                }
            }
        }

        // Return mixture of two pdfs, uniform sphere pdf approximates diffused multiple scattering
        return Lerp(0.9f, UniformSpherePDF(), pdf / samples);
    }

    virtual bool Sample_f(
        BSDFSample* sample,
        Vec3 wo,
        Float u0,
        Point2 u12,
        TransportDirection direction = TransportDirection::ToLight,
        BxDF_SamplingFlags flags = BxDF_SamplingFlags::All
    ) const override
    {
        bool flipped = false;
        if (two_sided && wo.z < 0)
        {
            wo.Negate();
            flipped = true;
        }

        // Sample BSDF at entrance interface to get inital direction
        bool entered_top = two_sided || wo.z > 0;

        bool sampled = false;
        BSDFSample wo_sample;
        if (entered_top)
        {
            sampled = top.Sample_f(&wo_sample, wo, u0, u12, direction);
        }
        else
        {
            sampled = bottom.Sample_f(&wo_sample, wo, u0, u12, direction);
        }

        if (!sampled || wo_sample.f == Spectrum::black || wo_sample.pdf == 0 || wo_sample.wi.z == 0)
        {
            return false;
        }

        if (wo_sample.IsReflection())
        {
            wo_sample.is_stochastic = true;

            if (flipped)
            {
                wo_sample.wi.Negate();
            }

            if (!(flags & BxDF_SamplingFlags::Reflection))
            {
                return false;
            }

            *sample = wo_sample;
            return true;
        }

        // Initial unidirectional random walk direction
        Vec3 w = wo_sample.wi;
        bool was_specular = wo_sample.IsSpecular();

        RNG rng(Hash(wo), Hash(u0, u12));

        // Path states intialized with initial w sample
        Spectrum f = wo_sample.f * AbsCosTheta(wo_sample.wi);
        Float pdf = wo_sample.pdf;

        Float z = entered_top ? thickness : 0;
        HenyeyGreensteinPhaseFunction phase_function(g);

        constexpr Float rr_min = 0.25f;
        for (int32 bounce = 0; bounce < max_bounces; ++bounce)
        {
            // Start random walk

            // Possibly terminate random walk with russian roulette
            if (bounce > 3)
            {
                if (Float beta = f.MaxComponent() / pdf; beta < rr_min)
                {
                    if (rng.NextFloat() > beta)
                    {
                        return false;
                    }
                    else
                    {
                        pdf /= beta;
                    }
                }
            }

            if (w.z == 0)
            {
                return false;
            }

            if (albedo == Spectrum::black)
            {
                // No medium scattering, advance to next layer boundary
                z = (z == thickness) ? 0 : thickness;

                // Update beta for transmittance
                f *= Tr(thickness, w);
            }
            else
            {
                // Sample scattering event and update path states
                constexpr Float sigma_t = 1;
                Float dz = SampleExponential(rng.NextFloat(), sigma_t / std::abs(w.z));
                Float z_p = w.z > 0 ? (z + dz) : (z - dz);

                // Sampled medium scattering
                if (0 < z_p && z_p < thickness)
                {
                    // Sample phase function for next path vertex
                    PhaseFunctionSample phase_sample;
                    if (!phase_function.Sample_p(&phase_sample, -w, { rng.NextFloat(), rng.NextFloat() }))
                    {
                        return false;
                    }

                    // Update path states for volume scattering vertex
                    f *= albedo * phase_sample.p;
                    pdf *= phase_sample.pdf;
                    was_specular = false;

                    w = phase_sample.wi;
                    z = z_p;

                    // Proceed to sample next path vertex
                    continue;
                }
                else // Sampled surface scattering
                {
                    z = Clamp(z_p, 0, thickness);
                }
            }

            VariantBxDF<TopBxDF, BottomBxDF> interface;
            if (z == 0)
            {
                interface = &bottom;
            }
            else
            {
                interface = &top;
            }

            BSDFSample bsdf_sample;
            if (!interface.Sample_f(&bsdf_sample, -w, rng.NextFloat(), { rng.NextFloat(), rng.NextFloat() }, direction))
            {
                return false;
            }

            if (bsdf_sample.f == Spectrum::black || bsdf_sample.pdf == 0 || bsdf_sample.wi.z == 0)
            {
                return false;
            }

            // Update path states for surface scattering vertex
            f *= bsdf_sample.f;
            pdf *= bsdf_sample.pdf;
            was_specular &= bsdf_sample.IsSpecular();
            w = bsdf_sample.wi;

            // Stop random walk and return sample if path has left the layers
            if (bsdf_sample.IsTransmission())
            {
                BxDF_Flags flags;
                flags = SameHemisphere(wo, w) ? BxDF_Flags::Reflection : BxDF_Flags::Transmission;
                flags |= was_specular ? BxDF_Flags::Specular : BxDF_Flags::Glossy;

                if (flipped)
                {
                    w.Negate();
                }

                *sample = BSDFSample(f, w, pdf, flags, 1, true);
                return true;
            }

            // Don't forget the cosine term on surface scattering
            f *= AbsCosTheta(bsdf_sample.wi);
        }

        return false;
    }

    virtual void Regularize() override
    {
        top.Regularize();
        bottom.Regularize();
    }

private:
    static Float Tr(Float dz, Vec3 w)
    {
        return std::exp(-std::abs(dz / w.z));
    }

    TopBxDF top;
    BottomBxDF bottom;

    Spectrum albedo; // single scattering albedo
    Float thickness, g;

    int32 max_bounces, samples;
};

constexpr size_t max_bxdf_size = std::max(
    { sizeof(LambertianBxDF), sizeof(SpecularReflectionBxDF), sizeof(DielectricBxDF), sizeof(ConductorBxDF),
      sizeof(DielectricMultiScatteringBxDF), sizeof(ConductorMultiScatteringBxDF), sizeof(ThinDielectricBxDF), sizeof(SheenBxDF),
      sizeof(MetallicRoughnessBxDF), sizeof(PrincipledBxDF), sizeof(NormalizedFresnelBxDF), sizeof(EONBxDF) }
);

} // namespace bulbit
