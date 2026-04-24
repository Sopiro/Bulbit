#pragma once

#include "spectral_data.h"

namespace bulbit
{

struct SpectrumSample;
struct WavelengthSample;

class Spectrum
{
public:
    enum class Representation : uint8_t
    {
        Constant,
        RGBReflectance,
        RGBIlluminant,
        Blackbody,
        Data,
    };

    Spectrum();
    explicit Spectrum(Float value);
    explicit Spectrum(const Vec3& rgb);
    Spectrum(Float red, Float green, Float blue);
    Spectrum(const Spectrum& other);

    Spectrum(Spectrum&&) noexcept = default;

    ~Spectrum() = default;

    Spectrum& operator=(const Spectrum& other);

    Spectrum& operator=(Spectrum&&) noexcept = default;

    static Spectrum Constant(Float value);
    static Spectrum FromRGB(const Vec3& rgb);
    static Spectrum FromIlluminantRGB(const Vec3& rgb);
    static Spectrum Blackbody(Float temperature, Float scale = 1.0f);
    static Spectrum FromData(const std::array<Float, spectral::num_samples>& sampled);
    static Spectrum FromData(const SpectralData& sampled);
    static Spectrum CauchyIOR(Float ior, Float dispersion);
    static Spectrum FromDataTriplet(const Vec3& v);

    SpectrumSample Sample(const WavelengthSample& wavelengths) const;
    std::array<Float, spectral::num_samples> Materialize() const;
    Vec3 ToLinearRGB() const;
    Float operator[](int32 i) const;
    bool IsBlack() const;
    bool IsNullish() const;
    Float Luminance() const;
    bool IsWavelengthConstant(Float epsilon = 1e-6f) const;
    Float Average() const;
    Float MinComponent() const;
    Float MaxComponent() const;
    bool Equals(const Spectrum& other) const;
    std::string ToString() const;

    static const int32 num_spectral_samples = 3;
    static const Spectrum black;

private:
    friend uint64_t HashSpectrum(const Spectrum& sp);

    static Float SampleRGBBasis(const Vec3& rgb, Float wavelength);
    static Float SampleIlluminantBasis(const Vec3& rgb, Float wavelength);
    static Float InterpolateProduct(const SpectralData& a, const SpectralData& b, Float wavelength);

    Float SampleRGBReflectance(Float wavelength) const;
    Float SampleRGBIlluminant(Float wavelength) const;
    Float SampleBlackbody(Float wavelength) const;
    Float ValueAtIndex(int32 i) const;

    explicit Spectrum(Representation representation, const Vec3& payload);
    explicit Spectrum(std::unique_ptr<SpectralData>&& data);

    Representation representation;
    Vec3 payload;
    std::unique_ptr<SpectralData> data;
};

uint64_t HashSpectrum(const Spectrum& sp);
bool operator==(const Spectrum& lhs, const Spectrum& rhs);
bool operator!=(const Spectrum& lhs, const Spectrum& rhs);

struct WavelengthSample
{
    static constexpr int32 num_lanes = spectral::num_wavelength_samples;
    static constexpr int32 hero_lane = 0;

    std::array<Float, num_lanes> lambda{};
    std::array<Float, num_lanes> pdf{};

    static WavelengthSample Sample(Float u_lambda)
    {
        constexpr Float domain = 400.0f;

        WavelengthSample sample;

        for (int32 i = 0; i < num_lanes; ++i)
        {
            Float t = std::fmod(u_lambda + Float(i - hero_lane) / Float(num_lanes) + 1.0f, 1.0f);
            sample.lambda[i] = spectral::lambda_min + domain * t;
            sample.pdf[i] = 1.0f / domain;
        }

        return sample;
    }

    void CollapseToPrimary()
    {
        if (IsCollapse())
        {
            return;
        }

        for (int32 i = 1; i < num_lanes; ++i)
        {
            pdf[i] = 0.0f;
        }

        pdf[hero_lane] /= Float(num_lanes);
    }

    bool IsCollapse() const
    {
        for (int32 i = 1; i < num_lanes; ++i)
        {
            if (pdf[i] != 0.0f)
            {
                return false;
            }
        }

        return true;
    }

    SpectrumSample PDF() const;
};

struct SpectrumSample
{
    static constexpr int32 num_lanes = WavelengthSample::num_lanes;

    std::array<Float, num_lanes> values{};

    SpectrumSample() = default;

    explicit constexpr SpectrumSample(Float value)
    {
        values.fill(value);
    }

    template <typename... Ts>
    requires(sizeof...(Ts) == num_lanes)
    constexpr SpectrumSample(Ts... lane_values)
        : values{ Float(lane_values)... }
    {
    }

    Float operator[](int32 i) const
    {
        return values[i];
    }

    Float& operator[](int32 i)
    {
        return values[i];
    }

    bool IsBlack() const
    {
        for (Float value : values)
        {
            if (value != 0.0f)
            {
                return false;
            }
        }

        return true;
    }

    bool IsNullish() const
    {
        for (Float value : values)
        {
            if (std::isnan(value) || std::isinf(value))
            {
                return true;
            }
        }

        return false;
    }

    Float Average() const
    {
        Float sum = 0;
        for (Float value : values)
        {
            sum += value;
        }
        return sum / Float(num_lanes);
    }

    Float MaxComponent() const
    {
        Float maximum = values[0];
        for (int32 i = 1; i < num_lanes; ++i)
        {
            maximum = std::max(maximum, values[i]);
        }
        return maximum;
    }

    Float MinComponent() const
    {
        Float minimum = values[0];
        for (int32 i = 1; i < num_lanes; ++i)
        {
            minimum = std::min(minimum, values[i]);
        }
        return minimum;
    }
};

constexpr inline SpectrumSample operator-(const SpectrumSample& sp)
{
    SpectrumSample result;
    for (int32 i = 0; i < SpectrumSample::num_lanes; ++i)
    {
        result[i] = -sp[i];
    }
    return result;
}

constexpr inline SpectrumSample operator+(const SpectrumSample& sp1, const SpectrumSample& sp2)
{
    SpectrumSample result;
    for (int32 i = 0; i < SpectrumSample::num_lanes; ++i)
    {
        result[i] = sp1[i] + sp2[i];
    }
    return result;
}

constexpr inline SpectrumSample operator-(const SpectrumSample& sp1, const SpectrumSample& sp2)
{
    SpectrumSample result;
    for (int32 i = 0; i < SpectrumSample::num_lanes; ++i)
    {
        result[i] = sp1[i] - sp2[i];
    }
    return result;
}

template <typename T>
requires(std::is_arithmetic_v<T>)
constexpr inline SpectrumSample operator*(const SpectrumSample& sp, T s)
{
    SpectrumSample result;
    for (int32 i = 0; i < SpectrumSample::num_lanes; ++i)
    {
        result[i] = sp[i] * Float(s);
    }
    return result;
}

template <typename T>
requires(std::is_arithmetic_v<T>)
constexpr inline SpectrumSample operator*(T s, const SpectrumSample& sp)
{
    return sp * s;
}

constexpr inline SpectrumSample operator*(const SpectrumSample& sp1, const SpectrumSample& sp2)
{
    SpectrumSample result;
    for (int32 i = 0; i < SpectrumSample::num_lanes; ++i)
    {
        result[i] = sp1[i] * sp2[i];
    }
    return result;
}

template <typename T>
requires(std::is_arithmetic_v<T>)
constexpr inline SpectrumSample operator/(const SpectrumSample& sp, T s)
{
    SpectrumSample result;
    for (int32 i = 0; i < SpectrumSample::num_lanes; ++i)
    {
        result[i] = sp[i] / Float(s);
    }
    return result;
}

constexpr inline SpectrumSample operator/(const SpectrumSample& sp1, const SpectrumSample& sp2)
{
    SpectrumSample result;
    for (int32 i = 0; i < SpectrumSample::num_lanes; ++i)
    {
        result[i] = sp1[i] / sp2[i];
    }
    return result;
}

constexpr inline SpectrumSample SafeDiv(const SpectrumSample& sp1, const SpectrumSample& sp2)
{
    SpectrumSample result;
    for (int32 i = 0; i < SpectrumSample::num_lanes; ++i)
    {
        result[i] = sp2[i] != 0.0f ? sp1[i] / sp2[i] : 0.0f;
    }
    return result;
}

template <typename T>
requires(std::is_arithmetic_v<T>)
constexpr inline SpectrumSample& operator+=(SpectrumSample& sp, T s)
{
    for (int32 i = 0; i < SpectrumSample::num_lanes; ++i)
    {
        sp[i] += Float(s);
    }
    return sp;
}

constexpr inline SpectrumSample& operator+=(SpectrumSample& sp1, const SpectrumSample& sp2)
{
    for (int32 i = 0; i < SpectrumSample::num_lanes; ++i)
    {
        sp1[i] += sp2[i];
    }
    return sp1;
}

template <typename T>
requires(std::is_arithmetic_v<T>)
constexpr inline SpectrumSample& operator-=(SpectrumSample& sp, T s)
{
    for (int32 i = 0; i < SpectrumSample::num_lanes; ++i)
    {
        sp[i] -= Float(s);
    }
    return sp;
}

constexpr inline SpectrumSample& operator-=(SpectrumSample& sp1, const SpectrumSample& sp2)
{
    for (int32 i = 0; i < SpectrumSample::num_lanes; ++i)
    {
        sp1[i] -= sp2[i];
    }
    return sp1;
}

template <typename T>
requires(std::is_arithmetic_v<T>)
constexpr inline SpectrumSample& operator*=(SpectrumSample& sp, T s)
{
    for (int32 i = 0; i < SpectrumSample::num_lanes; ++i)
    {
        sp[i] *= Float(s);
    }
    return sp;
}

constexpr inline SpectrumSample& operator*=(SpectrumSample& sp1, const SpectrumSample& sp2)
{
    for (int32 i = 0; i < SpectrumSample::num_lanes; ++i)
    {
        sp1[i] *= sp2[i];
    }
    return sp1;
}

template <typename T>
requires(std::is_arithmetic_v<T>)
constexpr inline SpectrumSample& operator/=(SpectrumSample& sp, T s)
{
    for (int32 i = 0; i < SpectrumSample::num_lanes; ++i)
    {
        sp[i] /= Float(s);
    }
    return sp;
}

constexpr inline SpectrumSample& operator/=(SpectrumSample& sp1, const SpectrumSample& sp2)
{
    for (int32 i = 0; i < SpectrumSample::num_lanes; ++i)
    {
        sp1[i] /= sp2[i];
    }
    return sp1;
}

template <typename T>
requires(std::is_arithmetic_v<T>)
constexpr inline SpectrumSample Lerp(const SpectrumSample& sp1, const SpectrumSample& sp2, T t)
{
    return (T(1) - t) * sp1 + t * sp2;
}

inline SpectrumSample Sqrt(const SpectrumSample& sp)
{
    SpectrumSample result;
    for (int32 i = 0; i < SpectrumSample::num_lanes; ++i)
    {
        result[i] = std::sqrt(sp[i]);
    }
    return result;
}

inline SpectrumSample Exp(const SpectrumSample& sp)
{
    SpectrumSample result;
    for (int32 i = 0; i < SpectrumSample::num_lanes; ++i)
    {
        result[i] = std::exp(sp[i]);
    }
    return result;
}

template <typename T>
requires(std::is_arithmetic_v<T>)
constexpr inline SpectrumSample Min(const SpectrumSample& sp, T val)
{
    SpectrumSample result;
    for (int32 i = 0; i < SpectrumSample::num_lanes; ++i)
    {
        result[i] = std::min(Float(val), sp[i]);
    }
    return result;
}

template <typename T>
requires(std::is_arithmetic_v<T>)
constexpr inline SpectrumSample Max(const SpectrumSample& sp, T val)
{
    SpectrumSample result;
    for (int32 i = 0; i < SpectrumSample::num_lanes; ++i)
    {
        result[i] = std::max(Float(val), sp[i]);
    }
    return result;
}

template <typename U, typename V>
requires(std::is_arithmetic_v<U> && std::is_arithmetic_v<V>)
constexpr inline SpectrumSample Clamp(const SpectrumSample& sp, U left, V right)
{
    SpectrumSample result;
    for (int32 i = 0; i < SpectrumSample::num_lanes; ++i)
    {
        result[i] = Clamp(sp[i], left, right);
    }
    return result;
}

inline Vec3 SpectrumSampleToLinearRGB(const SpectrumSample& sp, const WavelengthSample& lambda)
{
    return spectral::SpectrumSampleToLinearRGB(sp, lambda);
}

inline Float SpectrumSampleToLuminance(const SpectrumSample& sp, const WavelengthSample& lambda)
{
    return spectral::SpectrumSampleToLuminance(sp, lambda);
}

} // namespace bulbit
