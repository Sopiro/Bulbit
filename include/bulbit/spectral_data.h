#pragma once

#include "math.h"
#include "matrix.h"
#include "vectors.h"

namespace bulbit
{

struct WavelengthSample;
struct SpectrumSample;
class SpectralData;

namespace spectral
{

constexpr inline Float lambda_min = 380.0f;
constexpr inline Float lambda_max = 780.0f;
constexpr inline Float lambda_step = 5.0f;
constexpr inline int32 num_samples = 81;
constexpr inline int32 num_wavelength_samples = 4;
constexpr inline Float integral_scale = lambda_step / 10567.275056719916f;
constexpr inline Float monte_carlo_integral_scale = 1.0f / 10567.275056719916f;

struct InterpolationWeights
{
    int32 index;
    Float t;
};

Float Interpolate(const SpectralData& samples, Float wavelength);
Float Interpolate(const SpectralData& samples, const InterpolationWeights& weights);
SpectrumSample Sample(const SpectralData& data, const WavelengthSample& wavelengths);
Vec3 ToXYZ(const SpectrumSample& sp, const WavelengthSample& lambda);
Vec3 ToLinearRGB(const SpectrumSample& sp, const WavelengthSample& lambda);
Float Luminance(const SpectrumSample& sp, const WavelengthSample& lambda);

extern const SpectralData mallett_basis_r;
extern const SpectralData mallett_basis_g;
extern const SpectralData mallett_basis_b;
extern const SpectralData d65;
extern const SpectralData cie_x;
extern const SpectralData cie_y;
extern const SpectralData cie_z;

inline InterpolationWeights GetInterpolationWeights(Float wavelength)
{
    if (wavelength <= lambda_min)
    {
        return { 0, 0.0f };
    }
    else if (wavelength >= lambda_max)
    {
        return { num_samples - 2, 1.0f };
    }
    else
    {
        Float x = (wavelength - lambda_min) / lambda_step;
        int32 x0 = std::floor((wavelength - lambda_min) / lambda_step);
        int32 i = Clamp(x0, 0, num_samples - 2);
        Float t = Clamp(x - x0, 0.0f, 1.0f);
        return { i, t };
    }
}

inline Vec3 XYZToLinearRGB(const Vec3& xyz)
{
    // clang-format off
    constexpr Mat3 m(
        Vec3(3.2404542f, -0.9692660f, 0.0556434f),
        Vec3(-1.5371385f, 1.8760108f, -0.2040259f),
        Vec3(-0.4985314f, 0.0415560f, 1.0572252f)
    );
    // clang-format on
    return Mul(m, xyz);
}

inline Float EvaluateBlackbody(Float wavelength, Float temperature)
{
    if (temperature <= 0.0f)
    {
        return 0.0f;
    }

    constexpr Float h = 6.62606957e-34f;
    constexpr Float c = 299792458.0f;
    constexpr Float kb = 1.3806488e-23f;

    Float lambda = wavelength * 1e-9f;
    Float exponent = (h * c) / (lambda * kb * temperature);
    if (exponent > 80.0f)
    {
        return 0.0f;
    }

    return (2.0f * h * c * c) / (std::pow(lambda, 5.0f) * std::expm1(exponent));
}

} // namespace spectral

class SpectralData
{
public:
    std::array<Float, spectral::num_samples> samples{};

    SpectralData() = default;

    explicit SpectralData(const std::array<Float, spectral::num_samples>& values);
    explicit SpectralData(std::array<Float, spectral::num_samples>&& values);

    Float operator[](int32 i) const;
    Float& operator[](int32 i);

    static SpectralData Constant(Float value);
    static SpectralData CauchyIOR(Float cauchy_a, Float cauchy_b);

    SpectrumSample Sample(const WavelengthSample& wavelengths) const;

    bool IsBlack() const;
    bool IsNullish() const;
    Float Average() const;
    Float MinComponent() const;
    Float MaxComponent() const;
};

inline SpectralData operator-(const SpectralData& sp)
{
    SpectralData result;
    for (int32 i = 0; i < spectral::num_samples; ++i)
    {
        result[i] = -sp[i];
    }
    return result;
}

inline SpectralData operator+(const SpectralData& sp1, const SpectralData& sp2)
{
    SpectralData result;
    for (int32 i = 0; i < spectral::num_samples; ++i)
    {
        result[i] = sp1[i] + sp2[i];
    }
    return result;
}

inline SpectralData operator-(const SpectralData& sp1, const SpectralData& sp2)
{
    SpectralData result;
    for (int32 i = 0; i < spectral::num_samples; ++i)
    {
        result[i] = sp1[i] - sp2[i];
    }
    return result;
}

template <typename T>
requires(std::is_arithmetic_v<T>)
inline SpectralData operator*(const SpectralData& sp, T s)
{
    SpectralData result;
    for (int32 i = 0; i < spectral::num_samples; ++i)
    {
        result[i] = sp[i] * Float(s);
    }
    return result;
}

template <typename T>
requires(std::is_arithmetic_v<T>)
inline SpectralData operator*(T s, const SpectralData& sp)
{
    return sp * s;
}

inline SpectralData operator*(const SpectralData& sp1, const SpectralData& sp2)
{
    SpectralData result;
    for (int32 i = 0; i < spectral::num_samples; ++i)
    {
        result[i] = sp1[i] * sp2[i];
    }
    return result;
}

template <typename T>
requires(std::is_arithmetic_v<T>)
inline SpectralData operator/(const SpectralData& sp, T s)
{
    SpectralData result;
    for (int32 i = 0; i < spectral::num_samples; ++i)
    {
        result[i] = sp[i] / Float(s);
    }
    return result;
}

inline SpectralData operator/(const SpectralData& sp1, const SpectralData& sp2)
{
    SpectralData result;
    for (int32 i = 0; i < spectral::num_samples; ++i)
    {
        result[i] = sp1[i] / sp2[i];
    }
    return result;
}

template <typename T>
requires(std::is_arithmetic_v<T>)
inline SpectralData& operator*=(SpectralData& sp, T s)
{
    for (int32 i = 0; i < spectral::num_samples; ++i)
    {
        sp[i] *= Float(s);
    }
    return sp;
}

inline SpectralData& operator+=(SpectralData& sp1, const SpectralData& sp2)
{
    for (int32 i = 0; i < spectral::num_samples; ++i)
    {
        sp1[i] += sp2[i];
    }
    return sp1;
}

inline SpectralData& operator-=(SpectralData& sp1, const SpectralData& sp2)
{
    for (int32 i = 0; i < spectral::num_samples; ++i)
    {
        sp1[i] -= sp2[i];
    }
    return sp1;
}

inline SpectralData& operator*=(SpectralData& sp1, const SpectralData& sp2)
{
    for (int32 i = 0; i < spectral::num_samples; ++i)
    {
        sp1[i] *= sp2[i];
    }
    return sp1;
}

template <typename T>
requires(std::is_arithmetic_v<T>)
inline SpectralData& operator/=(SpectralData& sp, T s)
{
    for (int32 i = 0; i < spectral::num_samples; ++i)
    {
        sp[i] /= Float(s);
    }
    return sp;
}

inline SpectralData& operator/=(SpectralData& sp1, const SpectralData& sp2)
{
    for (int32 i = 0; i < spectral::num_samples; ++i)
    {
        sp1[i] /= sp2[i];
    }
    return sp1;
}

template <typename T>
requires(std::is_arithmetic_v<T>)
inline SpectralData Lerp(const SpectralData& sp1, const SpectralData& sp2, T t)
{
    return (T(1) - t) * sp1 + t * sp2;
}

inline SpectralData Sqrt(const SpectralData& sp)
{
    SpectralData result;
    for (int32 i = 0; i < spectral::num_samples; ++i)
    {
        result[i] = std::sqrt(sp[i]);
    }
    return result;
}

inline SpectralData Exp(const SpectralData& sp)
{
    SpectralData result;
    for (int32 i = 0; i < spectral::num_samples; ++i)
    {
        result[i] = std::exp(sp[i]);
    }
    return result;
}

template <typename T>
requires(std::is_arithmetic_v<T>)
inline SpectralData Min(const SpectralData& sp, T val)
{
    SpectralData result;
    for (int32 i = 0; i < spectral::num_samples; ++i)
    {
        result[i] = std::min(Float(val), sp[i]);
    }
    return result;
}

template <typename T>
requires(std::is_arithmetic_v<T>)
inline SpectralData Max(const SpectralData& sp, T val)
{
    SpectralData result;
    for (int32 i = 0; i < spectral::num_samples; ++i)
    {
        result[i] = std::max(Float(val), sp[i]);
    }
    return result;
}

template <typename U, typename V>
requires(std::is_arithmetic_v<U> && std::is_arithmetic_v<V>)
inline SpectralData Clamp(const SpectralData& sp, U left, V right)
{
    SpectralData result;
    for (int32 i = 0; i < spectral::num_samples; ++i)
    {
        result[i] = Clamp(sp[i], left, right);
    }
    return result;
}

} // namespace bulbit
