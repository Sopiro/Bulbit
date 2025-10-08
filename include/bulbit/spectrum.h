#pragma once

#include "vectors.h"

namespace bulbit
{

using Spectrum = struct RGBSpectrum;

struct RGBSpectrum
{
    Float r, g, b;

    RGBSpectrum() = default;

    explicit constexpr RGBSpectrum(Float s)
        : r{ s }
        , g{ s }
        , b{ s }
    {
    }

    constexpr RGBSpectrum(Float red, Float green, Float blue)
        : r{ red }
        , g{ green }
        , b{ blue }
    {
    }

    constexpr RGBSpectrum(const Vec3& rgb)
        : r{ rgb.x }
        , g{ rgb.y }
        , b{ rgb.z }
    {
    }

    Float operator[](int32 i) const
    {
        return (&r)[i];
    }

    Float& operator[](int32 i)
    {
        return (&r)[i];
    }

    bool IsBlack() const
    {
        return r == 0.0f && g == 0.0f && b == 0.0f;
    }

    Float Luminance() const
    {
        constexpr RGBSpectrum coefficient(0.2126f, 0.7152f, 0.0722f);
        return r * coefficient.r + g * coefficient.g + b * coefficient.b;
    }

    Float Average() const
    {
#if 0
        Float sp = 0;
        for (int32 i = 0; i < num_spectral_samples; ++i)
        {
            sp += (*this)[i];
        }
        return sp / num_spectral_samples;
#else
        return (r + g + b) / 3;
#endif
    }

    Float MinComponent() const
    {
        return std::min({ r, g, b });
    }

    Float MaxComponent() const
    {
        return std::max({ r, g, b });
    }

    bool IsNullish() const
    {
        return std::isnan(r) || std::isinf(r) || std::isnan(g) || std::isinf(g) || std::isnan(b) || std::isinf(b);
    }

    std::string ToString() const
    {
        return std::format("{:.4f}\t{:.4f}\t{:.4f}", r, g, b);
    }

    static const int32 num_spectral_samples;
    static const RGBSpectrum black;
};

constexpr inline int32 RGBSpectrum::num_spectral_samples(3);
constexpr inline RGBSpectrum RGBSpectrum::black(0);

constexpr inline RGBSpectrum operator-(const RGBSpectrum& sp)
{
    return RGBSpectrum(-sp.r, -sp.g, -sp.b);
}

constexpr inline RGBSpectrum operator+(const RGBSpectrum& sp1, const RGBSpectrum& sp2)
{
    return RGBSpectrum(sp1.r + sp2.r, sp1.g + sp2.g, sp1.b + sp2.b);
}

constexpr inline RGBSpectrum operator-(const RGBSpectrum& sp1, const RGBSpectrum& sp2)
{
    return RGBSpectrum(sp1.r - sp2.r, sp1.g - sp2.g, sp1.b - sp2.b);
}

template <typename T>
constexpr inline RGBSpectrum operator*(const RGBSpectrum& sp, T s)
{
    return RGBSpectrum(sp.r * Float(s), sp.g * Float(s), sp.b * Float(s));
}

template <typename T>
constexpr inline RGBSpectrum operator*(T s, const RGBSpectrum& sp)
{
    return RGBSpectrum(sp.r * s, sp.g * s, sp.b * s);
}

constexpr inline RGBSpectrum operator*(const RGBSpectrum& sp1, const RGBSpectrum& sp2)
{
    return RGBSpectrum(sp1.r * sp2.r, sp1.g * sp2.g, sp1.b * sp2.b);
}

template <typename T>
constexpr inline RGBSpectrum operator/(const RGBSpectrum& sp, T s)
{
    return RGBSpectrum(sp.r / s, sp.g / s, sp.b / s);
}

template <typename T>
constexpr inline RGBSpectrum operator/(T s, const RGBSpectrum& sp)
{
    return RGBSpectrum(s / sp.r, s / sp.g, s / sp.b);
}

constexpr inline RGBSpectrum operator/(const RGBSpectrum& sp1, const RGBSpectrum& sp2)
{
    return RGBSpectrum(sp1.r / sp2.r, sp1.g / sp2.g, sp1.b / sp2.b);
}

constexpr inline bool operator==(const RGBSpectrum& sp1, const RGBSpectrum& sp2)
{
    return sp1.r == sp2.r && sp1.g == sp2.g && sp1.b == sp2.b;
}

constexpr inline bool operator!=(const RGBSpectrum& sp1, const RGBSpectrum& sp2)
{
    return sp1.r != sp2.r || sp1.g != sp2.g || sp1.b != sp2.b;
}

template <typename T>
constexpr inline RGBSpectrum operator+=(RGBSpectrum& sp1, T s)
{
    sp1.r += s;
    sp1.g += s;
    sp1.b += s;
    return sp1;
}

template <typename T>
constexpr inline RGBSpectrum operator-=(RGBSpectrum& sp1, T s)
{
    sp1.r -= s;
    sp1.g -= s;
    sp1.b -= s;
    return sp1;
}

template <typename T>
constexpr inline RGBSpectrum operator*=(RGBSpectrum& sp1, T s)
{
    sp1.r *= s;
    sp1.g *= s;
    sp1.b *= s;
    return sp1;
}

template <typename T>
constexpr inline RGBSpectrum operator/=(RGBSpectrum& sp1, T s)
{
    sp1.r /= s;
    sp1.g /= s;
    sp1.b /= s;
    return sp1;
}

constexpr inline RGBSpectrum operator+=(RGBSpectrum& sp1, const RGBSpectrum& sp2)
{
    sp1.r += sp2.r;
    sp1.g += sp2.g;
    sp1.b += sp2.b;
    return sp1;
}

constexpr inline RGBSpectrum operator-=(RGBSpectrum& sp1, const RGBSpectrum& sp2)
{
    sp1.r -= sp2.r;
    sp1.g -= sp2.g;
    sp1.b -= sp2.b;
    return sp1;
}

constexpr inline RGBSpectrum operator*=(RGBSpectrum& sp1, const RGBSpectrum& sp2)
{
    sp1.r *= sp2.r;
    sp1.g *= sp2.g;
    sp1.b *= sp2.b;
    return sp1;
}

constexpr inline RGBSpectrum operator/=(RGBSpectrum& sp1, const RGBSpectrum& sp2)
{
    sp1.r /= sp2.r;
    sp1.g /= sp2.g;
    sp1.b /= sp2.b;
    return sp1;
}

template <typename T>
constexpr inline RGBSpectrum Lerp(const RGBSpectrum& sp1, const RGBSpectrum& sp2, T t)
{
    return (T(1) - t) * sp1 + t * sp2;
}

inline RGBSpectrum Sqrt(const RGBSpectrum& sp)
{
    return RGBSpectrum(std::sqrt(sp.r), std::sqrt(sp.g), std::sqrt(sp.b));
}

inline RGBSpectrum Exp(const RGBSpectrum& sp)
{
    return RGBSpectrum(std::exp(sp.r), std::exp(sp.g), std::exp(sp.b));
}

template <typename T>
constexpr inline RGBSpectrum Min(const RGBSpectrum& sp, T val)
{
    return RGBSpectrum(std::min(Float(val), sp.r), std::min(Float(val), sp.g), std::min(Float(val), sp.b));
}

template <typename T>
constexpr inline RGBSpectrum Max(const RGBSpectrum& sp, T val)
{
    return RGBSpectrum(std::max(Float(val), sp.r), std::max(Float(val), sp.g), std::max(Float(val), sp.b));
}

template <typename U, typename V>
constexpr inline RGBSpectrum Clamp(const RGBSpectrum& sp, U left, V right)
{
    return RGBSpectrum(Clamp(sp.r, left, right), Clamp(sp.g, left, right), Clamp(sp.b, left, right));
}

namespace spectral
{

// Analytic fit of CIE XYZ curve proposed by Wyman et al.: https://jcgt.org/published/0002/02/01/
inline Float X_Fit_1931(Float wavelength)
{
    Float d_param_1 = (wavelength - Float(442.0)) * ((wavelength < Float(442.0)) ? Float(0.0624) : Float(0.0374));
    Float d_param_2 = (wavelength - Float(599.8)) * ((wavelength < Float(599.8)) ? Float(0.0264) : Float(0.0323));
    Float d_param_3 = (wavelength - Float(501.1)) * ((wavelength < Float(501.1)) ? Float(0.0490) : Float(0.0382));
    return Float(0.362) * std::exp(-Float(0.5) * d_param_1 * d_param_1) +
           Float(1.056) * std::exp(-Float(0.5) * d_param_2 * d_param_2) -
           Float(0.065) * std::exp(-Float(0.5) * d_param_3 * d_param_3);
}

inline Float Y_Fit_1931(Float wavelength)
{
    Float d_param_1 = (wavelength - Float(568.8)) * ((wavelength < Float(568.8)) ? Float(0.0213) : Float(0.0247));
    Float d_param_2 = (wavelength - Float(530.9)) * ((wavelength < Float(530.9)) ? Float(0.0613) : Float(0.0322));
    return Float(0.821) * std::exp(-Float(0.5) * d_param_1 * d_param_1) +
           Float(0.286) * std::exp(-Float(0.5) * d_param_2 * d_param_2);
}

inline Float Z_Fit_1931(Float wavelength)
{
    Float d_param_1 = (wavelength - Float(437.0)) * ((wavelength < Float(437.0)) ? Float(0.0845) : Float(0.0278));
    Float d_param_2 = (wavelength - Float(459.0)) * ((wavelength < Float(459.0)) ? Float(0.0385) : Float(0.0725));
    return Float(1.217) * std::exp(-Float(0.5) * d_param_1 * d_param_1) +
           Float(0.681) * std::exp(-Float(0.5) * d_param_2 * d_param_2);
}

inline Vec3 XYZ_integral_coeff(Float wavelength)
{
    return Vec3{ X_Fit_1931(wavelength), Y_Fit_1931(wavelength), Z_Fit_1931(wavelength) };
}

inline Vec3 XYZ_to_sRGB(const Vec3& xyz)
{
    // clang-format off
    Mat3 m(
        Vec3(3.2404542f, -0.9692660f, 0.0556434f),
        Vec3(-1.5371385f, 1.8760108f, -0.2040259f),
        Vec3(-0.4985314f, 0.0415560f, 1.0572252f)
    );
    // clang-format on

    return Mul(m, xyz);
}

constexpr inline Float Blackbody(Float lambda, Float T)
{
    if (T <= 0)
    {
        return 0;
    }

    constexpr Float h = 6.62606957e-34f; // Planck constant (J·s)
    constexpr Float c = 299792458.0f;    // Speed of light (m/s)
    constexpr Float kb = 1.3806488e-23f; // Boltzmann constant (J/K)

    Float l = lambda * 1e-9f;            // nm -> m
    Float exponent = (h * c) / (l * kb * T);

    if (exponent > 80.0f)
    {
        return 0;
    }

    Float Le = (2 * h * c * c) / (std::pow(l, Float(5)) * std::expm1(exponent));
    return Le;
}

inline Spectrum BlackbodyRGB(Float T, Float step = 10)
{
    constexpr Float wavelength_begin = 400.0f;
    constexpr Float wavelength_end = 700.0f;

    Vec3 XYZ(0);

    for (Float wavelength = wavelength_begin; wavelength <= wavelength_end; wavelength += step)
    {
        Float Le = Blackbody(wavelength, T);
        Vec3 coeff = XYZ_integral_coeff(wavelength);

        XYZ += coeff * Le;
    }

    // Convert to xyY(Y=1) and back to XYZ
    Float Y = XYZ[1];
    if (Y > 0)
    {
        XYZ /= Y;
    }

    // XYZ to linear sRGB
    Spectrum RGB = XYZ_to_sRGB(XYZ);
    return Max(RGB, 0);
}

} // namespace spectral

} // namespace bulbit
