#include "bulbit/spectrum.h"
#include "bulbit/hash.h"

namespace bulbit
{

Spectrum::Spectrum()
    : Spectrum(Representation::Constant, Vec3(0.0f))
{
}

Spectrum::Spectrum(Float value)
    : Spectrum(Representation::Constant, Vec3(value, 0.0f, 0.0f))
{
}

Spectrum::Spectrum(const Vec3& rgb)
    : Spectrum(FromRGB(rgb))
{
}

Spectrum::Spectrum(Float red, Float green, Float blue)
    : Spectrum(FromRGB(Vec3(red, green, blue)))
{
}

Spectrum::Spectrum(const Spectrum& other)
    : representation{ other.representation }
    , payload{ other.payload }
    , data{ other.data ? std::make_unique<SpectralData>(*other.data) : nullptr }
{
}

Spectrum& Spectrum::operator=(const Spectrum& other)
{
    if (this == &other)
    {
        return *this;
    }

    representation = other.representation;
    payload = other.payload;
    data = other.data ? std::make_unique<SpectralData>(*other.data) : nullptr;
    return *this;
}

Spectrum Spectrum::Constant(Float value)
{
    return Spectrum(value);
}

Spectrum Spectrum::FromRGB(const Vec3& rgb)
{
    return Spectrum(Representation::RGBReflectance, rgb);
}

Spectrum Spectrum::FromIlluminantRGB(const Vec3& rgb)
{
    return Spectrum(Representation::RGBIlluminant, rgb);
}

Spectrum Spectrum::Blackbody(Float temperature, Float scale)
{
    Float peak = 0.0f;
    for (int32 i = 0; i < spectral::num_samples; ++i)
    {
        Float wavelength = spectral::lambda_min + i * spectral::lambda_step;
        peak = std::max(peak, spectral::EvaluateBlackbody(wavelength, temperature));
    }

    return Spectrum(Representation::Blackbody, Vec3(temperature, scale, peak > 0.0f ? 1.0f / peak : 0.0f));
}

Spectrum Spectrum::FromData(const std::array<Float, spectral::num_samples>& sampled)
{
    return Spectrum(std::make_unique<SpectralData>(sampled));
}

Spectrum Spectrum::FromData(const SpectralData& sampled)
{
    return Spectrum(std::make_unique<SpectralData>(sampled));
}

Spectrum Spectrum::CauchyIOR(Float ior, Float dispersion)
{
    return FromData(SpectralData::CauchyIOR(ior, dispersion));
}

Spectrum Spectrum::FromDataTriplet(const Vec3& v)
{
    std::array<Float, spectral::num_samples> sampled{};

    constexpr Float lambda_b = 460.0f;
    constexpr Float lambda_g = 550.0f;
    constexpr Float lambda_r = 640.0f;

    for (int32 i = 0; i < spectral::num_samples; ++i)
    {
        Float lambda = spectral::lambda_min + spectral::lambda_step * i;
        if (lambda <= lambda_b)
        {
            sampled[i] = v.z;
        }
        else if (lambda <= lambda_g)
        {
            Float t = (lambda - lambda_b) / (lambda_g - lambda_b);
            sampled[i] = Lerp(v.z, v.y, t);
        }
        else if (lambda <= lambda_r)
        {
            Float t = (lambda - lambda_g) / (lambda_r - lambda_g);
            sampled[i] = Lerp(v.y, v.x, t);
        }
        else
        {
            sampled[i] = v.x;
        }
    }

    return FromData(sampled);
}

SpectrumSample Spectrum::Sample(const WavelengthSample& wavelengths) const
{
    if (representation == Representation::Data)
    {
        return data ? data->Sample(wavelengths) : SpectrumSample(0.0f);
    }

    SpectrumSample result;
    for (int32 i = 0; i < WavelengthSample::num_lanes; ++i)
    {
        switch (representation)
        {
        case Representation::Constant:
            result[i] = payload.x;
            break;
        case Representation::RGBReflectance:
            result[i] = SampleRGBReflectance(wavelengths.lambda[i]);
            break;
        case Representation::RGBIlluminant:
            result[i] = SampleRGBIlluminant(wavelengths.lambda[i]);
            break;
        case Representation::Blackbody:
            result[i] = SampleBlackbody(wavelengths.lambda[i]);
            break;
        case Representation::Data:
            break;
        }
    }
    return result;
}

std::array<Float, spectral::num_samples> Spectrum::Materialize() const
{
    std::array<Float, spectral::num_samples> values{};
    for (int32 i = 0; i < spectral::num_samples; ++i)
    {
        values[i] = ValueAtIndex(i);
    }
    return values;
}

Vec3 Spectrum::ToLinearRGB() const
{
    Vec3 xyz(0);
    for (int32 i = 0; i < spectral::num_samples; ++i)
    {
        Float coeff = spectral::integral_scale * ValueAtIndex(i);
        xyz += coeff * Vec3(spectral::cie_x[i], spectral::cie_y[i], spectral::cie_z[i]);
    }
    return Max(spectral::XYZToLinearRGB(xyz), Vec3(0.0f));
}

Float Spectrum::operator[](int32 i) const
{
    return ToLinearRGB()[i];
}

bool Spectrum::IsBlack() const
{
    switch (representation)
    {
    case Representation::Constant:
        return payload.x == 0.0f;
    case Representation::RGBReflectance:
    case Representation::RGBIlluminant:
        return payload == Vec3(0.0f);
    case Representation::Blackbody:
        return payload.y == 0.0f || payload.z == 0.0f;
    case Representation::Data:
        return !data || data->IsBlack();
    }

    return true;
}

bool Spectrum::IsNullish() const
{
    if (representation == Representation::Data)
    {
        return data == nullptr || data->IsNullish();
    }

    for (int32 i = 0; i < 3; ++i)
    {
        if (std::isnan(payload[i]) || std::isinf(payload[i]))
        {
            return true;
        }
    }

    return false;
}

Float Spectrum::Luminance() const
{
    Vec3 linear_rgb = ToLinearRGB();
    return Dot(linear_rgb, Vec3(0.2126f, 0.7152f, 0.0722f));
}

bool Spectrum::IsWavelengthConstant(Float epsilon) const
{
    Float reference = ValueAtIndex(0);
    for (int32 i = 1; i < spectral::num_samples; ++i)
    {
        if (std::abs(ValueAtIndex(i) - reference) > epsilon)
        {
            return false;
        }
    }

    return true;
}

Float Spectrum::Average() const
{
    Float sum = 0.0f;
    for (int32 i = 0; i < spectral::num_samples; ++i)
    {
        sum += ValueAtIndex(i);
    }
    return sum / Float(spectral::num_samples);
}

Float Spectrum::MinComponent() const
{
    Float minimum = ValueAtIndex(0);
    for (int32 i = 1; i < spectral::num_samples; ++i)
    {
        minimum = std::min(minimum, ValueAtIndex(i));
    }
    return minimum;
}

Float Spectrum::MaxComponent() const
{
    Float maximum = ValueAtIndex(0);
    for (int32 i = 1; i < spectral::num_samples; ++i)
    {
        maximum = std::max(maximum, ValueAtIndex(i));
    }
    return maximum;
}

bool Spectrum::Equals(const Spectrum& other) const
{
    if (representation != other.representation)
    {
        return false;
    }

    if (representation != Representation::Data)
    {
        return payload == other.payload;
    }

    if (data == nullptr || other.data == nullptr)
    {
        return data == other.data;
    }

    return data->samples == other.data->samples;
}

std::string Spectrum::ToString() const
{
    Vec3 rgb = ToLinearRGB();
    return std::format("{:.4f}\t{:.4f}\t{:.4f}", rgb.x, rgb.y, rgb.z);
}

Float Spectrum::SampleRGBBasis(const Vec3& rgb, Float wavelength)
{
    spectral::InterpolationWeights weights = spectral::GetInterpolationWeights(wavelength);
    return rgb.x * spectral::Interpolate(spectral::mallett_basis_r, weights) +
           rgb.y * spectral::Interpolate(spectral::mallett_basis_g, weights) +
           rgb.z * spectral::Interpolate(spectral::mallett_basis_b, weights);
}

Float Spectrum::SampleIlluminantBasis(const Vec3& rgb, Float wavelength)
{
    return rgb.x * InterpolateProduct(spectral::d65, spectral::mallett_basis_r, wavelength) +
           rgb.y * InterpolateProduct(spectral::d65, spectral::mallett_basis_g, wavelength) +
           rgb.z * InterpolateProduct(spectral::d65, spectral::mallett_basis_b, wavelength);
}

Float Spectrum::InterpolateProduct(const SpectralData& a, const SpectralData& b, Float wavelength)
{
    spectral::InterpolationWeights weights = spectral::GetInterpolationWeights(wavelength);
    int32 i = weights.index;
    return Lerp(a[i] * b[i], a[i + 1] * b[i + 1], weights.t);
}

Float Spectrum::SampleRGBReflectance(Float wavelength) const
{
    return SampleRGBBasis(payload, wavelength);
}

Float Spectrum::SampleRGBIlluminant(Float wavelength) const
{
    return SampleIlluminantBasis(payload, wavelength);
}

Float Spectrum::SampleBlackbody(Float wavelength) const
{
    return payload.y * payload.z * spectral::EvaluateBlackbody(wavelength, payload.x);
}

Float Spectrum::ValueAtIndex(int32 i) const
{
    switch (representation)
    {
    case Representation::Constant:
        return payload.x;
    case Representation::RGBReflectance:
        return payload.x * spectral::mallett_basis_r[i] + payload.y * spectral::mallett_basis_g[i] +
               payload.z * spectral::mallett_basis_b[i];
    case Representation::RGBIlluminant:
        return payload.x * spectral::d65[i] * spectral::mallett_basis_r[i] +
               payload.y * spectral::d65[i] * spectral::mallett_basis_g[i] +
               payload.z * spectral::d65[i] * spectral::mallett_basis_b[i];
    case Representation::Blackbody:
        return SampleBlackbody(spectral::lambda_min + spectral::lambda_step * i);
    case Representation::Data:
        return data ? data->samples[i] : 0.0f;
    }

    return 0.0f;
}

Spectrum::Spectrum(Representation representation, const Vec3& payload)
    : representation{ representation }
    , payload{ payload }
    , data{ nullptr }
{
    BulbitAssert(representation != Representation::Data);
}

Spectrum::Spectrum(std::unique_ptr<SpectralData>&& data)
    : representation{ Representation::Data }
    , payload{ 0.0f }
    , data{ std::move(data) }
{
}

uint64_t HashSpectrum(const Spectrum& sp)
{
    switch (sp.representation)
    {
    case Spectrum::Representation::Constant:
    case Spectrum::Representation::RGBReflectance:
    case Spectrum::Representation::RGBIlluminant:
    case Spectrum::Representation::Blackbody:
        return Hash(sp.representation, sp.payload);
    case Spectrum::Representation::Data:
        if (!sp.data)
        {
            return Hash(sp.representation, 0u);
        }
        return Hash(sp.representation, HashBuffer(sp.data->samples.data(), spectral::num_samples * sizeof(Float)));
    }

    return 0;
}

bool operator==(const Spectrum& lhs, const Spectrum& rhs)
{
    return lhs.Equals(rhs);
}

bool operator!=(const Spectrum& lhs, const Spectrum& rhs)
{
    return !lhs.Equals(rhs);
}

const Spectrum Spectrum::black(0.0f);

} // namespace bulbit
