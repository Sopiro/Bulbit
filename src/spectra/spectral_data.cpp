#include "bulbit/spectrum.h"

namespace bulbit
{

namespace
{

using spectral::num_samples;

const std::array<Float, num_samples> mallett_basis_r_samples = {
    0.32745741382705507f,   0.3237505782705411f,   0.3134394612515771f,    0.28887938275526515f,   0.23920568115888585f,
    0.18970203689053514f,   0.121746067959218f,    0.07457827066946597f,   0.04443315863403367f,   0.028928632128502902f,
    0.022316653484751203f,  0.0169113072926318f,   0.014181107117966694f,  0.013053142677487299f,  0.011986163627845001f,
    0.011288714712404802f,  0.010906066465651705f, 0.010400713481004206f,  0.0106373602541465f,    0.010907662533774093f,
    0.011032712448098796f,  0.011310656591226797f, 0.011154642056940303f,  0.010148770406212204f,  0.00891858211883843f,
    0.00768557633847106f,   0.006705708284695256f, 0.005995805987644238f,  0.0055372566423418906f, 0.005193784241206632f,
    0.005025362265223342f,  0.005136362769675081f, 0.0054332002605398284f, 0.00581998590243535f,   0.006400572774624122f,
    0.0074495286834087805f, 0.008583635819376573f, 0.010395762465167406f,  0.013565433538649196f,  0.0193845158399742f,
    0.03208407120200237f,   0.07435603784594108f,  0.6243937241780748f,    0.91831003276872f,      0.9492530301750509f,
    0.9581878333292456f,    0.958187751332698f,    0.9581876250877813f,    0.9556790607717461f,    0.9580061548934292f,
    0.9541015734565635f,    0.9476076062372374f,   0.9386813284475495f,    0.9244666827514341f,    0.9046060253330559f,
    0.8804121989279332f,    0.8477878731517f,      0.8057791266230191f,    0.752531853871421f,     0.686439396844578f,
    0.6186945708606102f,    0.5402644439591109f,   0.472964416293838f,     0.432701596704049f,     0.40535804552839205f,
    0.385491834974902f,     0.370983584551061f,    0.357608701523081f,     0.34871280010839295f,   0.34488011934469104f,
    0.34191787732329093f,   0.3395310929871289f,   0.3371695037743671f,    0.3361720185277169f,    0.335167443433363f,
    0.334421625306463f,     0.3340087603764019f,   0.33391579279008193f,   0.3338184549463669f,    0.33367277492845593f,
    0.33356951340559104f
};

const std::array<Float, num_samples> mallett_basis_g_samples = {
    0.33186171308587403f,  0.32968818775939907f,  0.32786002162469713f,  0.3191735802317559f,   0.29432258369484204f,
    0.25869706476873583f,  0.18889431925476496f,  0.125388381991689f,    0.07868706031062171f,  0.05314327086594527f,
    0.042288146031342094f, 0.033318345502917124f, 0.0297559481859724f,   0.030331250536904702f, 0.030988571897300705f,
    0.03168635518883811f,  0.034669961502997386f, 0.034551957443674994f, 0.04068480619482971f,  0.05446003736940562f,
    0.08090528742047369f,  0.14634830285704392f,  0.3796796432966169f,   0.7667442686540331f,   0.8762147476133697f,
    0.9184916556138432f,   0.9406555625344368f,   0.9537318845330199f,   0.9616432798402382f,   0.9672000196850782f,
    0.9709897463900459f,   0.9728523035635542f,   0.9731165940764441f,   0.9733510691541436f,   0.9733511155443689f,
    0.9722610797317249f,   0.9733510217469168f,   0.973148495185693f,    0.9710613063009141f,   0.9663713059551835f,
    0.9549419675025478f,   0.9135789895512614f,   0.3643488039076871f,   0.0715072425408852f,   0.041230434471375116f,
    0.032423874183668516f, 0.031924629798200314f, 0.03127603317309691f,  0.032630370429057406f, 0.029530872149073892f,
    0.0315617611702464f,   0.03567421827082041f,  0.0414030053955673f,   0.05060426044895608f,  0.0634343003817003f,
    0.07891824529392293f,  0.0995427426653747f,   0.12559576009328705f,  0.15759091044168005f,  0.19539823904421008f,
    0.231474474772178f,    0.26885213609526193f,  0.29602916421792796f,  0.30975499444194493f,  0.31781588338382194f,
    0.32299034738989796f,  0.32635384793800903f,  0.3291439022789799f,   0.330808726803682f,    0.3314826899222431f,
    0.33198455035238894f,  0.33234117252254497f,  0.33291200941553906f,  0.332919279695214f,    0.3330276725788561f,
    0.33317970467325997f,  0.33324703097454916f,  0.33325934921060096f,  0.33327505027938276f,  0.33329432844873197f,
    0.33330942495777494f
};

const std::array<Float, num_samples> mallett_basis_b_samples = {
    0.34068079154805203f,  0.3465611866248519f,   0.3587004931403511f,   0.39194702658819497f,  0.46647173058733304f,
    0.551600895598602f,    0.6893596109489281f,   0.8000333468786072f,   0.8768797809353139f,   0.9179280974439552f,
    0.9353952006696319f,   0.9497703471151834f,   0.9560629448052402f,   0.9566156068903161f,   0.957025264931328f,
    0.9570249305347132f,   0.9544239727370661f,   0.9550473290202036f,   0.9486778330933341f,   0.9346322998423281f,
    0.9080619998522691f,   0.8423410394637268f,   0.6091657153656471f,   0.22310696095953275f,  0.11486667029133601f,
    0.07382276789574371f,  0.0526387287910555f,   0.04027230901688872f,  0.032819462650959114f, 0.027606195927045607f,
    0.023984891127039394f, 0.022011333352792203f, 0.021450205255996606f, 0.020828944509568504f, 0.020248311388808695f,
    0.020289391451206607f, 0.01806534233591301f,  0.0164557422344685f,   0.015373260134095497f, 0.014244178484551704f,
    0.012973961554334697f, 0.012064974134521796f, 0.011257478160390105f, 0.010182724671694201f, 0.009516535387237415f,
    0.00938829272866817f,  0.009887619090670282f, 0.0105363420064589f,   0.011690568837444794f, 0.012462972887103704f,
    0.014336665177420299f, 0.016718175327544296f, 0.0199156660750025f,   0.024929056163280998f, 0.03195967358604021f,
    0.040669554095248396f, 0.052669382421939595f, 0.06862511051419468f,  0.0898772323000136f,   0.118162358926434f,
    0.14983094744213296f,  0.190883409341834f,    0.231006403025217f,    0.25754338542220195f,  0.27682603872153594f,
    0.2915177728107951f,   0.3026625060832329f,   0.31324730130288614f,  0.3204783251246331f,   0.3236369947079611f,
    0.32609730884689997f,  0.32812736934018377f,  0.32991797595888805f,  0.33090790121664887f,  0.33180363309599503f,
    0.33239662725536107f,  0.3327407807268239f,   0.33282085708148906f,  0.33290173128344425f,  0.333025967488632f,
    0.333111083081497f
};

const std::array<Float, num_samples> d65_samples = {
    49.9755f, 52.3118f, 54.6482f, 68.7015f, 82.7549f, 87.1204f, 91.486f,  92.4589f, 93.4318f, 90.057f,  86.6823f, 95.7736f,
    104.865f, 110.936f, 117.008f, 117.41f,  117.812f, 116.336f, 114.861f, 115.392f, 115.923f, 112.367f, 108.811f, 109.082f,
    109.354f, 108.578f, 107.802f, 106.296f, 104.79f,  106.239f, 107.689f, 106.047f, 104.405f, 104.225f, 104.046f, 102.023f,
    100.0f,   98.1671f, 96.3342f, 96.0611f, 95.788f,  92.2368f, 88.6856f, 89.3459f, 90.0062f, 89.8026f, 89.5991f, 88.6489f,
    87.6987f, 85.4936f, 83.2886f, 83.4939f, 83.6992f, 81.863f,  80.0268f, 80.1207f, 80.2146f, 81.2462f, 82.2778f, 80.281f,
    78.2842f, 74.0027f, 69.7213f, 70.6652f, 71.6091f, 72.979f,  74.349f,  67.9765f, 61.604f,  65.7448f, 69.8856f, 72.4863f,
    75.087f,  69.3398f, 63.5927f, 55.0054f, 46.4182f, 56.6118f, 66.8054f, 65.0941f, 63.3828f
};

const std::array<Float, num_samples> cie_x_samples = {
    0.001368f,     0.002236f,     0.004243f,     0.00765f,      0.01431f,      0.02319f,     0.04351f,      0.07763f,
    0.13438f,      0.21477f,      0.2839f,       0.3285f,       0.34828f,      0.34806f,     0.3362f,       0.3187f,
    0.2908f,       0.2511f,       0.19536f,      0.1421f,       0.09564f,      0.05795001f,  0.03201f,      0.0147f,
    0.0049f,       0.0024f,       0.0093f,       0.0291f,       0.06327f,      0.1096f,      0.1655f,       0.2257499f,
    0.2904f,       0.3597f,       0.4334499f,    0.5120501f,    0.5945f,       0.6784f,      0.7621f,       0.8425f,
    0.9163f,       0.9786f,       1.0263f,       1.0567f,       1.0622f,       1.0456f,      1.0026f,       0.9384f,
    0.8544499f,    0.7514f,       0.6424f,       0.5419f,       0.4479f,       0.3608f,      0.2835f,       0.2187f,
    0.1649f,       0.1212f,       0.0874f,       0.0636f,       0.04677f,      0.0329f,      0.0227f,       0.01584f,
    0.01135916f,   0.008110916f,  0.005790346f,  0.004109457f,  0.002899327f,  0.00204919f,  0.001439971f,  0.0009999493f,
    0.0006900786f, 0.0004760213f, 0.0003323011f, 0.0002348261f, 0.0001661505f, 0.000117413f, 8.307527e-05f, 5.870652e-05f,
    4.150994e-05f
};

const std::array<Float, num_samples> cie_y_samples = {
    3.9e-05f,  6.4e-05f,   0.00012f,  0.000217f, 0.000396f, 0.00064f,   0.00121f,   0.00218f,   0.004f,    0.0073f,    0.0116f,
    0.01684f,  0.023f,     0.0298f,   0.038f,    0.048f,    0.06f,      0.0739f,    0.09098f,   0.1126f,   0.13902f,   0.1693f,
    0.20802f,  0.2586f,    0.323f,    0.4073f,   0.503f,    0.6082f,    0.71f,      0.7932f,    0.862f,    0.9148501f, 0.954f,
    0.9803f,   0.9949501f, 1.0f,      0.995f,    0.9786f,   0.952f,     0.9154f,    0.87f,      0.8163f,   0.757f,     0.6949f,
    0.631f,    0.5668f,    0.503f,    0.4412f,   0.381f,    0.321f,     0.265f,     0.217f,     0.175f,    0.1382f,    0.107f,
    0.0816f,   0.061f,     0.04458f,  0.032f,    0.0232f,   0.017f,     0.01192f,   0.00821f,   0.005723f, 0.004102f,  0.002929f,
    0.002091f, 0.001484f,  0.001047f, 0.00074f,  0.00052f,  0.0003611f, 0.0002492f, 0.0001719f, 0.00012f,  8.48e-05f,  6e-05f,
    4.24e-05f, 3e-05f,     2.12e-05f, 1.499e-05f
};

const std::array<Float, num_samples> cie_z_samples = {
    0.006450001f, 0.01054999f,  0.02005001f, 0.03621f, 0.06785001f,  0.1102f,  0.2074f, 0.3713f,      0.6456f,
    1.0390501f,   1.3856f,      1.62296f,    1.74706f, 1.7826f,      1.77211f, 1.7441f, 1.6692f,      1.5281f,
    1.28764f,     1.0419f,      0.8129501f,  0.6162f,  0.46518f,     0.3533f,  0.272f,  0.2123f,      0.1582f,
    0.1117f,      0.07824999f,  0.05725001f, 0.04216f, 0.02984f,     0.0203f,  0.0134f, 0.008749999f, 0.005749999f,
    0.0039f,      0.002749999f, 0.0021f,     0.0018f,  0.001650001f, 0.0014f,  0.0011f, 0.001f,       0.0008f,
    0.0006f,      0.00034f,     0.00024f,    0.00019f, 0.0001f,      5e-05f,   3e-05f,  2e-05f,       1e-05f,
    0.0f,         0.0f,         0.0f,        0.0f,     0.0f,         0.0f,     0.0f,    0.0f,         0.0f,
    0.0f,         0.0f,         0.0f,        0.0f,     0.0f,         0.0f,     0.0f,    0.0f,         0.0f,
    0.0f,         0.0f,         0.0f,        0.0f,     0.0f,         0.0f,     0.0f,    0.0f,         0.0f
};

} // namespace

SpectralData::SpectralData(const std::array<Float, spectral::num_samples>& values)
    : samples{ values }
{
}

SpectralData::SpectralData(std::array<Float, spectral::num_samples>&& values)
    : samples{ std::move(values) }
{
}

Float SpectralData::operator[](int32 i) const
{
    return samples[i];
}

Float& SpectralData::operator[](int32 i)
{
    return samples[i];
}

SpectralData SpectralData::Constant(Float value)
{
    std::array<Float, spectral::num_samples> values{};
    values.fill(value);
    return SpectralData(std::move(values));
}

SpectralData SpectralData::CauchyIOR(Float cauchy_a, Float cauchy_b)
{
    std::array<Float, spectral::num_samples> values{};
    for (int32 i = 0; i < spectral::num_samples; ++i)
    {
        Float wavelength_nm = spectral::lambda_min + i * spectral::lambda_step;
        Float wavelength_um = wavelength_nm * 1e-3f;
        values[i] = cauchy_a + cauchy_b / Sqr(wavelength_um);
    }
    return SpectralData(std::move(values));
}

SpectrumSample SpectralData::Sample(const WavelengthSample& wavelengths) const
{
    SpectrumSample result;
    for (int32 i = 0; i < WavelengthSample::num_lanes; ++i)
    {
        result[i] = spectral::Interpolate(*this, wavelengths.lambda[i]);
    }
    return result;
}

bool SpectralData::IsBlack() const
{
    for (Float sample : samples)
    {
        if (sample != 0.0f)
        {
            return false;
        }
    }
    return true;
}

bool SpectralData::IsNullish() const
{
    for (Float sample : samples)
    {
        if (std::isnan(sample) || std::isinf(sample))
        {
            return true;
        }
    }
    return false;
}

Float SpectralData::Average() const
{
    Float sum = 0.0f;
    for (Float sample : samples)
    {
        sum += sample;
    }
    return sum / Float(spectral::num_samples);
}

Float SpectralData::MinComponent() const
{
    Float minimum = samples[0];
    for (int32 i = 1; i < spectral::num_samples; ++i)
    {
        minimum = std::min(minimum, samples[i]);
    }
    return minimum;
}

Float SpectralData::MaxComponent() const
{
    Float maximum = samples[0];
    for (int32 i = 1; i < spectral::num_samples; ++i)
    {
        maximum = std::max(maximum, samples[i]);
    }
    return maximum;
}

SpectrumSample WavelengthSample::PDF() const
{
    SpectrumSample result;
    for (int32 i = 0; i < num_lanes; ++i)
    {
        result[i] = pdf[i];
    }
    return result;
}

namespace spectral
{

const SpectralData mallett_basis_r(mallett_basis_r_samples);
const SpectralData mallett_basis_g(mallett_basis_g_samples);
const SpectralData mallett_basis_b(mallett_basis_b_samples);
const SpectralData d65(d65_samples);
const SpectralData cie_x(cie_x_samples);
const SpectralData cie_y(cie_y_samples);
const SpectralData cie_z(cie_z_samples);

Float Interpolate(const SpectralData& samples, Float wavelength)
{
    InterpolationWeights weights = GetInterpolationWeights(wavelength);
    return Lerp(samples[weights.index], samples[weights.index + 1], weights.t);
}

Float Interpolate(const SpectralData& samples, const InterpolationWeights& weights)
{
    return Lerp(samples[weights.index], samples[weights.index + 1], weights.t);
}

SpectrumSample Sample(const SpectralData& data, const WavelengthSample& wavelengths)
{
    return data.Sample(wavelengths);
}

Vec3 ToXYZ(const SpectrumSample& sp, const WavelengthSample& lambda)
{
    Vec3 xyz(0);
    for (int32 i = 0; i < WavelengthSample::num_lanes; ++i)
    {
        if (lambda.pdf[i] == 0.0f)
        {
            continue;
        }

        InterpolationWeights weights = GetInterpolationWeights(lambda.lambda[i]);
        Float coeff = monte_carlo_integral_scale / lambda.pdf[i];
        xyz += coeff * sp[i] * Vec3(Interpolate(cie_x, weights), Interpolate(cie_y, weights), Interpolate(cie_z, weights));
    }

    xyz /= WavelengthSample::num_lanes;

    return xyz;
}

Vec3 ToLinearRGB(const SpectrumSample& sp, const WavelengthSample& lambda)
{
    return Max(XYZToLinearRGB(ToXYZ(sp, lambda)), Vec3(0.0f));
}

Float Luminance(const SpectrumSample& sp, const WavelengthSample& lambda)
{
    return ToXYZ(sp, lambda).y;
}

} // namespace spectral

} // namespace bulbit
