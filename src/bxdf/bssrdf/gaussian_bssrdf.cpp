#include "bulbit/bssrdfs.h"

namespace bulbit
{

Float GaussianBSSRDF::MaxSr(const WavelengthSample& lambda) const
{
    return Sample_Sr(lambda, 1 - 0.999f);
}

SpectrumSample GaussianBSSRDF::Sr(Float r) const
{
    return R * PDF_Sr(r);
}

Float GaussianBSSRDF::Sample_Sr(const WavelengthSample& lambda, Float u) const
{
    const int32 hero = WavelengthSample::hero_lane;
    BulbitNotUsed(lambda);
    Float sigma_hero = sigma[hero];
    return sigma_hero * std::sqrt(-2 * std::log(std::max<Float>(u, Float(1e-8))));
}

SpectrumSample GaussianBSSRDF::PDF_Sr(Float r) const
{
    SpectrumSample sigma2 = sigma * sigma;
    SpectrumSample exponent = SpectrumSample(-Sqr(r)) / (2.0f * sigma2);
    return SafeDiv(Exp(exponent), two_pi * sigma2);
}

} // namespace bulbit
