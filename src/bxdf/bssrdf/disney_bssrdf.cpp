#include "bulbit/bssrdfs.h"

namespace bulbit
{

Float DisneyBSSRDF::MaxSr(const WavelengthSample& lambda) const
{
    return 0.5f * Sample_Sr(lambda, 0.999f) + 0.5f * Sample_Sr(lambda, 0.249f);
}

SpectrumSample DisneyBSSRDF::Sr(Float r) const
{
    if (r < 1e-6f)
    {
        r = 1e-6f;
    }

    SpectrumSample term0 = Exp(SpectrumSample(-r) / d);
    SpectrumSample term1 = Exp(SpectrumSample(-r) / (3.0f * d));
    return SafeDiv(R * (term0 + term1), d * (8.0f * pi * r));
}

Float DisneyBSSRDF::Sample_Sr(const WavelengthSample& lambda, Float u) const
{
    const int32 hero = WavelengthSample::hero_lane;
    BulbitNotUsed(lambda);
    Float d_hero = d[hero];

    if (u < 0.25f)
    {
        // Sample the first exponential
        u = std::min<Float>(u * 4, 1 - epsilon); // renormalize to [0,1)
        return d_hero * std::log(1 / (1 - u));
    }
    else
    {
        // Sample the second exponenital
        u = std::min<Float>((u - .25f) / .75f, 1 - epsilon); // normalize to [0,1)
        return 3 * d_hero * std::log(1 / (1 - u));
    }
}

SpectrumSample DisneyBSSRDF::PDF_Sr(Float r) const
{
    if (r < 1e-6f)
    {
        r = 1e-6f;
    }

    // Weight the two individual PDFs
    SpectrumSample pdf0 = SafeDiv(0.25f * Exp(SpectrumSample(-r) / d), d * (2.0f * pi * r));
    SpectrumSample pdf1 = SafeDiv(0.75f * Exp(SpectrumSample(-r) / (3.0f * d)), d * (6.0f * pi * r));
    return pdf0 + pdf1;
}

} // namespace bulbit
