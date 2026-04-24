#pragma once

#include "bssrdf.h"

namespace bulbit
{

// Approximate Reflectance Profiles for Efficient Subsurface Scattering
// by Christensen and Burley
class DisneyBSSRDF : public SeparableBSSRDF
{
public:
    DisneyBSSRDF(const SpectrumSample& R, const SpectrumSample& d, const Intersection& po, Float eta)
        : SeparableBSSRDF(po, eta)
        , R{ R }
        , d{ d }
    {
    }

    virtual Float MaxSr(const WavelengthSample& lambda) const override;
    virtual SpectrumSample Sr(Float d) const override;
    virtual Float Sample_Sr(const WavelengthSample& lambda, Float u) const override;
    virtual SpectrumSample PDF_Sr(Float r) const override;

private:
    SpectrumSample R, d;
};

// Very non-physically based BSSRDF
class GaussianBSSRDF : public SeparableBSSRDF
{
public:
    GaussianBSSRDF(const SpectrumSample& R, const SpectrumSample& sigma, const Intersection& po, Float eta)
        : SeparableBSSRDF(po, eta)
        , R{ R }
        , sigma{ sigma }
    {
    }

    virtual Float MaxSr(const WavelengthSample& lambda) const override;
    virtual SpectrumSample Sr(Float d) const override;
    virtual Float Sample_Sr(const WavelengthSample& lambda, Float u) const override;
    virtual SpectrumSample PDF_Sr(Float r) const override;

private:
    SpectrumSample R;
    SpectrumSample sigma;
};

constexpr size_t max_bssrdf_size = std::max({ sizeof(DisneyBSSRDF), sizeof(GaussianBSSRDF), sizeof(RandomWalkBSSRDF) });

} // namespace bulbit
