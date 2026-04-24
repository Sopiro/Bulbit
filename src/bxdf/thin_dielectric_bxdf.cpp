#include "bulbit/bxdfs.h"
#include "bulbit/scattering.h"

namespace bulbit
{

namespace
{

Float ThinDielectricReflectance(Float eta, Float cos_theta)
{
    Float R = FresnelDielectric(cos_theta, eta);
    Float T = 1 - R;

    if (R < 1)
    {
        R += Sqr(T) * R / (1 - Sqr(R));
    }

    return R;
}

Float HeroEta(const SpectrumSample& eta)
{
    return eta[WavelengthSample::hero_lane];
}
} // namespace

SpectrumSample ThinDielectricBxDF::f(Vec3 wo, Vec3 wi, TransportDirection direction) const
{
    BulbitNotUsed(wo);
    BulbitNotUsed(wi);
    BulbitNotUsed(direction);
    return SpectrumSample(0);
}

Float ThinDielectricBxDF::PDF(Vec3 wo, Vec3 wi, TransportDirection direction, BxDF_SamplingFlags flags) const
{
    BulbitNotUsed(wo);
    BulbitNotUsed(wi);
    BulbitNotUsed(direction);
    BulbitNotUsed(flags);
    return 0;
}

bool ThinDielectricBxDF::Sample_f(
    BSDFSample* sample, Vec3 wo, Float u0, Point2 u12, TransportDirection direction, BxDF_SamplingFlags flags
) const
{
    BulbitNotUsed(u12);
    BulbitNotUsed(direction);

    Float R_hero = ThinDielectricReflectance(HeroEta(eta), AbsCosTheta(wo));
    Float T_hero = 1 - R_hero;

    Float pr = R_hero;
    Float pt = T_hero;
    if (!(flags & BxDF_SamplingFlags::Reflection)) pr = 0;
    if (!(flags & BxDF_SamplingFlags::Transmission)) pt = 0;
    if (pr == 0 && pt == 0)
    {
        return false;
    }

    if (u0 < pr / (pr + pt))
    {
        Vec3 wi(-wo.x, -wo.y, wo.z);
        SpectrumSample fr(0);
        for (int32 i = 0; i < SpectrumSample::num_lanes; ++i)
        {
            fr[i] = ThinDielectricReflectance(eta[i], AbsCosTheta(wo)) / AbsCosTheta(wi);
        }

        *sample = BSDFSample(fr, wi, pr / (pr + pt), BxDF_Flags::SpecularReflection);
    }
    else
    {
        Vec3 wi = -wo;
        SpectrumSample ft(0);
        for (int32 i = 0; i < SpectrumSample::num_lanes; ++i)
        {
            Float R = ThinDielectricReflectance(eta[i], AbsCosTheta(wo));
            ft[i] = (1 - R) / AbsCosTheta(wi);
        }

        SpectrumSample fs = ft * r;
        *sample = BSDFSample(fs, wi, pt / (pr + pt), BxDF_Flags::SpecularTransmission);
    }

    return true;
}

} // namespace bulbit
