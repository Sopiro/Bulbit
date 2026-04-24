#include "bulbit/media.h"

namespace bulbit
{

HomogeneousMedium::HomogeneousMedium(Spectrum sigma_a, Spectrum sigma_s, Spectrum Le, Float g)
    : Medium(TypeIndexOf<HomogeneousMedium>())
    , sigma_a{ sigma_a }
    , sigma_s{ sigma_s }
    , Le{ Le }
    , phase{ g }
{
}

bool HomogeneousMedium::IsEmissive() const
{
    return !Le.IsBlack();
}

MediumSample HomogeneousMedium::SamplePoint(Point3 p, const WavelengthSample& lambda) const
{
    BulbitNotUsed(p);
    return MediumSample{ sigma_a.Sample(lambda), sigma_s.Sample(lambda), Le.Sample(lambda), &phase };
}

HomogeneousMajorantIterator HomogeneousMedium::SampleRay(Ray ray, Float t_max, const WavelengthSample& lambda) const
{
    BulbitNotUsed(ray);
    BulbitNotUsed(t_max);
    return HomogeneousMajorantIterator(Float(0), t_max, sigma_a.Sample(lambda) + sigma_s.Sample(lambda));
}

RayMajorantIterator* HomogeneousMedium::SampleRay(Ray ray, Float t_max, const WavelengthSample& lambda, Allocator& alloc) const
{
    BulbitNotUsed(ray);
    return alloc.new_object<HomogeneousMajorantIterator>(Float(0), t_max, sigma_a.Sample(lambda) + sigma_s.Sample(lambda));
}

} // namespace bulbit
