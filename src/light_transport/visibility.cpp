#include "bulbit/visibility.h"
#include "bulbit/integrator.h"
#include "bulbit/media.h"

namespace bulbit
{

bool V(const Integrator* I, const Point3 p1, const Point3 p2)
{
    Vec3 w = p2 - p1;
    Float visibility = w.Normalize() - Ray::epsilon;

    Ray ray(p1, w);

    Intersection isect;
    while (visibility > 0 && I->Intersect(&isect, ray, Ray::epsilon, visibility))
    {
        if (isect.primitive->GetMaterial())
        {
            return false;
        }

        ray.o = isect.point;
        visibility -= isect.t;
    }

    return true;
}

SpectrumSample Tr(const Integrator* I, const Point3 p1, const Point3 p2, const Medium* medium, const WavelengthSample& lambda)
{
    Vec3 w = p2 - p1;
    Float visibility = w.Normalize() - Ray::epsilon;

    Ray ray(p1, w);

    SpectrumSample Tr(1);
    SpectrumSample r_pdf(1);

    constexpr int32 hero = WavelengthSample::hero_lane;
    RNG rng(Hash(p1, lambda.lambda[hero]), Hash(p2, lambda.lambda[hero]));

    while (visibility > 0)
    {
        Intersection isect;
        bool found_intersection = I->Intersect(&isect, ray, Ray::epsilon, visibility);

        if (found_intersection && isect.primitive->GetMaterial())
        {
            return SpectrumSample(0);
        }

        if (medium)
        {
            Float t_max = found_intersection ? isect.t : visibility;

            SpectrumSample T_maj = Sample_MajorantTransmittance(
                medium, lambda, ray, t_max, rng.NextFloat(), rng,
                [&](Point3 p, MediumSample ms, SpectrumSample sigma_maj, SpectrumSample T_maj) {
                    BulbitNotUsed(p);

                    SpectrumSample sigma_a = ms.sigma_a;
                    SpectrumSample sigma_s = ms.sigma_s;
                    SpectrumSample sigma_n = Max<Float>(sigma_maj - sigma_a - sigma_s, 0);
                    Float pdf = sigma_maj[hero] * T_maj[hero];

                    Tr *= sigma_n * T_maj / pdf;
                    r_pdf *= sigma_maj * T_maj / pdf;

                    return !Tr.IsBlack() && !r_pdf.IsBlack();
                }
            );

            Float pdf = T_maj[hero];
            Tr *= T_maj / pdf;
            r_pdf *= T_maj / pdf;
        }

        if (Tr.IsBlack())
        {
            return SpectrumSample(0);
        }

        if (!found_intersection)
        {
            break;
        }

        ray.o = isect.point;
        visibility -= isect.t;
        medium = isect.GetMedium(ray.d);
    }

    return Tr / r_pdf.Average();
}

} // namespace bulbit
