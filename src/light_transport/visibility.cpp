#include "bulbit/visibility.h"
#include "bulbit/integrator.h"
#include "bulbit/media.h"

namespace bulbit
{

bool V(const Integrator* I, const Point3 p1, const Point3 p2)
{
    Vec3 d = p2 - p1;
    Float visibility = d.Normalize() - Ray::epsilon;
    Ray ray(p1, d);

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

Spectrum Tr(const Integrator* I, const Point3 p1, const Point3 p2, const Medium* medium, int32 wavelength)
{
    Vec3 w = p2 - p1;
    Float visibility = w.Normalize();

    Ray ray(p1, w);

    Spectrum Tr(1);
    Spectrum r_pdf(1);

    RNG rng(Hash(p1, wavelength), Hash(p2, wavelength));

    while (visibility > 0)
    {
        Intersection isect;
        bool found_intersection = I->Intersect(&isect, ray, Ray::epsilon, visibility);

        // Intersects opaque boundary
        if (found_intersection && isect.primitive->GetMaterial())
        {
            return Spectrum::black;
        }

        if (medium)
        {
            Float t_max = found_intersection ? isect.t : visibility;

            // Estimate transmittance with ratio tracking
            Spectrum T_maj = Sample_MajorantTransmittance(
                medium, wavelength, ray, t_max, rng.NextFloat(), rng,
                [&](Point3 p, MediumSample ms, Spectrum sigma_maj, Spectrum T_maj) {
                    BulbitNotUsed(p);

                    Spectrum sigma_n = Max<Float>(sigma_maj - ms.sigma_a - ms.sigma_s, 0);
                    Float pdf = sigma_maj[wavelength] * T_maj[wavelength];

                    Tr *= sigma_n * T_maj / pdf;
                    r_pdf *= sigma_maj * T_maj / pdf;

                    return !Tr.IsBlack() && !r_pdf.IsBlack();
                }
            );

            Float pdf = T_maj[wavelength];
            Tr *= T_maj / pdf;
            r_pdf *= T_maj / pdf;
        }

        if (Tr.IsBlack())
        {
            return Spectrum::black;
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
