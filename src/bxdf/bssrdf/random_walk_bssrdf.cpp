#include "bulbit/bssrdf.h"
#include "bulbit/hash.h"
#include "bulbit/primitive.h"

namespace bulbit
{

Spectrum RandomWalkBSSRDF::S(const Intersection& pi, const Vec3& wi, TransportDirection direction) const
{
    BulbitNotUsed(pi);
    BulbitNotUsed(wi);
    BulbitNotUsed(direction);
    BulbitAssert(false);

    return Spectrum::black;
}

bool RandomWalkBSSRDF::Sample_S(
    BSSRDFSample* bssrdf_sample, const BSDFSample& bsdf_sample, const Intersectable* accel, int32 wavelength, Float u0, Point2 u12
)
{
    BulbitAssert(bsdf_sample.IsTransmission());

    Spectrum beta(1);
    Spectrum pdf(1);

    RNG rng(Hash(po, u0, u12));

    Intersection pi;
    Ray ray(po.point, bsdf_sample.wi);

    const int32 min_bounces = 2;
    const int32 max_bounces = 256;
    const Float rr = 0.995f;

    int32 bounce = 0;
    while (bounce++ < max_bounces)
    {
        // Stochastically terminate path with russian roulette
        if (bounce > min_bounces)
        {
            if (rng.NextFloat() > rr)
            {
                return false;
            }
            beta /= rr;
            pdf /= rr;
        }

        Float d = SampleExponential(rng.NextFloat(), sigma_t[wavelength]);

        bool found_intersection = accel->Intersect(&pi, ray, Ray::epsilon, d);
        if (found_intersection)
        {
            // Sampled L_o, escaped medium boundary
            if (pi.primitive->GetMaterial() == po.primitive->GetMaterial())
            {
                Spectrum Tr = Exp(-sigma_t * pi.t);

                beta *= Tr / Tr[wavelength];
                pdf *= Tr / Tr[wavelength];
                break;
            }
        }

        Spectrum Tr = Exp(-sigma_t * d);
        beta *= sigma_t * Tr / sigma_t[wavelength] * Tr[wavelength];
        pdf *= sigma_t * Tr / sigma_t[wavelength] * Tr[wavelength];

        // Random walk forward
        ray.o += d * ray.d;

        PhaseFunctionSample sample;
        if (!phase_function.Sample_p(&sample, -ray.d, { rng.NextFloat(), rng.NextFloat() }))
        {
            return false;
        }
        ray.d = sample.wi;

        // Evaluate L_s
        beta *= albedo * sample.p / sample.pdf;
        pdf *= sample.pdf / sample.pdf;
    }

    if (bounce >= max_bounces)
    {
        return false;
    }

    bssrdf_sample->pi = pi;
    bssrdf_sample->Sp = beta * R;
    bssrdf_sample->pdf = pdf;
    bssrdf_sample->p = 1;

    Vec3 n = pi.front_face ? pi.shading.normal : -pi.shading.normal;
    bssrdf_sample->Sw = BSDF(n, &sw);
    bssrdf_sample->wo = n;
    return true;
}

} // namespace bulbit
