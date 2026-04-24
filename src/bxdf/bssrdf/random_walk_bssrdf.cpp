#include "bulbit/bssrdf.h"
#include "bulbit/hash.h"
#include "bulbit/primitive.h"

namespace bulbit
{

SpectrumSample RandomWalkBSSRDF::S(const Intersection& pi, const Vec3& wo, const Vec3& wi, TransportDirection direction) const
{
    BulbitNotUsed(pi);
    BulbitNotUsed(wo);
    BulbitNotUsed(wi);
    BulbitNotUsed(direction);
    BulbitAssert(false);

    return SpectrumSample(0);
}

bool RandomWalkBSSRDF::Sample_S(
    BSSRDFSample* bssrdf_sample,
    const BSDFSample& bsdf_sample,
    const Intersectable* accel,
    const WavelengthSample& lambda,
    Float u0,
    Point2 u12
)
{
    BulbitAssert(bsdf_sample.IsTransmission());

    BulbitNotUsed(lambda);

    const int32 hero = WavelengthSample::hero_lane;
    SpectrumSample R_sample = R;
    SpectrumSample sigma_t = sigma_a + sigma_s;
    SpectrumSample albedo = SafeDiv(sigma_s, sigma_t);

    SpectrumSample beta(1);
    SpectrumSample pdf(1);

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

        Float d = SampleExponential(rng.NextFloat(), sigma_t[hero]);

        bool found_intersection = accel->Intersect(&pi, ray, Ray::epsilon, d);
        if (found_intersection)
        {
            // Sampled L_o, escaped medium boundary
            if (pi.primitive->GetMaterial() == po.primitive->GetMaterial())
            {
                SpectrumSample Tr = Exp(-sigma_t * pi.t);

                beta *= Tr / Tr[hero];
                pdf *= Tr / Tr[hero];
                break;
            }
        }

        SpectrumSample Tr = Exp(-sigma_t * d);
        Float distance_pdf = sigma_t[hero] * Tr[hero];
        beta *= sigma_t * Tr / distance_pdf;
        pdf *= sigma_t * Tr / distance_pdf;

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
    }

    if (bounce >= max_bounces)
    {
        return false;
    }

    if (pdf.IsBlack())
    {
        return false;
    }

    bssrdf_sample->pi = pi;
    bssrdf_sample->Sp = beta * R_sample / pdf.Average();
    bssrdf_sample->pdf = pdf / pdf.Average();
    bssrdf_sample->p = 1;

    Vec3 n = pi.front_face ? pi.shading.normal : -pi.shading.normal;
    bssrdf_sample->Sw = BSDF(n, &sw);
    bssrdf_sample->wo = n;
    return true;
}

} // namespace bulbit
