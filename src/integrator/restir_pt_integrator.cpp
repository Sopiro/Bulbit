#include "bulbit/async_job.h"
#include "bulbit/bsdf.h"
#include "bulbit/bxdfs.h"
#include "bulbit/camera.h"
#include "bulbit/hash.h"
#include "bulbit/integrators.h"
#include "bulbit/lights.h"
#include "bulbit/parallel_for.h"
#include "bulbit/progresses.h"
#include "bulbit/sampler.h"
#include "bulbit/sampling.h"

namespace bulbit
{

struct ReSTIRPTVisiblePoint
{
    Float primary_weight;

    Vec3 wo;
    Intersection isect;
    BSDF bsdf;

    Spectrum Le;

    ReSTIRPTVisiblePoint()
        : bsdf_buffer(&bxdf_mem, sizeof(bxdf_mem))
    {
    }

    int8 bxdf_mem[max_bxdf_size];
    BufferResource bsdf_buffer;
};

struct ReSTIRPTSample
{
    int32 path_length = -1;
    int32 reconnection_vertex = -1; // Reject this sample if reconnection vertex not set

    Spectrum contribution;          // path contribution in PSS
    Float p_hat = 0;                // p_hat(contribution)
    Float W = 0;                    // UCW
};

class ReSTIRPTReservoir
{
public:
    ReSTIRPTReservoir(uint64 seed = 0)
        : w{ 0 }
        , w_sum{ 0 }
        , M{ 0 }
        , rng(seed)
    {
    }

    void Seed(uint64 seed)
    {
        rng.Seed(seed);
    }

    bool Add(const ReSTIRPTSample& sample, Float weight)
    {
        if (weight <= 0)
        {
            return false;
        }

        ++M;
        w_sum += weight;

        if (rng.NextFloat() < weight / w_sum)
        {
            y = sample;
            w = weight;
            return true;
        }

        return false;
    }

    bool HasSample() const
    {
        return w_sum > 0;
    }

    void Reset()
    {
        w = 0;
        w_sum = 0;
        M = 0;
    }

    ReSTIRPTSample y{};
    Float w;
    Float w_sum;
    Float M;

private:
    RNG rng;
};

ReSTIRPTIntegrator::ReSTIRPTIntegrator(
    const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler, int32 max_bounces, int32 rr_min_bounces
)
    : Integrator(accel, std::move(lights), std::make_unique<PowerLightSampler>())
    , sampler_prototype{ sampler }
    , max_bounces{ max_bounces }
    , rr_min_bounces{ rr_min_bounces }
{
    // Assume scene contains area lights only
    BulbitAssert(area_lights.Size() == all_lights.size());
}

Rendering* ReSTIRPTIntegrator::Render(Allocator& alloc, const Camera* camera)
{
    Point2i resolution = camera->GetScreenResolution();

    const int32 spp = sampler_prototype->samples_per_pixel;
    const int32 tile_size = 16;

    const int32 num_pixels = resolution.x * resolution.y;
    const Point2i num_tiles = (resolution + (tile_size - 1)) / tile_size;
    const int32 tile_count = num_tiles.x * num_tiles.y;
    const int32 num_passes = 2;
    const size_t total_works = size_t(std::max(spp, 1) * tile_count * num_passes);

    SinglePhaseRendering* progress = alloc.new_object<SinglePhaseRendering>(camera, total_works);
    progress->job = RunAsync([=, this]() {
        for (int32 s = 0; s < spp; ++s)
        {
            std::vector<ReSTIRPTVisiblePoint> visible_points(num_pixels);
            std::vector<ReSTIRPTReservoir> base_reservoirs(num_pixels);

            // Generate visible points and base path reservoirs using RIS in path space
            ParallelFor2D(
                resolution,
                [&](AABB2i tile) {
                    int8 sampler_mem[64];
                    BufferResource sampler_buffer(sampler_mem, sizeof(sampler_mem));
                    Allocator sampler_alloc(&sampler_buffer);
                    Sampler* sampler = sampler_prototype->Clone(sampler_alloc);

                    for (Point2i pixel : tile)
                    {
                        sampler->StartPixelSample(pixel, s);

                        const int32 index = resolution.x * pixel.y + pixel.x;
                        ReSTIRPTReservoir& reservoir = base_reservoirs[index];
                        reservoir.Seed(Hash(pixel, s, 123));

                        PrimaryRay primary_ray;
                        camera->SampleRay(&primary_ray, pixel, sampler->Next2D(), sampler->Next2D());

                        ReSTIRPTVisiblePoint& vp = visible_points[index];
                        vp.Le = Spectrum(0);
                        vp.primary_weight = primary_ray.weight;

                        int32 bounce = 0;
                        Spectrum beta(1);
                        bool specular_bounce = false;
                        Float eta_scale = 1;
                        Ray ray = primary_ray.ray;
                        Float prev_bsdf_pdf = 0;

                        bool prev_diffuse = false;

                        int32 reconnection_vertex = -1;

                        // Generate path tree with NEE path tracing
                        while (true)
                        {
                            Intersection isect;
                            if (!Intersect(&isect, ray, Ray::epsilon, infinity))
                            {
                                if (bounce == 0)
                                {
                                    vp.isect.primitive = nullptr;
                                }

                                break;
                            }

                            Vec3 wo = Normalize(-ray.d);
                            if (bounce == 0)
                            {
                                vp.wo = wo;
                                vp.isect = isect;
                            }

                            int8 mem[max_bxdf_size];
                            BufferResource res(mem, sizeof(mem));
                            Allocator alloc(&res);
                            BSDF bsdf;
                            if (!isect.GetBSDF(&bsdf, wo, alloc))
                            {
                                ray = Ray(isect.point, -wo);
                                continue;
                            }

                            // landed on diffuse surface?
                            bool is_diffuse = IsDiffuse(bsdf.Flags());
                            int32 vertex_index = bounce + 1;

                            if (reconnection_vertex < 0 && prev_diffuse && is_diffuse)
                            {
                                reconnection_vertex = vertex_index;
                            }
                            prev_diffuse = is_diffuse;

                            if (const Light* area_light = GetAreaLight(isect); area_light)
                            {
                                if (Spectrum Le = area_light->Le(isect, wo); !Le.IsBlack())
                                {
                                    Spectrum L(0);
                                    if (bounce == 0)
                                    {
                                        vp.Le = beta * Le;
                                    }
                                    else if (specular_bounce)
                                    {

                                        L = beta * Le;
                                    }
                                    else
                                    {
                                        // Evaluate BSDF sample with MIS for area light
                                        Float light_pdf =
                                            isect.primitive->GetShape()->PDF(isect, ray) * light_sampler->EvaluatePMF(area_light);
                                        Float mis_weight = PowerHeuristic(1, prev_bsdf_pdf, 1, light_pdf);

                                        L = beta * mis_weight * Le;
                                    }

                                    // Add path sample where length > 1
                                    ReSTIRPTSample sample;
                                    sample.path_length = vertex_index;
                                    if (reconnection_vertex > 0)
                                    {
                                        sample.reconnection_vertex = reconnection_vertex;
                                    }
                                    else
                                    {
                                        sample.reconnection_vertex = prev_diffuse ? vertex_index : reconnection_vertex;
                                    }
                                    sample.contribution = L;
                                    sample.p_hat = L.Average();
                                    reservoir.Add(sample, sample.p_hat);
                                }
                            }

                            if (bounce++ >= max_bounces)
                            {
                                break;
                            }

                            // Do NEE
                            Spectrum L(0);
                            if (IsNonSpecular(bsdf.Flags()))
                            {
                                L = SampleDirectLight(wo, isect, &bsdf, sampler, beta);
                            }

                            if (!L.IsBlack())
                            {
                                ReSTIRPTSample sample;
                                sample.path_length = vertex_index;
                                if (reconnection_vertex > 0)
                                {
                                    sample.reconnection_vertex = reconnection_vertex;
                                }
                                else
                                {
                                    sample.reconnection_vertex = is_diffuse ? vertex_index : reconnection_vertex;
                                }
                                sample.contribution = L;
                                sample.p_hat = L.Average();
                                reservoir.Add(sample, sample.p_hat);
                            }

                            BSDFSample bsdf_sample;
                            if (!bsdf.Sample_f(&bsdf_sample, wo, sampler->Next1D(), sampler->Next2D()))
                            {
                                break;
                            }

                            specular_bounce = bsdf_sample.IsSpecular();
                            if (bsdf_sample.IsTransmission())
                            {
                                eta_scale *= Sqr(bsdf_sample.eta);
                            }

                            // Save bsdf pdf for MIS
                            prev_bsdf_pdf = bsdf_sample.is_stochastic ? bsdf.PDF(wo, bsdf_sample.wi) : bsdf_sample.pdf;
                            beta *= bsdf_sample.f * AbsDot(isect.shading.normal, bsdf_sample.wi) / bsdf_sample.pdf;
                            ray = Ray(isect.point, bsdf_sample.wi);

                            // Terminate path with russian roulette
                            if (bounce > rr_min_bounces)
                            {
                                if (Float p = beta.MaxComponent() * eta_scale; p < 1)
                                {
                                    if (sampler->Next1D() > p)
                                    {
                                        break;
                                    }
                                    else
                                    {
                                        beta /= p;
                                    }
                                }
                            }
                        }

                        // Found no intersection
                        if (bounce == 0)
                        {
                            continue;
                        }

                        if (reservoir.HasSample())
                        {
                            reservoir.y.W = (1 / reservoir.y.p_hat) * reservoir.w_sum;
                        }
                        else
                        {
                            reservoir.y.W = 0;
                        }
                    }

                    progress->work_dones.fetch_add(1, std::memory_order_relaxed);
                },
                tile_size
            );

            // Shade
            ParallelFor2D(
                resolution,
                [&](AABB2i tile) {
                    for (Point2i pixel : tile)
                    {
                        const int32 index = resolution.x * pixel.y + pixel.x;
                        const ReSTIRPTVisiblePoint& vp = visible_points[index];

                        if (!vp.isect.primitive)
                        {
                            progress->film.AddSample(pixel, Spectrum::black);
                            continue;
                        }

                        Spectrum L = vp.Le;
                        const ReSTIRPTSample& sample = base_reservoirs[index].y;
                        if (sample.W > 0)
                        {
                            L += sample.contribution * sample.W;
                        }

                        progress->film.AddSample(pixel, vp.primary_weight * L);
                    }

                    progress->work_dones.fetch_add(1, std::memory_order_relaxed);
                },
                tile_size
            );
        }

        progress->done.store(true, std::memory_order_release);
        return true;
    });

    return progress;
}

Spectrum ReSTIRPTIntegrator::SampleDirectLight(
    const Vec3& wo, const Intersection& isect, BSDF* bsdf, Sampler* sampler, const Spectrum& beta
) const
{
    Float u0 = sampler->Next1D();
    Point2 u12 = sampler->Next2D();
    SampledLight sampled_light;
    if (!light_sampler->Sample(&sampled_light, isect, u0))
    {
        return Spectrum::black;
    }

    LightSampleLi light_sample;
    if (!sampled_light.light->Sample_Li(&light_sample, isect, u12))
    {
        return Spectrum::black;
    }

    Float bsdf_pdf = bsdf->PDF(wo, light_sample.wi);
    if (light_sample.Li.IsBlack() || bsdf_pdf == 0)
    {
        return Spectrum::black;
    }

    Ray shadow_ray(isect.point, light_sample.wi);
    if (IntersectAny(shadow_ray, Ray::epsilon, light_sample.visibility))
    {
        return Spectrum::black;
    }

    Float light_pdf = sampled_light.pmf * light_sample.pdf;
    Spectrum f_cos = bsdf->f(wo, light_sample.wi) * AbsDot(isect.shading.normal, light_sample.wi);

    if (sampled_light.light->IsDeltaLight())
    {
        return beta * light_sample.Li * f_cos / light_pdf;
    }
    else
    {
        Float mis_weight = PowerHeuristic(1, light_pdf, 1, bsdf_pdf);
        return beta * mis_weight * light_sample.Li * f_cos / light_pdf;
    }
}

} // namespace bulbit
