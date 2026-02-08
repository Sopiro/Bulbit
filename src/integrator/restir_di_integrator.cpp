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
#include "bulbit/visibility.h"

namespace bulbit
{

struct ReSTIRDIVisiblePoint
{
    Float primary_weight;

    Intersection isect;
    Vec3 wo;

    Spectrum Le;
};

struct ReSTIRDISample
{
    Point3 x;
    Vec3 wi;
    Spectrum Li;

    Float p_hat = 0; // p_hat(y): target function value of selected sample
    Float W = 0;
};

class ReSTIRDIReservoir
{
public:
    ReSTIRDIReservoir(uint64 seed = 0)
        : w{ 0 }
        , w_sum{ 0 }
        , rng(seed)
    {
    }

    void Seed(uint64 seed)
    {
        rng.Seed(seed);
    }

    bool Add(const ReSTIRDISample& sample, Float weight)
    {
        if (weight <= 0)
        {
            return false;
        }

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

    Float GetSampleProbability() const
    {
        return w / w_sum;
    }

    void Reset()
    {
        w = 0;
        w_sum = 0;
    }

    ReSTIRDISample y{};
    Float w;
    Float w_sum;

private:
    RNG rng;
};

ReSTIRDIIntegrator::ReSTIRDIIntegrator(const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler)
    : Integrator(accel, std::move(lights), std::make_unique<PowerLightSampler>())
    , sampler_prototype{ sampler }
{
}

Rendering* ReSTIRDIIntegrator::Render(Allocator& alloc, const Camera* camera)
{
    Point2i resolution = camera->GetScreenResolution();

    const int32 spp = sampler_prototype->samples_per_pixel;
    const int32 tile_size = 16;

    const int32 num_pixels = resolution.x * resolution.y;
    const Point2i num_tiles = (resolution + (tile_size - 1)) / tile_size;
    int32 tile_count = num_tiles.x * num_tiles.y;

    const int32 num_passes = 3;

    size_t total_works = size_t(std::max(spp, 1) * tile_count * num_passes);

    AABB world_bounds = accel->GetAABB();
    Point3 p;
    Float world_radius;
    world_bounds.ComputeBoundingSphere(&p, &world_radius);

    SinglePhaseRendering* progress = alloc.new_object<SinglePhaseRendering>(camera, total_works);
    progress->job = RunAsync([=, this]() {
        for (int32 s = 0; s < spp; ++s)
        {
            std::vector<ReSTIRDIVisiblePoint> visible_points(num_pixels);
            std::vector<ReSTIRDISample> samples(num_pixels);

            // Trace primary rays and generate initial sample
            ParallelFor2D(
                resolution,
                [&](AABB2i tile) {
                    int8 sample_mem[64];
                    BufferResource sampler_buffer(sample_mem, sizeof(sample_mem));
                    Allocator sampler_alloc(&sampler_buffer);
                    Sampler* sampler = sampler_prototype->Clone(sampler_alloc);

                    int8 bxdf_mem[max_bxdf_size];
                    BufferResource bsdf_buffer(bxdf_mem, sizeof(bxdf_mem));
                    Allocator bsdf_alloc(&bsdf_buffer);

                    for (Point2i pixel : tile)
                    {
                        sampler->StartPixelSample(pixel, s);
                        const int32 index = resolution.x * pixel.y + pixel.x;

                        PrimaryRay primary_ray;
                        camera->SampleRay(&primary_ray, pixel, sampler->Next2D(), sampler->Next2D());

                        Ray ray = primary_ray.ray;

                        ReSTIRDIVisiblePoint& vp = visible_points[index];
                        Intersection& isect = vp.isect;

                        vp.primary_weight = primary_ray.weight;
                        vp.wo = Normalize(-ray.d);

                        bool found_intersection = false;
                        while (true)
                        {
                            // Find visible point
                            found_intersection = Intersect(&isect, ray, Ray::epsilon, infinity);
                            if (!found_intersection)
                            {
                                for (Light* light : infinite_lights)
                                {
                                    vp.Le += light->Le(ray);
                                }
                                isect.primitive = nullptr;
                                break;
                            }
                            else if (const Light* light = GetAreaLight(isect); light)
                            {
                                if (Spectrum Le = light->Le(isect, -ray.d); !Le.IsBlack())
                                {
                                    vp.Le += Le;
                                }
                            }

                            bsdf_buffer.release();
                            BSDF bsdf;
                            if (!isect.GetBSDF(&bsdf, vp.wo, bsdf_alloc))
                            {
                                ray.o = isect.point;
                                continue;
                            }

                            break;
                        }

                        if (!found_intersection)
                        {
                            continue;
                        }

                        bsdf_buffer.release();
                        BSDF bsdf;
                        if (!isect.GetBSDF(&bsdf, vp.wo, bsdf_alloc))
                        {
                            continue;
                        }

                        ReSTIRDIReservoir reservoir(Hash(pixel, s));

                        // RIS: draw M initial candidates, keep one by WRS, then compute W(y)
                        const int32 M_light = 8;
                        const int32 M_bsdf = 1;

                        for (int32 i = 0; i < M_light; ++i)
                        {
                            SampledLight sampled_light;
                            if (!light_sampler->Sample(&sampled_light, isect, sampler->Next1D()))
                            {
                                continue;
                            }

                            LightSampleLi light_sample;
                            if (!sampled_light.light->Sample_Li(&light_sample, isect, sampler->Next2D()))
                            {
                                continue;
                            }

                            if (light_sample.Li.IsBlack())
                            {
                                continue;
                            }

                            Float p_light = sampled_light.pmf * light_sample.pdf;
                            Float p_bsdf = bsdf.PDF(vp.wo, light_sample.wi);
                            Float mis_denom = M_light * p_light + M_bsdf * p_bsdf;
                            if (mis_denom <= 0)
                            {
                                continue;
                            }
                            Float w_mis = p_light / mis_denom;

                            Spectrum f_cos = bsdf.f(vp.wo, light_sample.wi) * AbsDot(isect.shading.normal, light_sample.wi);
                            Float p_hat = (light_sample.Li * f_cos).Luminance();
                            if (p_hat > 0)
                            {
                                ReSTIRDISample candidate;
                                candidate.x = light_sample.point;
                                candidate.Li = light_sample.Li;
                                candidate.wi = light_sample.wi;
                                candidate.p_hat = p_hat;

                                Float w = w_mis * p_hat / p_light;

                                reservoir.Add(candidate, w);
                            }
                        }

                        for (int32 i = 0; i < M_bsdf; ++i)
                        {
                            BSDFSample bsdf_sample;
                            if (!bsdf.Sample_f(&bsdf_sample, vp.wo, sampler->Next1D(), sampler->Next2D()))
                            {
                                continue;
                            }

                            if (bsdf_sample.pdf == 0 || bsdf_sample.f.IsBlack())
                            {
                                continue;
                            }

                            Float p_bsdf = bsdf_sample.is_stochastic ? bsdf.PDF(vp.wo, bsdf_sample.wi) : bsdf_sample.pdf;

                            Intersection shadow_isect;
                            Ray shadow_ray(isect.point, bsdf_sample.wi);
                            if (!Intersect(&shadow_isect, Ray(isect.point, bsdf_sample.wi), Ray::epsilon, infinity))
                            {
                                Float p_light = 0;
                                Spectrum Li(0);
                                for (const Light* light : infinite_lights)
                                {
                                    p_light += light->EvaluatePDF_Li(shadow_ray) * light_sampler->EvaluatePMF(light);
                                    Li += light->Le(shadow_ray);
                                }

                                Float mis_denom = M_light * p_light + M_bsdf * p_bsdf;
                                if (mis_denom <= 0)
                                {
                                    continue;
                                }
                                Float w_mis = p_bsdf / mis_denom;

                                Spectrum f_cos = bsdf_sample.f * AbsDot(isect.shading.normal, bsdf_sample.wi);
                                Float p_hat = (Li * f_cos).Luminance();
                                if (p_hat > 0)
                                {
                                    ReSTIRDISample candidate;
                                    candidate.x = isect.point + bsdf_sample.wi * world_radius * 2;
                                    candidate.Li = Li;
                                    candidate.wi = bsdf_sample.wi;
                                    candidate.p_hat = p_hat;

                                    Float w = w_mis * p_hat / p_bsdf;

                                    reservoir.Add(candidate, w);
                                }
                            }
                            else if (const Light* light = GetAreaLight(shadow_isect); light)
                            {
                                Float p_light = shadow_isect.primitive->GetShape()->PDF(shadow_isect, shadow_ray) *
                                                light_sampler->EvaluatePMF(light);
                                Float mis_denom = M_light * p_light + M_bsdf * p_bsdf;
                                if (mis_denom <= 0)
                                {
                                    continue;
                                }
                                Float w_mis = p_bsdf / mis_denom;

                                Spectrum f_cos = bsdf_sample.f * AbsDot(isect.shading.normal, bsdf_sample.wi);
                                Spectrum Li = light->Le(shadow_isect, -bsdf_sample.wi);
                                Float p_hat = (Li * f_cos).Luminance();
                                if (p_hat > 0)
                                {
                                    ReSTIRDISample candidate;
                                    candidate.x = shadow_isect.point;
                                    candidate.Li = Li;
                                    candidate.wi = bsdf_sample.wi;

                                    candidate.p_hat = p_hat;
                                    Float w = w_mis * p_hat / p_bsdf;

                                    reservoir.Add(candidate, w);
                                }
                            }
                        }

                        ReSTIRDISample& sample = samples[index];
                        if (reservoir.HasSample())
                        {
                            sample = reservoir.y;

                            if (sample.p_hat > 0)
                            {
                                sample.W = (1 / reservoir.y.p_hat) * reservoir.w_sum;
                            }
                        }
                    }

                    progress->work_dones.fetch_add(1, std::memory_order_relaxed);
                },
                tile_size
            );

            // Visibility pass
            ParallelFor2D(
                resolution,
                [&](AABB2i tile) {
                    for (Point2i pixel : tile)
                    {
                        const int32 index = resolution.x * pixel.y + pixel.x;
                        ReSTIRDIVisiblePoint& vp = visible_points[index];
                        Intersection& isect = vp.isect;
                        ReSTIRDISample& sample = samples[index];
                        if (!isect.primitive)
                        {
                            continue;
                        }

                        if (!V(this, isect.point, sample.x))
                        {
                            sample.W = 0;
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
                        ReSTIRDIVisiblePoint& vp = visible_points[index];
                        Intersection& isect = vp.isect;

                        if (!isect.primitive)
                        {
                            progress->film.AddSample(pixel, vp.primary_weight * vp.Le);
                            continue;
                        }
                        ReSTIRDISample& sample = samples[index];

                        int8 bxdf_mem[max_bxdf_size];
                        BufferResource bsdf_buffer(bxdf_mem, sizeof(bxdf_mem));
                        Allocator bsdf_alloc(&bsdf_buffer);
                        BSDF bsdf;
                        if (!isect.GetBSDF(&bsdf, vp.wo, bsdf_alloc))
                        {
                            BulbitAssert(false);
                        }

                        Spectrum f_cos = bsdf.f(vp.wo, sample.wi) * AbsDot(isect.shading.normal, sample.wi);
                        Spectrum L = vp.Le + sample.Li * f_cos * sample.W;
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

} // namespace bulbit
