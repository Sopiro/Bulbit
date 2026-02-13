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
    BSDF bsdf;
    Vec3 wo;

    Spectrum Le;

    ReSTIRDIVisiblePoint()
        : bsdf_buffer(&bxdf_mem, sizeof(bxdf_mem))
    {
    }

    int8 bxdf_mem[max_bxdf_size];
    BufferResource bsdf_buffer;
};

struct ReSTIRDISample
{
    const Light* light;
    Point3 x, n;

    Float d2, cos;
    Vec3 wi;
    Spectrum Li;

    Float p_hat = 0; // p_hat(y): target function value of selected sample
    Float W = 0;     // UCW
};

class ReSTIRDIReservoir
{
public:
    ReSTIRDIReservoir(uint64 seed = 0)
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

    bool Add(const ReSTIRDISample& sample, Float weight)
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

    Float GetSampleProbability() const
    {
        return w / w_sum;
    }

    void Reset()
    {
        w = 0;
        w_sum = 0;
        M = 0;
    }

    ReSTIRDISample y{};
    Float w;
    Float w_sum;
    Float M;

private:
    RNG rng;
};

inline Float MIS_Canonical(
    Float c_1,
    Float c_total,
    Float c_j,
    Float p_hat_x1, // r1.p_hat(x) where x = X1
    Float p_hat_y,  // p_hat(y) in neighbor domain (from inverse shift)
    Float jacobian  // |(T^{-1}_j)'(X1)|
)
{
    Float w = c_1 * p_hat_x1;
    Float denom = w + (c_total - c_1) * p_hat_y * jacobian;
    if (denom <= 0)
    {
        return 0;
    }

    return (c_j / c_total) * (w / denom);
}

inline Float MIS_NonCanonical(
    Float c_1,
    Float c_total,
    Float c_j,
    Float p_hat_xj, // rj.p_hat(x) where x = X_j
    Float p_hat_y,  // p_hat(y) in canonical domain
    Float jacobian  // |T'_j(X_j)|
)
{
    Float w = (c_total - c_1) * p_hat_xj;
    Float denom = w + c_1 * p_hat_y * jacobian;
    if (denom <= 0)
    {
        return 0;
    }

    return (c_j / c_total) * (w / denom);
}

ReSTIRDIIntegrator::ReSTIRDIIntegrator(const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler)
    : Integrator(accel, std::move(lights), std::make_unique<UniformLightSampler>())
    , sampler_prototype{ sampler }
{
    // Assume scene contains area lights only
    BulbitAssert(area_lights.Size() == all_lights.size());
}

Rendering* ReSTIRDIIntegrator::Render(Allocator& alloc, const Camera* camera)
{
    Point2i resolution = camera->GetScreenResolution();

    const int32 spp = sampler_prototype->samples_per_pixel;
    const int32 tile_size = 16;

    const int32 num_pixels = resolution.x * resolution.y;
    const Point2i num_tiles = (resolution + (tile_size - 1)) / tile_size;
    const int32 tile_count = num_tiles.x * num_tiles.y;
    const int32 num_passes = 4;
    const size_t total_works = size_t(std::max(spp, 1) * tile_count * num_passes);

    const Float spatial_radius = 3.0f;
    const int32 num_spatial_samples = 5;

    const int32 M_light = 1;
    const int32 M_bsdf = 1;

    const bool include_visibility = true;

    SinglePhaseRendering* progress = alloc.new_object<SinglePhaseRendering>(camera, total_works);
    progress->job = RunAsync([=, this]() {
        for (int32 s = 0; s < spp; ++s)
        {
            std::vector<ReSTIRDIVisiblePoint> visible_points(num_pixels);

            std::vector<ReSTIRDIReservoir> ris_reservoirs(num_pixels);     // output sample after RIS sampling + visibility pass
            std::vector<ReSTIRDIReservoir> spatial_reservoirs(num_pixels); // output sample after spatial resampling

            // Trace primary rays and generate initial sample with RIS
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

                        PrimaryRay primary_ray;
                        camera->SampleRay(&primary_ray, pixel, sampler->Next2D(), sampler->Next2D());

                        Ray ray = primary_ray.ray;

                        ReSTIRDIVisiblePoint& vp = visible_points[index];
                        Intersection& isect = vp.isect;

                        vp.primary_weight = primary_ray.weight;
                        vp.wo = Normalize(-ray.d);

                        BSDF& bsdf = vp.bsdf;
                        bool found_intersection = false;
                        while (true)
                        {
                            // Find visible point
                            found_intersection = Intersect(&isect, ray, Ray::epsilon, infinity);
                            if (!found_intersection)
                            {
                                isect.primitive = nullptr;
                                break;
                            }

                            if (const Light* light = GetAreaLight(isect); light)
                            {
                                if (Spectrum Le = light->Le(isect, -ray.d); !Le.IsBlack())
                                {
                                    vp.Le = Le;
                                }
                            }

                            Allocator bsdf_alloc(&vp.bsdf_buffer);
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

                        ReSTIRDIReservoir& reservoir = ris_reservoirs[index];
                        reservoir.Seed(Hash(pixel, s, 123));

                        // RIS: draw M initial candidates, keep one by WRS, then compute W(y)
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

                            // solid angle domain pdfs
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
                            if (p_hat <= 0)
                            {
                                continue;
                            }

                            // Include visibility for light samples
                            if (include_visibility && !V(this, isect.point, light_sample.point))
                            {
                                continue;
                            }

                            ReSTIRDISample sample;
                            sample.light = sampled_light.light;
                            sample.x = light_sample.point;
                            sample.n = light_sample.normal;

                            sample.cos = AbsDot(light_sample.normal, light_sample.wi);
                            sample.d2 = Dist2(light_sample.point, isect.point);
                            sample.wi = light_sample.wi;
                            sample.Li = light_sample.Li;

                            sample.p_hat = p_hat;

                            Float w = w_mis * p_hat / p_light;

                            reservoir.Add(sample, w);
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
                            if (Intersect(&shadow_isect, Ray(isect.point, bsdf_sample.wi), Ray::epsilon, infinity))
                            {
                                if (const Light* light = GetAreaLight(shadow_isect); light)
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
                                    if (p_hat <= 0)
                                    {
                                        continue;
                                    }

                                    ReSTIRDISample sample;
                                    sample.light = light;
                                    sample.x = shadow_isect.point;
                                    sample.n = shadow_isect.normal;

                                    sample.d2 = Sqr(shadow_isect.t);
                                    sample.cos = AbsDot(shadow_isect.normal, -bsdf_sample.wi);
                                    sample.wi = bsdf_sample.wi;
                                    sample.Li = Li;

                                    sample.p_hat = p_hat;
                                    Float w = w_mis * p_hat / p_bsdf;

                                    reservoir.Add(sample, w);
                                }
                            }
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

            // Visibility pass
            if (!include_visibility)
            {
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
                                continue;
                            }

                            ReSTIRDISample& sample = ris_reservoirs[index].y;
                            if (sample.W > 0 && !V(this, isect.point, sample.x))
                            {
                                sample.W = 0;
                            }
                        }

                        progress->work_dones.fetch_add(1, std::memory_order_relaxed);
                    },
                    tile_size
                );
            }

            // Spatial resampling
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
                            continue;
                        }

                        ReSTIRDIReservoir& ris_reservoir = ris_reservoirs[index];
                        ReSTIRDIReservoir& reservoir = spatial_reservoirs[index];
                        reservoir.Seed(Hash(pixel, s, 456));

                        ReSTIRDISample canonical_sample = ris_reservoir.y;

                        RNG rng(Hash(pixel, s), 789);

                        // Pairwise MIS weight for canonical sample
                        Float c_1 = canonical_sample.W > 0 ? ris_reservoir.M : 0;
                        Float c_total = c_1;

                        int32 neighbors[num_spatial_samples - 1];
                        for (int32 i = 0; i < num_spatial_samples - 1; ++i)
                        {
                            Point2 offset = spatial_radius * SampleUniformUnitDisk({ rng.NextFloat(), rng.NextFloat() });
                            Point2i neighbor_pixel(
                                Clamp(int32(pixel.x + offset.x), 0, resolution.x - 1),
                                Clamp(int32(pixel.y + offset.y), 0, resolution.y - 1)
                            );
                            int32 neighbor_index = resolution.x * neighbor_pixel.y + neighbor_pixel.x;

                            ReSTIRDIReservoir& neighbor_reservoir = ris_reservoirs[neighbor_index];
                            if (neighbor_reservoir.y.W > 0)
                            {
                                neighbors[i] = neighbor_index;
                            }
                            else
                            {
                                neighbors[i] = -1;
                            }

                            c_total += neighbor_reservoir.M;
                        }
                        Float m_1 = c_1 / c_total;

                        for (int32 i = 0; i < num_spatial_samples - 1; ++i)
                        {
                            int32 neighbor_index = neighbors[i];
                            if (neighbor_index < 0)
                            {
                                continue;
                            }

                            ReSTIRDIVisiblePoint& neighbor_vp = visible_points[neighbor_index];
                            ReSTIRDIReservoir& neighbor_reservoir = ris_reservoirs[neighbor_index];
                            ReSTIRDISample sample = neighbor_reservoir.y;

                            if (neighbor_index == index)
                            {
                                continue;
                            }

                            Vec3 wi = sample.x - isect.point;
                            Float d2 = Length2(wi);
                            wi /= std::sqrt(d2);

                            // Shift neighbor sample to canonical domain
                            Spectrum Li =
                                sample.light->Le(Intersection{ .point = sample.x, .front_face = Dot(sample.n, wi) < 0 }, -wi);
                            Spectrum f_cos = vp.bsdf.f(vp.wo, wi) * AbsDot(isect.shading.normal, wi);

                            Float p_hat_y = (Li * f_cos).Luminance();
                            if (p_hat_y <= 0)
                            {
                                continue;
                            }

                            Float c_j = neighbor_reservoir.M;

                            Float jacobian = std::max(0.0f, (Dot(sample.n, -wi) * sample.d2) / (sample.cos * d2));
                            Float m_i = MIS_NonCanonical(c_1, c_total, c_j, sample.p_hat, p_hat_y, jacobian);

                            if (m_i > 0)
                            {
                                Float w = m_i * p_hat_y * sample.W * jacobian;
                                sample.p_hat = p_hat_y;
                                sample.wi = wi;
                                sample.Li = Li;
                                reservoir.Add(sample, w);
                            }

                            if (canonical_sample.W == 0)
                            {
                                continue;
                            }

                            // Shift canonical sample to neighbor domain
                            wi = canonical_sample.x - neighbor_vp.isect.point;
                            d2 = Length2(wi);
                            wi /= std::sqrt(d2);

                            Li = canonical_sample.light->Le(
                                Intersection{ .point = canonical_sample.x, .front_face = Dot(canonical_sample.n, wi) < 0 }, -wi
                            );
                            f_cos = neighbor_vp.bsdf.f(neighbor_vp.wo, wi) * AbsDot(neighbor_vp.isect.shading.normal, wi);

                            p_hat_y = (Li * f_cos).Luminance();
                            Float jacobian_rev = std::max(
                                0.0f, (Dot(canonical_sample.n, -wi) * canonical_sample.d2) / (canonical_sample.cos * d2)
                            );
                            m_1 += MIS_Canonical(c_1, c_total, c_j, canonical_sample.p_hat, p_hat_y, jacobian_rev);
                        }

                        if (canonical_sample.W > 0 && m_1 > 0)
                        {
                            Float w = m_1 * canonical_sample.p_hat * canonical_sample.W;
                            reservoir.Add(canonical_sample, w);
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
                        ReSTIRDIVisiblePoint& vp = visible_points[index];
                        Intersection& isect = vp.isect;

                        if (!isect.primitive)
                        {
                            progress->film.AddSample(pixel, Spectrum::black);
                            continue;
                        }
                        ReSTIRDISample& sample = spatial_reservoirs[index].y;

                        Spectrum L = vp.Le;
                        if (sample.W > 0 && V(this, isect.point, sample.x))
                        {
                            Spectrum f_cos = vp.bsdf.f(vp.wo, sample.wi) * AbsDot(isect.shading.normal, sample.wi);
                            L += sample.Li * f_cos * sample.W;
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

} // namespace bulbit
