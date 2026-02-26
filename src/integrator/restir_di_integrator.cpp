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
    bool is_infinite_light = false;
    Point3 x, n;

    // Partial jacobian for shift mapping density conversion
    // Area lights: da/dw, Infinite lights: 1 (already in solid-angle measure)
    Float jacobian;
    Vec3 wi;
    Spectrum Li;

    Spectrum contribution;
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

inline bool TestRejection(const ReSTIRDIVisiblePoint& canonical, const ReSTIRDIVisiblePoint& neighbor)
{
    if (!neighbor.isect.primitive)
    {
        return false;
    }

    const Float cosine = std::cos(RadToDeg(50));
    const Float depth = 0.5f;

    // Test normal similarity
    if (Dot(canonical.isect.shading.normal, neighbor.isect.shading.normal) < cosine)
    {
        return false;
    }

    // Test depth similarity
    if (Abs(canonical.isect.t - neighbor.isect.t) > depth)
    {
        return false;
    }

    return true;
};

ReSTIRDIIntegrator::ReSTIRDIIntegrator(
    const Intersectable* accel,
    std::vector<Light*> lights,
    const Sampler* sampler,
    Float spatial_radius,
    int32 spatial_samples,
    int32 M_light,
    int32 M_bsdf,
    bool include_visibility
)
    : Integrator(accel, std::move(lights), std::make_unique<UniformLightSampler>())
    , sampler_prototype{ sampler }
    , spatial_radius{ std::max(0.0f, spatial_radius) }
    , num_spatial_samples{ std::max(1, spatial_samples) }
    , M_light{ std::max(0, M_light) }
    , M_bsdf{ std::max(0, M_bsdf) }
    , include_visibility{ include_visibility }
{
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
                        vp.Le = Spectrum::black;

                        BSDF& bsdf = vp.bsdf;
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
                            Spectrum contribution = light_sample.Li * f_cos;
                            Float p_hat = contribution.Luminance();
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
                            sample.is_infinite_light = sampled_light.light->IsInfiniteLight();
                            sample.x = light_sample.point;
                            sample.n = light_sample.normal;

                            if (sample.is_infinite_light)
                            {
                                sample.jacobian = 1;
                            }
                            else
                            {
                                sample.jacobian =
                                    Dist2(light_sample.point, isect.point) / AbsDot(light_sample.normal, light_sample.wi);
                            }
                            sample.wi = light_sample.wi;
                            sample.Li = light_sample.Li;
                            sample.contribution = contribution;

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
                                    Spectrum contribution = Li * f_cos;
                                    Float p_hat = contribution.Luminance();
                                    if (p_hat <= 0)
                                    {
                                        continue;
                                    }

                                    ReSTIRDISample sample;
                                    sample.light = light;
                                    sample.is_infinite_light = false;
                                    sample.x = shadow_isect.point;
                                    sample.n = shadow_isect.normal;

                                    sample.jacobian = Sqr(shadow_isect.t) / AbsDot(shadow_isect.normal, -bsdf_sample.wi);
                                    sample.wi = bsdf_sample.wi;
                                    sample.Li = Li;
                                    sample.contribution = contribution;

                                    sample.p_hat = p_hat;
                                    Float w = w_mis * p_hat / p_bsdf;

                                    reservoir.Add(sample, w);
                                }
                            }
                            else
                            {
                                for (Light* light : infinite_lights)
                                {
                                    Float p_light = light_sampler->EvaluatePMF(light) * light->EvaluatePDF_Li(shadow_ray);
                                    Float mis_denom = M_light * p_light + M_bsdf * p_bsdf;
                                    if (mis_denom <= 0)
                                    {
                                        continue;
                                    }
                                    Float w_mis = p_bsdf / mis_denom;

                                    Spectrum f_cos = bsdf_sample.f * AbsDot(isect.shading.normal, bsdf_sample.wi);
                                    Spectrum Li = light->Le(shadow_ray);
                                    Spectrum contribution = Li * f_cos;
                                    Float p_hat = contribution.Luminance();
                                    if (p_hat <= 0)
                                    {
                                        continue;
                                    }

                                    ReSTIRDISample sample;
                                    sample.light = light;
                                    sample.is_infinite_light = true;
                                    sample.x = isect.point + bsdf_sample.wi;
                                    sample.n = Point3(0);
                                    sample.jacobian = 1;
                                    sample.wi = bsdf_sample.wi;
                                    sample.Li = Li;
                                    sample.contribution = contribution;

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

            const auto test_visibility = [&](const Intersection& isect, const ReSTIRDISample& sample) -> bool {
                if (!sample.is_infinite_light)
                {
                    return V(this, isect.point, sample.x);
                }

                Ray ray(isect.point, sample.wi);
                Intersection shadow_isect;
                while (Intersect(&shadow_isect, ray, Ray::epsilon, infinity))
                {
                    if (shadow_isect.primitive->GetMaterial())
                    {
                        return false;
                    }

                    ray.o = shadow_isect.point;
                }

                return true;
            };

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
                            if (sample.W > 0 && !test_visibility(isect, sample))
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
                    std::vector<int32> neighbors(num_spatial_samples);

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
                        Float c_1 = ris_reservoir.M;
                        Float c_total = c_1;

                        int32 num_neighbors = 0;
                        for (int32 i = 0; i < num_spatial_samples - 1; ++i)
                        {
                            Point2 offset = spatial_radius * SampleUniformUnitDisk({ rng.NextFloat(), rng.NextFloat() });
                            Point2i neighbor_pixel(int32(pixel.x + offset.x), int32(pixel.y + offset.y));
                            if (neighbor_pixel.x < 0 || neighbor_pixel.x >= resolution.x || neighbor_pixel.y < 0 ||
                                neighbor_pixel.y >= resolution.y)
                            {
                                continue;
                            }

                            int32 neighbor_index = resolution.x * neighbor_pixel.y + neighbor_pixel.x;

                            ReSTIRDIReservoir& neighbor_reservoir = ris_reservoirs[neighbor_index];
                            if (!TestRejection(vp, visible_points[neighbor_index]))
                            {
                                continue;
                            }

                            if (neighbor_reservoir.y.W > 0)
                            {
                                neighbors[num_neighbors++] = neighbor_index;
                            }

                            c_total += neighbor_reservoir.M;
                        }

                        Float m_1 = c_1 / c_total;

                        for (int32 i = 0; i < num_neighbors; ++i)
                        {
                            int32 neighbor_index = neighbors[i];
                            if (neighbor_index == index)
                            {
                                continue;
                            }

                            ReSTIRDIVisiblePoint& neighbor_vp = visible_points[neighbor_index];
                            ReSTIRDIReservoir& neighbor_reservoir = ris_reservoirs[neighbor_index];
                            ReSTIRDISample sample = neighbor_reservoir.y;

                            Vec3 wi;
                            Float d2 = 0;
                            Spectrum Li;
                            Float jacobian = sample.jacobian;
                            if (sample.is_infinite_light)
                            {
                                wi = sample.wi;
                                Li = sample.light->Le(Ray(isect.point, wi));
                            }
                            else
                            {
                                wi = sample.x - isect.point;
                                d2 = Length2(wi);
                                wi /= std::sqrt(d2);

                                Li =
                                    sample.light->Le(Intersection{ .point = sample.x, .front_face = Dot(sample.n, wi) < 0 }, -wi);
                                jacobian = std::max(0.0f, (Dot(sample.n, -wi) / d2) * sample.jacobian);
                            }

                            // Shift neighbor sample to canonical domain
                            Spectrum f_cos = vp.bsdf.f(vp.wo, wi) * AbsDot(isect.shading.normal, wi);

                            Spectrum contribution = Li * f_cos;
                            Float p_hat_y = contribution.Luminance();
                            Float c_j = neighbor_reservoir.M;

                            Float m_i = MIS_NonCanonical(c_1, c_total, c_j, sample.p_hat, p_hat_y, jacobian);

                            if (m_i > 0)
                            {
                                Float w = m_i * p_hat_y * sample.W * jacobian;
                                sample.p_hat = p_hat_y;
                                sample.wi = wi;
                                sample.Li = Li;
                                sample.contribution = contribution;
                                reservoir.Add(sample, w);
                            }

                            if (canonical_sample.W == 0)
                            {
                                continue;
                            }

                            // Shift canonical sample to neighbor domain
                            Float jacobian_rev = canonical_sample.jacobian;
                            if (canonical_sample.is_infinite_light)
                            {
                                wi = canonical_sample.wi;
                                Li = canonical_sample.light->Le(Ray(neighbor_vp.isect.point, wi));
                            }
                            else
                            {
                                wi = canonical_sample.x - neighbor_vp.isect.point;
                                d2 = Length2(wi);
                                wi /= std::sqrt(d2);

                                Li = canonical_sample.light->Le(
                                    Intersection{ .point = canonical_sample.x, .front_face = Dot(canonical_sample.n, wi) < 0 },
                                    -wi
                                );
                                jacobian_rev = std::max(0.0f, (Dot(canonical_sample.n, -wi) / d2) * canonical_sample.jacobian);
                            }

                            f_cos = neighbor_vp.bsdf.f(neighbor_vp.wo, wi) * AbsDot(neighbor_vp.isect.shading.normal, wi);

                            p_hat_y = (Li * f_cos).Luminance();
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
                            progress->film.AddSample(pixel, vp.primary_weight * vp.Le);
                            continue;
                        }

                        ReSTIRDISample& sample = spatial_reservoirs[index].y;

                        Spectrum L = vp.Le;
                        if (sample.W > 0 && test_visibility(isect, sample))
                        {
                            L += sample.contribution * sample.W;
                        }

                        if (!L.IsNullish())
                        {
                            progress->film.AddSample(pixel, vp.primary_weight * L);
                        }
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
