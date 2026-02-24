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

    Spectrum Le;
};

enum ReSTIRPTSampleFlag : uint8
{
    light_vertex = 1,
    bsdf_sampled = 2,
    light_sampled = 4,
    preceding_light_vertex = 8,
    mid_vertex = 16,
};

struct ReSTIRPTSample
{
    uint8 flag = 0;
    int32 reconnection_vertex = -1; // Reject this sample if reconnection vertex not set
    uint64 seed;                    // RNG seed for hybrid shift replay

    const Light* light = nullptr;

    Intersection isect;
    Vec3 wi = Vec3::zero;         // Incident direction to reconnection vertex
    Spectrum L = Spectrum::black; // Radiance estimate after reconnection vertex

    Float jacobian;               // Partial jacobian (1/p_w0) * da/dw * (1/p_w1)

    Spectrum contribution;        // Path contribution

    Float p_hat = 0;              // p_hat(contribution)
    Float W = 0;                  // UCW
};

struct ReSTIRPTReplay
{
    Intersection isect;
    Vec3 wo;
    Spectrum beta;
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

ReSTIRPTIntegrator::ReSTIRPTIntegrator(
    const Intersectable* accel,
    std::vector<Light*> lights,
    const Sampler* sampler,
    int32 max_bounces,
    int32 rr_min_bounces,
    Float spatial_radius,
    int32 spatial_samples
)
    : Integrator(accel, std::move(lights), std::make_unique<PowerLightSampler>())
    , sampler_prototype{ sampler }
    , max_bounces{ max_bounces }
    , rr_min_bounces{ rr_min_bounces }
    , spatial_radius{ std::max(0.0f, spatial_radius) }
{
    // Assume scene contains area lights only
    BulbitAssert(area_lights.Size() == all_lights.size());

    if (spatial_samples <= 0)
    {
        num_spatial_samples = std::min(2, int32(pi * Sqr(spatial_radius) * 0.05f));
    }
    else
    {
        num_spatial_samples = spatial_samples;
    }
}

Rendering* ReSTIRPTIntegrator::Render(Allocator& alloc, const Camera* camera)
{
    Point2i resolution = camera->GetScreenResolution();

    const int32 spp = sampler_prototype->samples_per_pixel;
    const int32 tile_size = 16;

    const int32 num_pixels = resolution.x * resolution.y;
    const Point2i num_tiles = (resolution + (tile_size - 1)) / tile_size;
    const int32 tile_count = num_tiles.x * num_tiles.y;
    const int32 num_passes = 3;
    const size_t total_works = size_t(std::max(spp, 1) * tile_count * num_passes);

    constexpr int32 earliest_reconnection_vertex = 2;

    SinglePhaseRendering* progress = alloc.new_object<SinglePhaseRendering>(camera, total_works);
    progress->job = RunAsync([=, this]() {
        for (int32 s = 0; s < spp; ++s)
        {
            std::vector<ReSTIRPTVisiblePoint> visible_points(num_pixels);

            std::vector<ReSTIRPTReservoir> base_reservoirs(num_pixels);
            std::vector<ReSTIRPTReservoir> spatial_reservoirs(num_pixels);

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
                        vp.primary_weight = primary_ray.weight;
                        vp.Le = Spectrum::black;

                        // Path sampling state
                        int32 bounce = 0;
                        Spectrum beta(1);
                        bool specular_bounce = false;
                        Float eta_scale = 1;
                        Ray ray = primary_ray.ray;
                        Float prev_bsdf_pdf = 0;
                        bool prev_diffuse = false;

                        // Reconnection vertex state
                        int32 reconnection_vertex = -1;
                        Intersection rc_isect;
                        Vec3 rc_wi;
                        Spectrum rc_beta(0);
                        Float rc_jacobian = 0.0f;

                        const uint64 seed = Hash(pixel, s);
                        RNG rng(seed);

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

                            int32 vertex_index = bounce + 1;

                            // landed on diffuse surface?
                            bool is_diffuse = IsDiffuse(bsdf.Flags());
                            if (reconnection_vertex < 0 && prev_diffuse && is_diffuse)
                            {
                                reconnection_vertex = vertex_index;
                            }

                            if (vertex_index == reconnection_vertex)
                            {
                                rc_isect = isect;
                                rc_beta = Spectrum(1);
                                rc_jacobian = (Sqr(isect.t) / AbsDot(isect.normal, wo)) / prev_bsdf_pdf;
                            }

                            const Light* area_light = GetAreaLight(isect);
                            while (area_light)
                            {
                                Spectrum Le = area_light->Le(isect, wo);
                                if (Le.IsBlack())
                                {
                                    break;
                                }

                                if (bounce == 0)
                                {
                                    vp.Le = beta * Le;
                                    break;
                                }

                                Spectrum L;
                                if (specular_bounce)
                                {
                                    L = Le;
                                }
                                else
                                {
                                    // Evaluate BSDF sample with MIS for area light
                                    Float light_pdf =
                                        isect.primitive->GetShape()->PDF(isect, ray) * light_sampler->EvaluatePMF(area_light);
                                    Float mis_weight = BalanceHeuristic(1, prev_bsdf_pdf, 1, light_pdf);

                                    L = mis_weight * Le;
                                }

                                // Add bsdf sampled path sample
                                ReSTIRPTSample sample;
                                sample.seed = seed;
                                if (reconnection_vertex > 0)
                                {
                                    sample.reconnection_vertex = reconnection_vertex;
                                    if (vertex_index == reconnection_vertex)
                                    {
                                        // Reconnection vertex is the light vertex
                                        sample.flag = bsdf_sampled | light_vertex;
                                        sample.light = area_light;
                                        sample.isect = isect;
                                        sample.wi = Vec3::zero;
                                        sample.L = Le;
                                        sample.jacobian = rc_jacobian;
                                    }
                                    else if (vertex_index == reconnection_vertex + 1)
                                    {
                                        // Reconnection vertex is previous vertex
                                        sample.flag = bsdf_sampled | preceding_light_vertex;
                                        sample.light = area_light;
                                        sample.isect = rc_isect;
                                        sample.wi = rc_wi;
                                        sample.L = Le;
                                        sample.jacobian = rc_jacobian;
                                    }
                                    else
                                    {
                                        // Reconnection vertex is set far before the light vertex
                                        sample.flag = mid_vertex;
                                        sample.isect = rc_isect;
                                        sample.wi = rc_wi;
                                        sample.L = rc_beta * L;
                                        sample.jacobian = rc_jacobian;
                                    }
                                }
                                else
                                {
                                    // Reconnection vertex is the light vertex
                                    sample.reconnection_vertex = prev_diffuse ? vertex_index : -1;
                                    sample.flag = bsdf_sampled | light_vertex;
                                    sample.light = area_light;
                                    sample.isect = isect;
                                    sample.wi = Vec3::zero;
                                    sample.L = Le;
                                    sample.jacobian = (Sqr(isect.t) / AbsDot(isect.normal, wo)) / prev_bsdf_pdf;
                                }

                                sample.contribution = beta * L;
                                sample.p_hat = sample.contribution.Luminance();
                                reservoir.Add(sample, sample.p_hat);
                                break;
                            }

                            // Mark current vertex as reconnection vertex
                            prev_diffuse = is_diffuse;

                            if (bounce++ >= max_bounces)
                            {
                                break;
                            }

                            Float u0 = rng.NextFloat();
                            Point2 u12 = { rng.NextFloat(), rng.NextFloat() };

                            // Do NEE
                            while (IsNonSpecular(bsdf.Flags()))
                            {
                                SampledLight sampled_light;
                                if (!light_sampler->Sample(&sampled_light, isect, u0))
                                {
                                    break;
                                }

                                LightSampleLi light_sample;
                                if (!sampled_light.light->Sample_Li(&light_sample, isect, u12))
                                {
                                    break;
                                }

                                Float bsdf_pdf = bsdf.PDF(wo, light_sample.wi);
                                if (light_sample.Li.IsBlack() || bsdf_pdf == 0)
                                {
                                    break;
                                }

                                Ray shadow_ray(isect.point, light_sample.wi);
                                if (IntersectAny(shadow_ray, Ray::epsilon, light_sample.visibility))
                                {
                                    break;
                                }

                                Float light_pdf = sampled_light.pmf * light_sample.pdf;
                                Spectrum f_cos = bsdf.f(wo, light_sample.wi) * AbsDot(isect.shading.normal, light_sample.wi);

                                Spectrum L;
                                if (sampled_light.light->IsDeltaLight())
                                {
                                    L = light_sample.Li / light_pdf;
                                }
                                else
                                {
                                    Float mis_weight = BalanceHeuristic(1, light_pdf, 1, bsdf_pdf);
                                    L = mis_weight * light_sample.Li / light_pdf;
                                }

                                // Add light sampled path sample
                                ReSTIRPTSample sample;
                                sample.seed = seed;
                                if (reconnection_vertex > 0)
                                {
                                    sample.reconnection_vertex = reconnection_vertex;
                                    if (vertex_index == reconnection_vertex)
                                    {
                                        // Reconnection vertex is preceding the light vertex
                                        sample.flag = light_sampled | preceding_light_vertex;
                                        sample.light = sampled_light.light;
                                        sample.isect = rc_isect;
                                        sample.wi = light_sample.wi;
                                        sample.L = light_sample.Li;
                                        sample.jacobian = rc_jacobian / light_pdf;
                                    }
                                    else
                                    {
                                        // Reconnection vertex is set far before the light vertex
                                        sample.flag = mid_vertex;
                                        sample.light = nullptr;
                                        sample.isect = rc_isect;
                                        sample.wi = rc_wi;
                                        sample.L = rc_beta * f_cos * L;
                                        sample.jacobian = rc_jacobian;
                                    }
                                }
                                else
                                {
                                    // Reconnection vertex is the light vertex
                                    sample.reconnection_vertex = is_diffuse ? (vertex_index + 1) : -1;
                                    sample.flag = light_sampled | light_vertex;
                                    sample.light = sampled_light.light;
                                    sample.isect.primitive = nullptr;
                                    sample.isect.point = light_sample.point;
                                    sample.isect.normal = light_sample.normal;
                                    sample.wi = Vec3::zero;
                                    sample.L = light_sample.Li;
                                    sample.jacobian =
                                        (Sqr(light_sample.visibility) / AbsDot(light_sample.normal, light_sample.wi)) / light_pdf;
                                }

                                sample.contribution = beta * f_cos * L;
                                sample.p_hat = sample.contribution.Luminance();
                                reservoir.Add(sample, sample.p_hat);
                                break;
                            }

                            Float u3 = rng.NextFloat();
                            Point2 u45 = { rng.NextFloat(), rng.NextFloat() };

                            BSDFSample bsdf_sample;
                            if (!bsdf.Sample_f(&bsdf_sample, wo, u3, u45))
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

                            if (vertex_index == reconnection_vertex)
                            {
                                rc_wi = bsdf_sample.wi;
                                rc_jacobian /= bsdf_sample.pdf;
                            }
                            else if (reconnection_vertex > 0 && vertex_index > reconnection_vertex)
                            {
                                rc_beta *= bsdf_sample.f * AbsDot(isect.shading.normal, bsdf_sample.wi) / bsdf_sample.pdf;
                            }

                            Float u6 = rng.NextFloat();

                            // Terminate path with russian roulette
                            if (bounce > rr_min_bounces)
                            {
                                if (Float p = beta.MaxComponent() * eta_scale; p < 1)
                                {
                                    if (u6 > p)
                                    {
                                        break;
                                    }
                                    else
                                    {
                                        beta /= p;
                                        rc_beta /= p;
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

            const auto replay_reconnection_prefix = [&](ReSTIRPTReplay* replay, const ReSTIRPTVisiblePoint& vp,
                                                        const ReSTIRPTSample& sample) -> bool {
                BulbitAssert(replay != nullptr);

                if (sample.reconnection_vertex < earliest_reconnection_vertex)
                {
                    return false;
                }

                // Start raytracing from visible point
                replay->isect = vp.isect;
                replay->wo = vp.wo;
                replay->beta = Spectrum(1);

                if (sample.reconnection_vertex == earliest_reconnection_vertex)
                {
                    return true;
                }

                RNG rng(sample.seed);

                bool prev_diffuse = false;
                Float eta_scale = 1;
                int32 bounce = 0;

                while (true)
                {
                    Intersection& isect = replay->isect;
                    Vec3& wo = replay->wo;

                    int8 mem[max_bxdf_size];
                    BufferResource res(mem, sizeof(mem));
                    Allocator alloc(&res);
                    BSDF bsdf;
                    if (!isect.GetBSDF(&bsdf, wo, alloc))
                    {
                        Ray ray(isect.point, -wo);
                        if (!Intersect(&isect, ray, Ray::epsilon, infinity))
                        {
                            return false;
                        }

                        replay->isect = isect;
                        replay->wo = Normalize(-ray.d);
                        continue;
                    }

                    const int32 vertex_index = bounce + 1;
                    const bool is_diffuse = IsDiffuse(bsdf.Flags());

                    // Reject if replayed path would choose an earlier reconnection vertex becaus it's non-invertable path
                    if (vertex_index < sample.reconnection_vertex && prev_diffuse && is_diffuse)
                    {
                        return false;
                    }

                    // Reached to prefix vertex
                    if (vertex_index == sample.reconnection_vertex - 1)
                    {
                        // Verify invertibility
                        if (vertex_index > 1 && is_diffuse)
                        {
                            replay->isect = isect;
                            replay->wo = wo;
                            return true;
                        }
                        else
                        {
                            return false;
                        }
                    }

                    prev_diffuse = is_diffuse;

                    if (bounce++ >= max_bounces)
                    {
                        return false;
                    }

                    // Consume NEE dimensions to match base path PSS traversal.
                    Float u0 = rng.NextFloat();
                    Point2 u12 = { rng.NextFloat(), rng.NextFloat() };
                    BulbitNotUsed(u0);
                    BulbitNotUsed(u12);

                    Float u3 = rng.NextFloat();
                    Point2 u45 = { rng.NextFloat(), rng.NextFloat() };

                    BSDFSample bsdf_sample;
                    if (!bsdf.Sample_f(&bsdf_sample, wo, u3, u45))
                    {
                        return false;
                    }

                    if (bsdf_sample.pdf == 0)
                    {
                        return false;
                    }

                    if (bsdf_sample.IsTransmission())
                    {
                        eta_scale *= Sqr(bsdf_sample.eta);
                    }

                    replay->beta *= bsdf_sample.f * AbsDot(isect.shading.normal, bsdf_sample.wi) / bsdf_sample.pdf;

                    Ray ray(isect.point, bsdf_sample.wi);
                    Intersection next_isect;
                    if (!Intersect(&next_isect, ray, Ray::epsilon, infinity))
                    {
                        return false;
                    }

                    Float u6 = rng.NextFloat();
                    if (bounce > rr_min_bounces)
                    {
                        if (Float p = replay->beta.MaxComponent() * eta_scale; p < 1)
                        {
                            if (u6 > p)
                            {
                                return false;
                            }

                            replay->beta /= p;
                        }
                    }

                    replay->isect = next_isect;
                    replay->wo = Normalize(-ray.d);
                }

                return false;
            };

            const auto shift_sample = [&](ReSTIRPTSample* shifted_sample, Float* shifted_jacobian,
                                          const ReSTIRPTVisiblePoint& target_vp, ReSTIRPTSample& source_sample) -> bool {
                int8 bxdf_mem[2 * max_bxdf_size];
                BufferResource bsdf_buffer(bxdf_mem, sizeof(bxdf_mem));
                Allocator bsdf_alloc(&bsdf_buffer);

                if (source_sample.W == 0 || source_sample.reconnection_vertex < earliest_reconnection_vertex)
                {
                    return false;
                }

                ReSTIRPTReplay replay;
                if (!replay_reconnection_prefix(&replay, target_vp, source_sample))
                {
                    return false;
                }

                // wo: y_{k-1} -> y_{k-2}
                // wi: y_{k-1} -> x_k
                // sample.wi: x_k -> x_{k+1}
                Vec3 wi = source_sample.isect.point - replay.isect.point;
                Float d2 = Length2(wi);
                Float d = std::sqrt(d2);
                wi /= d;

                BSDF bsdf;
                if (!replay.isect.GetBSDF(&bsdf, replay.wo, bsdf_alloc))
                {
                    return false;
                }

                Ray shadow_ray(replay.isect.point, wi);
                if (IntersectAny(shadow_ray, Ray::epsilon, d - Ray::epsilon))
                {
                    return false;
                }

                Float bsdf_pdf = bsdf.PDF(replay.wo, wi);
                if (bsdf_pdf == 0)
                {
                    return false;
                }

                Float jacobian = std::max(0.0f, (Dot(source_sample.isect.normal, -wi) / d2));
                if (jacobian == 0)
                {
                    return false;
                }

                Spectrum f_cos = bsdf.f(replay.wo, wi) * AbsDot(replay.isect.shading.normal, wi);

                ReSTIRPTSample shifted = source_sample;
                if (source_sample.flag & light_vertex)
                {
                    // Reconnection vertex is light vertex
                    BulbitAssert(source_sample.light != nullptr);

                    Float light_pdf =
                        light_sampler->EvaluatePMF(source_sample.light) * source_sample.light->EvaluatePDF_Li(shadow_ray);
                    if (light_pdf == 0)
                    {
                        return false;
                    }

                    jacobian *= source_sample.jacobian;
                    if (source_sample.flag & light_sampled)
                    {
                        Float mis_weight = source_sample.light->IsDeltaLight() ? 1 : BalanceHeuristic(1, light_pdf, 1, bsdf_pdf);

                        shifted.contribution = replay.beta * mis_weight * (f_cos / light_pdf) * source_sample.L;
                        jacobian *= light_pdf;
                    }
                    else
                    {
                        Float mis_weight = source_sample.light->IsDeltaLight() ? 1 : BalanceHeuristic(1, bsdf_pdf, 1, light_pdf);

                        shifted.contribution = replay.beta * mis_weight * (f_cos / bsdf_pdf) * source_sample.L;
                        jacobian *= bsdf_pdf;
                    }
                }
                else
                {
                    // Reconnection vertex is set before the light vertex
                    BSDF bsdf_rc;
                    if (!source_sample.isect.GetBSDF(&bsdf_rc, -wi, bsdf_alloc))
                    {
                        return false;
                    }

                    Spectrum f_cos_rc =
                        bsdf_rc.f(-wi, source_sample.wi) * AbsDot(source_sample.isect.shading.normal, source_sample.wi);
                    Float bsdf_pdf_rc = bsdf_rc.PDF(-wi, source_sample.wi);
                    if (bsdf_pdf_rc == 0)
                    {
                        return false;
                    }

                    jacobian *= bsdf_pdf * source_sample.jacobian;
                    if (source_sample.flag & preceding_light_vertex)
                    {
                        // Reconnection vertex is preceding the light vertex
                        BulbitAssert(source_sample.light != nullptr);

                        Float light_pdf_rc =
                            light_sampler->EvaluatePMF(source_sample.light) *
                            source_sample.light->EvaluatePDF_Li(Ray(source_sample.isect.point, source_sample.wi));

                        if (light_pdf_rc == 0)
                        {
                            return false;
                        }

                        if (source_sample.flag & light_sampled)
                        {
                            Float mis_weight =
                                source_sample.light->IsDeltaLight() ? 1 : BalanceHeuristic(1, light_pdf_rc, 1, bsdf_pdf_rc);

                            shifted.contribution =
                                replay.beta * (f_cos / bsdf_pdf) * mis_weight * (f_cos_rc / light_pdf_rc) * source_sample.L;
                            jacobian *= light_pdf_rc;
                        }
                        else
                        {
                            Float mis_weight =
                                source_sample.light->IsDeltaLight() ? 1 : BalanceHeuristic(1, bsdf_pdf_rc, 1, light_pdf_rc);

                            shifted.contribution =
                                replay.beta * (f_cos / bsdf_pdf) * mis_weight * (f_cos_rc / bsdf_pdf_rc) * source_sample.L;
                            jacobian *= bsdf_pdf_rc;
                        }
                    }
                    else
                    {
                        BulbitAssert((source_sample.flag & mid_vertex) == mid_vertex);

                        shifted.contribution = replay.beta * (f_cos / bsdf_pdf) * (f_cos_rc / bsdf_pdf_rc) * source_sample.L;
                        jacobian *= bsdf_pdf_rc;
                    }
                }

                Float p_hat = shifted.contribution.Luminance();
                if (p_hat <= 0)
                {
                    return false;
                }

                shifted.p_hat = p_hat;
                *shifted_sample = shifted;
                *shifted_jacobian = jacobian;
                return true;
            };

            // Spatial reuse
            ParallelFor2D(
                resolution,
                [&](AABB2i tile) {
                    std::vector<int32> neighbors(num_spatial_samples);

                    for (Point2i pixel : tile)
                    {
                        const int32 index = resolution.x * pixel.y + pixel.x;
                        ReSTIRPTVisiblePoint& vp = visible_points[index];
                        Intersection& isect = vp.isect;

                        if (!isect.primitive)
                        {
                            continue;
                        }

                        ReSTIRPTReservoir& base_reservoir = base_reservoirs[index];
                        ReSTIRPTReservoir& reservoir = spatial_reservoirs[index];
                        reservoir.Seed(Hash(pixel, s, 456));

                        ReSTIRPTSample canonical_sample = base_reservoir.y;
                        RNG rng(Hash(pixel, s), 789);

                        Float c_1 = 1;
                        Float c_total = num_spatial_samples;

                        int32 num_neighbors = 0;
                        for (int32 i = 0; i < num_spatial_samples - 1; ++i)
                        {
                            Point2 offset = spatial_radius * SampleUniformUnitDisk({ rng.NextFloat(), rng.NextFloat() });
                            Point2i neighbor_pixel(
                                Clamp(int32(pixel.x + offset.x), 0, resolution.x - 1),
                                Clamp(int32(pixel.y + offset.y), 0, resolution.y - 1)
                            );

                            int32 neighbor_index = resolution.x * neighbor_pixel.y + neighbor_pixel.x;
                            ReSTIRPTReservoir& neighbor_reservoir = base_reservoirs[neighbor_index];

                            if (neighbor_reservoir.y.W > 0)
                            {
                                neighbors[num_neighbors++] = neighbor_index;
                            }
                        }

                        Float m_1 = c_1 / c_total;
                        for (int32 i = 0; i < num_neighbors; ++i)
                        {
                            int32 neighbor_index = neighbors[i];
                            if (neighbor_index == index)
                            {
                                continue;
                            }

                            ReSTIRPTVisiblePoint& neighbor_vp = visible_points[neighbor_index];
                            ReSTIRPTReservoir& neighbor_reservoir = base_reservoirs[neighbor_index];
                            ReSTIRPTSample sample = neighbor_reservoir.y;

                            Float c_j = 1;

                            ReSTIRPTSample shifted_sample;
                            Float jacobian = 0;
                            if (shift_sample(&shifted_sample, &jacobian, vp, sample))
                            {
                                Float m_i = MIS_NonCanonical(c_1, c_total, c_j, sample.p_hat, shifted_sample.p_hat, jacobian);
                                if (m_i > 0)
                                {
                                    Float w = m_i * shifted_sample.p_hat * sample.W * jacobian;
                                    reservoir.Add(shifted_sample, w);
                                }
                            }

                            if (canonical_sample.W == 0)
                            {
                                continue;
                            }

                            ReSTIRPTSample shifted_canonical_sample;
                            Float jacobian_rev = 0;
                            shift_sample(&shifted_canonical_sample, &jacobian_rev, neighbor_vp, canonical_sample);
                            m_1 += MIS_Canonical(
                                c_1, c_total, c_j, canonical_sample.p_hat, shifted_canonical_sample.p_hat, jacobian_rev
                            );
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
                        const ReSTIRPTVisiblePoint& vp = visible_points[index];

                        if (!vp.isect.primitive)
                        {
                            progress->film.AddSample(pixel, Spectrum::black);
                            continue;
                        }

                        Spectrum L = vp.Le;
                        const ReSTIRPTSample& sample = spatial_reservoirs[index].y;
                        if (sample.W > 0)
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
