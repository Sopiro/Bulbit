#include "bulbit/async_job.h"
#include "bulbit/bsdf.h"
#include "bulbit/bxdfs.h"
#include "bulbit/camera.h"
#include "bulbit/film.h"
#include "bulbit/integrators.h"
#include "bulbit/parallel_for.h"
#include "bulbit/progresses.h"
#include "bulbit/sampler.h"
#include "bulbit/visibility.h"

namespace bulbit
{

struct VCMSubPathState
{
    Point3 origin;
    Vec3 direction;
    Spectrum beta;

    int32 path_length : 30 = 1;
    uint32 is_finite_light : 1 = false;
    uint32 specular_path : 1 = true;

    Float eta_scale = 1;

    // Cache variables for efficient subpath MIS weight evaluation (O(1) per vc/vm)
    // See below for details..
    // http://www.iliyan.com/publications/ImplementingVCM
    Float d_vcm = 0; // MIS cache used for vertex connection and merging
    Float d_vc = 0;  // MIS cache used for vertex connection
    Float d_vm = 0;  // MIS cache used for vertex merging

    // A naive approach for computing MIS weights is to evaluate Eq. (12) per path, which requires explicitly evaluating all
    // path densities. VCM instead rewrites the MIS weights into a recursive form (Eqs. (24)–(28)) so the required PDF
    // ratios can be accumulated incrementally along the path. During the connection/merging step, the reverse density is computed
    // by factoring it into a solid-angle density term and a geometry (measure-conversion) term (Eqs. (29) and (30)), which
    // postpones explicit reverse PDF evaluation until the final measure-conversion factor is applied.
};

struct VCMLightVertex
{
    Point3 p;
    Vec3 wo;
    Vec3 shading_normal;

    Spectrum beta;
    BSDF bsdf;

    int32 path_length = 0;
    Float d_vcm = 0;
    Float d_vc = 0;
    Float d_vm = 0;
    Float cont_prob = 1;
};

inline Float Mis(Float pdf)
{
    // Balance heuristic
    return pdf;
}

inline Float ComputeContinuationProb(const VCMSubPathState& state, int32 rr_min_bounces)
{
    if (state.path_length <= rr_min_bounces)
    {
        return 1;
    }

    return std::min<Float>(state.beta.MaxComponent() * state.eta_scale, 1);
}

bool SampleScattering(
    VCMSubPathState* state,
    const Intersection& isect,
    const BSDF& bsdf,
    const Vec3& wo,
    TransportDirection direction,
    Float cont_prob,
    Float mis_vm_weight,
    Float mis_vc_weight,
    Sampler& sampler
)
{
    BSDFSample bsdf_sample;
    if (!bsdf.Sample_f(&bsdf_sample, wo, sampler.Next1D(), sampler.Next2D(), direction))
    {
        return false;
    }

    if (cont_prob == 0)
    {
        return false;
    }
    if (cont_prob < 1 && sampler.Next1D() > cont_prob)
    {
        return false;
    }

    Float cos_theta_out = AbsDot(isect.shading.normal, bsdf_sample.wi);
    if (cos_theta_out == 0)
    {
        return false;
    }

    if (bsdf_sample.IsTransmission())
    {
        state->eta_scale *= Sqr(bsdf_sample.eta);
    }

    // Throughput update uses the actual sampling pdf; MIS uses the full strategy pdf.
    state->beta *= bsdf_sample.f * (cos_theta_out / (bsdf_sample.pdf * cont_prob));

    Float bsdf_dir_pdf_w;
    Float bsdf_rev_pdf_w;
    if (bsdf_sample.IsSpecular())
    {
        bsdf_dir_pdf_w = bsdf_sample.pdf;
        bsdf_rev_pdf_w = bsdf_sample.pdf;
    }
    else
    {
        bsdf_dir_pdf_w = bsdf_sample.is_stochastic ? bsdf.PDF(wo, bsdf_sample.wi, direction) : bsdf_sample.pdf;
        bsdf_rev_pdf_w = bsdf.PDF(bsdf_sample.wi, wo, !direction);
    }

    bsdf_dir_pdf_w *= cont_prob;
    bsdf_rev_pdf_w *= cont_prob;

    if (bsdf_dir_pdf_w == 0)
    {
        return false;
    }

    if (bsdf_sample.IsSpecular())
    {
        // Specular scattering update
        state->d_vcm = 0;
        state->d_vc *= Mis(cos_theta_out);
        state->d_vm *= Mis(cos_theta_out);
    }
    else
    {
        // Non-specular scattering update
        Float factor = Mis(cos_theta_out / bsdf_dir_pdf_w);
        state->d_vc = factor * (state->d_vc * Mis(bsdf_rev_pdf_w) + state->d_vcm + mis_vm_weight);
        state->d_vm = factor * (state->d_vm * Mis(bsdf_rev_pdf_w) + state->d_vcm * mis_vc_weight + 1);
        state->d_vcm = Mis(1 / bsdf_dir_pdf_w);

        state->specular_path = false;
    }

    if (state->beta.IsBlack())
    {
        return false;
    }

    state->origin = isect.point;
    state->direction = bsdf_sample.wi;
    ++state->path_length;
    return true;
}

Float EmissionPDFW(const Light* light, const Point3& light_point, const Vec3& light_normal, const Vec3& w_to_prev)
{
    Float pdf_p, pdf_w;

    if (!light->IsDeltaLight() && !light->IsInfiniteLight())
    {
        Intersection isect;
        isect.point = light_point;
        isect.normal = light_normal;
        light->PDF_Le(&pdf_p, &pdf_w, isect, w_to_prev);
    }
    else
    {
        light->EvaluatePDF_Le(&pdf_p, &pdf_w, Ray(light_point, w_to_prev));
    }

    BulbitNotUsed(pdf_p);
    return pdf_w;
}

Spectrum AreaLightLe(
    const Integrator* I,
    const Light* area_light,
    const Intersection& isect,
    const Vec3& wo,
    const Point3& prev_point,
    const VCMSubPathState& camera_state
)
{
    Spectrum radiance = area_light->Le(isect, wo);
    if (radiance.IsBlack())
    {
        return Spectrum::black;
    }

    if (camera_state.path_length == 1)
    {
        return radiance;
    }

    Float dist2 = Sqr(isect.t);
    Float cos_at_light = AbsDot(isect.normal, wo);
    if (dist2 == 0 || cos_at_light == 0)
    {
        return Spectrum::black;
    }

    Float light_pmf = I->GetLightSampler()->EvaluatePMF(area_light);
    Float direct_pdf_w = isect.primitive->GetShape()->PDF(isect, Ray(prev_point, -wo));

    Float direct_pdf_a = light_pmf * direct_pdf_w * (cos_at_light / dist2);
    Float emission_pdf_w = light_pmf * EmissionPDFW(area_light, isect.point, isect.normal, wo);

    Float w_camera = Mis(direct_pdf_a) * camera_state.d_vcm + Mis(emission_pdf_w) * camera_state.d_vc;
    Float mis_weight = 1 / (1 + w_camera);

    return mis_weight * radiance;
}

Spectrum DirectIllumination(
    const Integrator* I,
    const VCMSubPathState& camera_state,
    const Intersection& isect,
    const Vec3& wo,
    const BSDF& bsdf,
    Float camera_cont_prob,
    Float mis_vm_weight,
    Sampler& sampler
)
{
    SampledLight sampled_light;
    if (!I->GetLightSampler()->Sample(&sampled_light, isect, sampler.Next1D()))
    {
        return Spectrum::black;
    }

    LightSampleLi light_sample;
    if (!sampled_light.light->Sample_Li(&light_sample, isect, sampler.Next2D()))
    {
        return Spectrum::black;
    }

    Vec3 wi = light_sample.wi;
    Float cos_to_light = AbsDot(isect.shading.normal, wi);
    if (cos_to_light == 0)
    {
        return Spectrum::black;
    }

    Spectrum f_cos = bsdf.f(wo, wi, TransportDirection::ToLight) * cos_to_light;
    if (f_cos.IsBlack())
    {
        return Spectrum::black;
    }

    Float direct_pdf_w = sampled_light.pmf * light_sample.pdf;
    if (direct_pdf_w == 0)
    {
        return Spectrum::black;
    }

    Float bsdf_dir_pdf_w =
        sampled_light.light->IsDeltaLight() ? 0 : bsdf.PDF(wo, wi, TransportDirection::ToLight) * camera_cont_prob;
    Float bsdf_rev_pdf_w = bsdf.PDF(wi, wo, TransportDirection::ToCamera) * camera_cont_prob;

    Float emission_pdf_w = sampled_light.pmf * EmissionPDFW(sampled_light.light, light_sample.point, light_sample.normal, -wi);

    Float cos_at_light = (light_sample.normal != Vec3::zero) ? AbsDot(light_sample.normal, -wi) : 1;
    if (cos_at_light == 0)
    {
        return Spectrum::black;
    }

    if (!V(I, isect.point, light_sample.point))
    {
        return Spectrum::black;
    }

    Float w_light = Mis(bsdf_dir_pdf_w / direct_pdf_w);

    Float ratio = (emission_pdf_w * cos_to_light) / (direct_pdf_w * cos_at_light);
    Float w_camera = Mis(ratio) * (mis_vm_weight + camera_state.d_vcm + camera_state.d_vc * Mis(bsdf_rev_pdf_w));

    Float mis_weight = 1 / (w_light + 1 + w_camera);

    return mis_weight * light_sample.Li * f_cos / direct_pdf_w;
}

Spectrum ConnectVertices(
    const Integrator* I,
    const VCMLightVertex& light_vertex,
    const BSDF& light_bsdf,
    const VCMSubPathState& camera_state,
    const Intersection& camera_isect,
    const Vec3& camera_wo,
    const BSDF& camera_bsdf,
    Float camera_cont_prob,
    Float mis_vm_weight
)
{
    Vec3 wi = light_vertex.p - camera_isect.point;
    Float dist2 = Length2(wi);
    if (dist2 == 0)
    {
        return Spectrum::black;
    }

    Float distance = std::sqrt(dist2);
    wi /= distance;

    Float cos_camera = AbsDot(camera_isect.shading.normal, wi);
    if (cos_camera == 0)
    {
        return Spectrum::black;
    }

    Float cos_light = AbsDot(light_vertex.shading_normal, -wi);
    if (cos_light == 0)
    {
        return Spectrum::black;
    }

    Spectrum camera_f_cos = camera_bsdf.f(camera_wo, wi, TransportDirection::ToLight) * cos_camera;
    if (camera_f_cos.IsBlack())
    {
        return Spectrum::black;
    }

    Spectrum light_f_cos = light_bsdf.f(light_vertex.wo, -wi, TransportDirection::ToCamera) * cos_light;
    if (light_f_cos.IsBlack())
    {
        return Spectrum::black;
    }

    if (!V(I, camera_isect.point, light_vertex.p))
    {
        return Spectrum::black;
    }

    Float camera_bsdf_dir_pdf_w = camera_bsdf.PDF(camera_wo, wi, TransportDirection::ToLight) * camera_cont_prob;
    Float camera_bsdf_rev_pdf_w = camera_bsdf.PDF(wi, camera_wo, TransportDirection::ToCamera) * camera_cont_prob;

    Float light_bsdf_dir_pdf_w = light_bsdf.PDF(light_vertex.wo, -wi, TransportDirection::ToCamera) * light_vertex.cont_prob;
    Float light_bsdf_rev_pdf_w = light_bsdf.PDF(-wi, light_vertex.wo, TransportDirection::ToLight) * light_vertex.cont_prob;

    Float camera_bsdf_dir_pdf_a = camera_bsdf_dir_pdf_w * cos_light / dist2;
    Float light_bsdf_dir_pdf_a = light_bsdf_dir_pdf_w * cos_camera / dist2;

    Float w_light =
        Mis(camera_bsdf_dir_pdf_a) * (mis_vm_weight + light_vertex.d_vcm + light_vertex.d_vc * Mis(light_bsdf_rev_pdf_w));

    Float w_camera =
        Mis(light_bsdf_dir_pdf_a) * (mis_vm_weight + camera_state.d_vcm + camera_state.d_vc * Mis(camera_bsdf_rev_pdf_w));

    Float mis_weight = 1 / (w_light + 1 + w_camera);
    Float geometry_term = 1 / dist2;
    return mis_weight * geometry_term * camera_f_cos * light_f_cos;
}

void ConnectToCamera(
    const Integrator* I,
    Film& film,
    const Camera* camera,
    const VCMSubPathState& light_state,
    const Intersection& isect,
    const Vec3& wo,
    const BSDF& bsdf,
    Float light_cont_prob,
    Float mis_vm_weight,
    Float light_subpath_count,
    Sampler& sampler
)
{
    Intersection ref;
    ref.point = isect.point;

    CameraSampleWi camera_sample;
    if (!camera->SampleWi(&camera_sample, ref, sampler.Next2D()))
    {
        return;
    }

    Vec3 wi = camera_sample.wi;

    Float cos_to_camera = AbsDot(isect.shading.normal, wi);
    if (cos_to_camera == 0)
    {
        return;
    }

    Spectrum f_cos = bsdf.f(wo, wi, TransportDirection::ToCamera) * cos_to_camera;
    if (f_cos.IsBlack())
    {
        return;
    }

    if (!V(I, isect.point, camera_sample.p_aperture))
    {
        return;
    }

    Float camera_pdf_p, camera_pdf_w;
    camera->PDF_We(&camera_pdf_p, &camera_pdf_w, Ray(camera_sample.p_aperture, -wi));

    Float dist2 = Dist2(isect.point, camera_sample.p_aperture);
    Float camera_pdf_a = camera_pdf_p * camera_pdf_w * cos_to_camera / dist2;
    if (camera_pdf_a == 0)
    {
        return;
    }

    Float bsdf_rev_pdf_w = bsdf.PDF(wi, wo, TransportDirection::ToLight) * light_cont_prob;

    Float w_light =
        Mis(camera_pdf_a / light_subpath_count) * (mis_vm_weight + light_state.d_vcm + light_state.d_vc * Mis(bsdf_rev_pdf_w));

    Float mis_weight = 1 / (1 + w_light);

    Spectrum contribution = mis_weight * light_state.beta * f_cos * camera_sample.Wi / (light_subpath_count * camera_sample.pdf);
    if (!contribution.IsBlack())
    {
        film.AddSplat(camera_sample.p_raster, contribution);
    }
}

VCMIntegrator::VCMIntegrator(
    const Intersectable* accel,
    std::vector<Light*> lights,
    const Sampler* sampler,
    int32 max_bounces,
    int32 rr_min_bounces,
    Float merge_radius,
    Float radius_alpha
)
    : Integrator(accel, std::move(lights), std::make_unique<PowerLightSampler>())
    , sampler_prototype{ sampler }
    , max_bounces{ max_bounces }
    , rr_min_bounces{ rr_min_bounces }
    , initial_radius{ merge_radius }
    , radius_alpha{ radius_alpha }
{
    if (initial_radius <= 0)
    {
        AABB world_bounds = accel->GetAABB();
        Point3 world_center;
        Float world_radius;
        world_bounds.ComputeBoundingSphere(&world_center, &world_radius);

        initial_radius = 2 * world_radius * 3e-3f;
    }
}

Rendering* VCMIntegrator::Render(Allocator& alloc, const Camera* camera)
{
    const int32 tile_size = 16;
    const int32 n_iterations = std::max<int32>(1, sampler_prototype->samples_per_pixel);
    const int32 max_path_length = max_bounces + 1;

    Point2i res = camera->GetScreenResolution();
    const int32 path_count = res.x * res.y;
    const Float light_subpath_count = path_count;

    Point2i num_tiles = (res + (tile_size - 1)) / tile_size;
    const int32 tile_count = num_tiles.x * num_tiles.y;

    std::vector<size_t> phase_works(2 * size_t(n_iterations));
    for (int32 i = 0; i < n_iterations; ++i)
    {
        phase_works[2 * i] = size_t(path_count);
        phase_works[2 * i + 1] = size_t(tile_count);
    }

    MultiPhaseRendering* progress = alloc.new_object<MultiPhaseRendering>(camera, phase_works);
    progress->job = RunAsync([=, this]() {
        std::vector<std::unique_ptr<BufferResource>> light_vertex_buffers;
        ThreadLocal<Allocator> light_vertex_allocators([&light_vertex_buffers]() {
            light_vertex_buffers.push_back(std::make_unique<BufferResource>(1024 * 1024));
            BufferResource* ptr = light_vertex_buffers.back().get();
            return Allocator(ptr);
        });

        for (int32 iteration = 0; iteration < n_iterations; ++iteration)
        {
            Float radius = initial_radius;
            radius /= std::pow(Float(iteration + 1), 0.5f * (1 - radius_alpha));
            radius = std::max(radius, 1e-7f);

            Float radius2 = Sqr(radius);
            Float vm_normalization = 1 / (pi * radius2 * light_subpath_count);

            Float eta_vcm = (pi * radius2) * light_subpath_count;
            Float mis_vm_weight = Mis(eta_vcm);
            Float mis_vc_weight = Mis(1 / eta_vcm);

            struct PathLightVertex
            {
                int32 path_index = 0;
                VCMLightVertex vertex;
            };

            struct LightPathChunk
            {
                Vec2i range = { 0, 0 };
                std::vector<int32> counts;
                std::vector<PathLightVertex> vertices;
            };

            ThreadLocal<std::vector<LightPathChunk>> tl_light_chunks;

            // Trace light sub-path and connect light vertex to camera vertex
            ParallelFor(0, path_count, [&](int32 begin, int32 end) {
                int8 mem[64];
                BufferResource buffer(mem, sizeof(mem));
                Allocator sampler_alloc(&buffer);
                Sampler* sampler = sampler_prototype->Clone(sampler_alloc);

                Allocator& alloc = light_vertex_allocators.Get();
                std::vector<LightPathChunk>& chunks = tl_light_chunks.Get();
                chunks.emplace_back();

                LightPathChunk& chunk = chunks.back();
                chunk.range = { begin, end };
                chunk.counts.assign(size_t(end - begin), 0);
                chunk.vertices.clear();
                chunk.vertices.reserve(size_t(2 * (end - begin)));

                for (int32 path_index = begin; path_index < end; ++path_index)
                {
                    Point2i pixel(path_index % res.x, path_index / res.x);
                    sampler->StartPixelSample(-pixel, iteration);

                    SampledLight sampled_light;
                    if (!light_sampler->Sample(&sampled_light, Intersection{}, sampler->Next1D()))
                    {
                        continue;
                    }

                    LightSampleLe light_sample;
                    if (!sampled_light.light->Sample_Le(&light_sample, sampler->Next2D(), sampler->Next2D()))
                    {
                        continue;
                    }

                    Float emission_pdf_w = sampled_light.pmf * light_sample.pdf_w;
                    if (emission_pdf_w == 0)
                    {
                        continue;
                    }

                    VCMSubPathState light_state;
                    light_state.origin = light_sample.ray.o;
                    light_state.direction = light_sample.ray.d;
                    light_state.path_length = 1;
                    light_state.specular_path = true;
                    light_state.eta_scale = 1;

                    light_state.is_finite_light = !sampled_light.light->IsInfiniteLight();

                    light_state.beta = light_sample.Le / (sampled_light.pmf * light_sample.pdf_p * light_sample.pdf_w);
                    if (light_sample.normal != Vec3::zero)
                    {
                        light_state.beta *= AbsDot(light_sample.normal, light_state.direction);
                    }

                    Float direct_pdf_a = sampled_light.pmf * light_sample.pdf_p;

                    light_state.d_vcm = Mis(direct_pdf_a / emission_pdf_w);

                    if (!sampled_light.light->IsDeltaLight())
                    {
                        Float used_cos = light_state.is_finite_light ? AbsDot(light_sample.normal, light_state.direction) : 1;
                        light_state.d_vc = Mis(used_cos / emission_pdf_w);
                    }
                    else
                    {
                        light_state.d_vc = 0;
                    }

                    light_state.d_vm = light_state.d_vc * mis_vc_weight;

                    while (true)
                    {
                        Intersection isect;
                        Ray ray(light_state.origin, light_state.direction);
                        if (!Intersect(&isect, ray, Ray::epsilon, infinity))
                        {
                            break;
                        }

                        Vec3 wo = Normalize(-ray.d);

                        BSDF bsdf;
                        if (!isect.GetBSDF(&bsdf, wo, alloc))
                        {
                            light_state.origin = isect.point;
                            continue;
                        }

                        Float cos_theta = AbsDot(isect.shading.normal, wo);
                        if (isect.t <= 0 || cos_theta <= 0)
                        {
                            break;
                        }

                        if (light_state.path_length > 1 || light_state.is_finite_light)
                        {
                            light_state.d_vcm *= Mis(Sqr(isect.t));
                        }

                        Float inv_cos = 1 / Mis(cos_theta);
                        light_state.d_vcm *= inv_cos;
                        light_state.d_vc *= inv_cos;
                        light_state.d_vm *= inv_cos;

                        Float vertex_cont_prob = ComputeContinuationProb(light_state, rr_min_bounces);

                        if (!IsSpecular(bsdf.Flags()))
                        {
                            PathLightVertex v_path;
                            v_path.path_index = path_index;

                            VCMLightVertex& v = v_path.vertex;
                            v.p = isect.point;
                            v.wo = wo;
                            v.shading_normal = isect.shading.normal;

                            v.beta = light_state.beta;
                            v.bsdf = bsdf;
                            v.path_length = light_state.path_length;
                            v.d_vcm = light_state.d_vcm;
                            v.d_vc = light_state.d_vc;
                            v.d_vm = light_state.d_vm;
                            v.cont_prob = vertex_cont_prob;

                            chunk.vertices.push_back(v_path);
                            ++chunk.counts[size_t(path_index - begin)];

                            if (light_state.path_length + 1 <= max_path_length)
                            {
                                ConnectToCamera(
                                    this, progress->film, camera, light_state, isect, wo, bsdf, vertex_cont_prob, mis_vm_weight,
                                    light_subpath_count, *sampler
                                );
                            }
                        }

                        if (light_state.path_length + 2 > max_path_length)
                        {
                            break;
                        }

                        if (!SampleScattering(
                                &light_state, isect, bsdf, wo, TransportDirection::ToCamera, vertex_cont_prob, mis_vm_weight,
                                mis_vc_weight, *sampler
                            ))
                        {
                            break;
                        }
                    }
                }

                progress->phase_works_dones[2 * iteration].fetch_add(end - begin, std::memory_order_relaxed);
            });

            std::vector<const LightPathChunk*> light_chunks;
            tl_light_chunks.ForEach([&](std::thread::id tid, std::vector<LightPathChunk>& chunks) {
                BulbitNotUsed(tid);

                for (const LightPathChunk& chunk : chunks)
                {
                    light_chunks.push_back(&chunk);
                }
            });

            std::sort(light_chunks.begin(), light_chunks.end(), [](const LightPathChunk* a, const LightPathChunk* b) {
                return a->range.x < b->range.x;
            });

            // Collect all light vertices and compute prefix sums(path_ends)
            std::vector<VCMLightVertex> light_vertices;
            std::vector<int32> path_ends(path_count);

            int32 total_light_vertices = 0;
            for (const LightPathChunk* chunk : light_chunks)
            {
                BulbitAssert(chunk->range.y >= chunk->range.x);
                BulbitAssert(chunk->counts.size() == size_t(chunk->range.y - chunk->range.x));

                for (int32 i = 0; i < chunk->range.y - chunk->range.x; ++i)
                {
                    int32 path_index = chunk->range.x + i;
                    total_light_vertices += chunk->counts[size_t(i)];
                    path_ends[path_index] = total_light_vertices;
                }
            }

            light_vertices.resize(total_light_vertices);

            std::vector<int32> next_write(path_count, 0);
            for (int32 path_index = 0; path_index < path_count; ++path_index)
            {
                next_write[path_index] = (path_index == 0) ? 0 : path_ends[path_index - 1];
            }

            for (const LightPathChunk* chunk : light_chunks)
            {
                for (const PathLightVertex& v_path : chunk->vertices)
                {
                    BulbitAssert(v_path.path_index >= 0 && v_path.path_index < path_count);
                    int32 write = next_write[v_path.path_index]++;

                    BulbitAssert(write < path_ends[v_path.path_index]);
                    if (write < path_ends[v_path.path_index])
                    {
                        light_vertices[write] = v_path.vertex;
                    }
                }
            }

            for (int32 path_index = 0; path_index < path_count; ++path_index)
            {
                BulbitAssert(next_write[path_index] == path_ends[path_index]);
            }

            HashGrid light_grid;
            if (!light_vertices.empty())
            {
                light_grid.Build(light_vertices, radius);
            }

            progress->phase_dones[2 * iteration].store(true, std::memory_order_release);

            // Trace camera sub-path and perform VCM
            ParallelFor2D(
                res,
                [&](AABB2i tile) {
                    int8 mem[64];
                    BufferResource buffer(mem, sizeof(mem));
                    Allocator sampler_alloc(&buffer);
                    Sampler* sampler = sampler_prototype->Clone(sampler_alloc);

                    for (Point2i pixel : tile)
                    {
                        sampler->StartPixelSample(pixel, iteration);

                        PrimaryRay primary_ray;
                        camera->SampleRay(&primary_ray, pixel, sampler->Next2D(), sampler->Next2D());

                        Float camera_pdf_p, camera_pdf_w;
                        camera->PDF_We(&camera_pdf_p, &camera_pdf_w, primary_ray.ray);
                        BulbitNotUsed(camera_pdf_p);

                        VCMSubPathState camera_state;
                        camera_state.origin = primary_ray.ray.o;
                        camera_state.direction = primary_ray.ray.d;
                        camera_state.beta = Spectrum(primary_ray.weight);
                        camera_state.path_length = 1;
                        camera_state.specular_path = true;
                        camera_state.eta_scale = 1;

                        camera_state.d_vcm = (camera_pdf_w > 0) ? Mis(1 / camera_pdf_w) : 0;
                        camera_state.d_vc = 0;
                        camera_state.d_vm = 0;

                        Spectrum L(0);

                        while (true)
                        {
                            Ray ray(camera_state.origin, camera_state.direction);

                            Intersection isect;
                            if (!Intersect(&isect, ray, Ray::epsilon, infinity))
                            {
                                if (camera_state.path_length <= max_path_length)
                                {
                                    for (const Light* light : infinite_lights)
                                    {
                                        Spectrum Le = light->Le(ray);
                                        if (Le.IsBlack())
                                        {
                                            continue;
                                        }

                                        if (camera_state.path_length == 1)
                                        {
                                            L += camera_state.beta * Le;
                                            continue;
                                        }

                                        Float light_pmf = GetLightSampler()->EvaluatePMF(light);
                                        Float direct_pdf_a = light_pmf * light->EvaluatePDF_Li(ray);
                                        Float emission_pdf_w = light_pmf * EmissionPDFW(light, ray.o, Vec3::zero, -ray.d);

                                        Float w_camera =
                                            Mis(direct_pdf_a) * camera_state.d_vcm + Mis(emission_pdf_w) * camera_state.d_vc;
                                        Float mis_weight = 1 / (1 + w_camera);

                                        L += camera_state.beta * mis_weight * Le;
                                    }
                                }
                                break;
                            }

                            Vec3 wo = Normalize(-ray.d);

                            int8 bsdf_mem[max_bxdf_size];
                            BufferResource bsdf_buffer(bsdf_mem, sizeof(bsdf_mem));
                            Allocator bsdf_alloc(&bsdf_buffer);
                            BSDF bsdf;

                            if (!isect.GetBSDF(&bsdf, wo, bsdf_alloc))
                            {
                                camera_state.origin = isect.point;
                                continue;
                            }

                            Float cos_theta = AbsDot(isect.shading.normal, wo);
                            if (isect.t <= 0 || cos_theta <= 0)
                            {
                                break;
                            }

                            camera_state.d_vcm *= Mis(Sqr(isect.t));

                            Float inv_cos = 1 / Mis(cos_theta);
                            camera_state.d_vcm *= inv_cos;
                            camera_state.d_vc *= inv_cos;
                            camera_state.d_vm *= inv_cos;

                            const Light* area_light = GetAreaLight(isect);
                            if (area_light)
                            {
                                if (camera_state.path_length <= max_path_length)
                                {
                                    L += camera_state.beta *
                                         AreaLightLe(this, area_light, isect, wo, camera_state.origin, camera_state);
                                }

                                // Light sources are treated as non-scattering endpoints for VCM
                                break;
                            }

                            if (camera_state.path_length >= max_path_length)
                            {
                                break;
                            }

                            Float vertex_cont_prob = ComputeContinuationProb(camera_state, rr_min_bounces);

                            if (!IsSpecular(bsdf.Flags()))
                            {
                                if (camera_state.path_length + 1 <= max_path_length)
                                {
                                    L += camera_state.beta *
                                         DirectIllumination(
                                             this, camera_state, isect, wo, bsdf, vertex_cont_prob, mis_vm_weight, *sampler
                                         );
                                }

                                int32 path_index = pixel.y * res.x + pixel.x;
                                int32 begin = (path_index == 0) ? 0 : path_ends[path_index - 1];
                                int32 end = path_ends[path_index];

                                for (int32 i = begin; i < end; ++i)
                                {
                                    const VCMLightVertex& light_vertex = light_vertices[i];
                                    if (light_vertex.path_length + camera_state.path_length + 1 > max_path_length)
                                    {
                                        break;
                                    }

                                    L += camera_state.beta * light_vertex.beta *
                                         ConnectVertices(
                                             this, light_vertex, light_vertex.bsdf, camera_state, isect, wo, bsdf,
                                             vertex_cont_prob, mis_vm_weight
                                         );
                                }
                            }

                            if (!IsSpecular(bsdf.Flags()) && !light_vertices.empty())
                            {
                                Spectrum merged(0);
                                light_grid.Query<VCMLightVertex>(
                                    light_vertices, isect.point, radius, [&](const VCMLightVertex& light_vertex) {
                                        if (light_vertex.path_length + camera_state.path_length > max_path_length)
                                        {
                                            return;
                                        }

                                        Vec3 wi = light_vertex.wo;
                                        Spectrum camera_bsdf = bsdf.f(wo, wi, TransportDirection::ToLight);
                                        if (camera_bsdf.IsBlack())
                                        {
                                            return;
                                        }

                                        Float camera_bsdf_dir_pdf_w =
                                            bsdf.PDF(wo, wi, TransportDirection::ToLight) * vertex_cont_prob;
                                        Float camera_bsdf_rev_pdf_w =
                                            bsdf.PDF(wi, wo, TransportDirection::ToCamera) * light_vertex.cont_prob;

                                        Float w_light =
                                            light_vertex.d_vcm * mis_vc_weight + light_vertex.d_vm * Mis(camera_bsdf_dir_pdf_w);
                                        Float w_camera =
                                            camera_state.d_vcm * mis_vc_weight + camera_state.d_vm * Mis(camera_bsdf_rev_pdf_w);

                                        Float mis_weight = 1 / (w_light + 1 + w_camera);
                                        merged += mis_weight * camera_bsdf * light_vertex.beta;
                                    }
                                );

                                L += camera_state.beta * vm_normalization * merged;
                            }

                            if (!SampleScattering(
                                    &camera_state, isect, bsdf, wo, TransportDirection::ToLight, vertex_cont_prob, mis_vm_weight,
                                    mis_vc_weight, *sampler
                                ))
                            {
                                break;
                            }
                        }

                        if (!L.IsNullish())
                        {
                            progress->film.AddSample(pixel, L);
                        }
                    }

                    progress->phase_works_dones[2 * iteration + 1].fetch_add(1, std::memory_order_relaxed);
                },
                tile_size
            );

            progress->phase_dones[2 * iteration + 1].store(true, std::memory_order_release);

            for (size_t i = 0; i < light_vertex_buffers.size(); ++i)
            {
                light_vertex_buffers[i]->release();
            }
        }

        progress->film.WeightSplats(1.0f / n_iterations);
        return true;
    });

    return progress;
}

} // namespace bulbit
