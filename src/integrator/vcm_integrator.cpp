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

    // Cache variables for efficient subpath MIS weight evaluation
    // See below for details..
    // http://www.iliyan.com/publications/ImplementingVCM
    Float d_vcm = 0; // MIS cache used for vertex connection and merging
    Float d_vc = 0;  // MIS cache used for vertex connection
    Float d_vm = 0;  // MIS cache used for vertex merging
};

struct VCMLightVertex
{
    Point3 p;
    Vec3 wo;
    Vec3 normal, shading_normal;

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

Float LightPDF(const Light* light, Float light_pmf, const Point3& light_point, const Vec3& light_normal, const Vec3& w_to_prev)
{
    Float pdf_p = 0;
    Float pdf_w = 0;

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

    // Keep delta-light conventions consistent with the rest of the renderer.
    // Point/spot lights are delta in position; directional lights are delta in direction.
    if (light->Is<PointLight>() || light->Is<SpotLight>())
    {
        return light_pmf * pdf_w;
    }
    else if (light->Is<DirectionalLight>() || light->Is<DirectionalAreaLight>())
    {
        return light_pmf * pdf_p;
    }
    else
    {
        return light_pmf * pdf_p * pdf_w;
    }
}

Spectrum EvaluateAreaLight(
    const Integrator* I,
    const Light* light,
    const Intersection& isect,
    const Vec3& wo,
    const Point3& prev_point,
    const VCMSubPathState& camera_state
)
{
    Spectrum radiance = light->Le(isect, wo);
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

    Float light_pmf = I->GetLightSampler()->EvaluatePMF(light);
    Float direct_pdf_w = isect.primitive->GetShape()->PDF(isect, Ray(prev_point, -wo));

    Float direct_pdf_a = light_pmf * direct_pdf_w * (cos_at_light / dist2);
    Float light_pdf = LightPDF(light, light_pmf, isect.point, isect.normal, wo);

    Float w_camera = Mis(direct_pdf_a) * camera_state.d_vcm + Mis(light_pdf) * camera_state.d_vc;
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

    Float direct_pdf_w = sampled_light.pmf * light_sample.pdf;
    if (direct_pdf_w == 0 || light_sample.Li.IsBlack())
    {
        return Spectrum::black;
    }

    Vec3 wi = light_sample.wi;
    Float cos_light = AbsDot(isect.shading.normal, wi);
    if (cos_light == 0)
    {
        return Spectrum::black;
    }

    Spectrum f_cos = bsdf.f(wo, wi, TransportDirection::ToLight) * cos_light;
    if (f_cos.IsBlack())
    {
        return Spectrum::black;
    }

    if (!V(I, isect.point, light_sample.point))
    {
        return Spectrum::black;
    }

    Float bsdf_dir_pdf_w = bsdf.PDF(wo, wi, TransportDirection::ToLight) * camera_cont_prob;
    if (sampled_light.light->IsDeltaLight())
    {
        bsdf_dir_pdf_w = 0;
    }

    Float bsdf_rev_pdf_w = bsdf.PDF(wi, wo, TransportDirection::ToCamera) * camera_cont_prob;

    Float light_pdf = LightPDF(sampled_light.light, sampled_light.pmf, light_sample.point, light_sample.normal, -wi);

    Float cos_at_light = (light_sample.normal != Vec3::zero) ? AbsDot(light_sample.normal, -wi) : 1;
    if (cos_at_light == 0)
    {
        return Spectrum::black;
    }

    Float w_light = Mis(bsdf_dir_pdf_w / direct_pdf_w);

    Float ratio = light_pdf * cos_light / (direct_pdf_w * cos_at_light);
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
    Vec3 direction = light_vertex.p - camera_isect.point;
    Float dist2 = Length2(direction);
    if (dist2 == 0)
    {
        return Spectrum::black;
    }

    Float distance = std::sqrt(dist2);
    direction /= distance;

    Float cos_camera = AbsDot(camera_isect.normal, direction);
    if (cos_camera == 0)
    {
        return Spectrum::black;
    }

    Float cos_light = AbsDot(light_vertex.normal, -direction);
    if (cos_light == 0)
    {
        return Spectrum::black;
    }

    Spectrum camera_f_cos = camera_bsdf.f(camera_wo, direction, TransportDirection::ToLight) * cos_camera;
    if (camera_f_cos.IsBlack())
    {
        return Spectrum::black;
    }

    Spectrum light_f_cos =
        light_bsdf.f(light_vertex.wo, -direction, TransportDirection::ToCamera) * AbsDot(light_vertex.shading_normal, -direction);
    if (light_f_cos.IsBlack())
    {
        return Spectrum::black;
    }

    if (!V(I, camera_isect.point, light_vertex.p))
    {
        return Spectrum::black;
    }

    Float camera_bsdf_dir_pdf_w = camera_bsdf.PDF(camera_wo, direction, TransportDirection::ToLight) * camera_cont_prob;
    Float camera_bsdf_rev_pdf_w = camera_bsdf.PDF(direction, camera_wo, TransportDirection::ToCamera) * camera_cont_prob;

    Float light_bsdf_dir_pdf_w =
        light_bsdf.PDF(light_vertex.wo, -direction, TransportDirection::ToCamera) * light_vertex.cont_prob;
    Float light_bsdf_rev_pdf_w =
        light_bsdf.PDF(-direction, light_vertex.wo, TransportDirection::ToLight) * light_vertex.cont_prob;

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

    Float camera_pdf_a = camera_sample.pdf * AbsDot(isect.normal, wi) / Dist2(isect.point, camera_sample.p_aperture);
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

            std::vector<std::vector<VCMLightVertex>> path_light_vertices(path_count);

            std::vector<std::unique_ptr<BufferResource>> light_vertex_buffers;
            ThreadLocal<Allocator> light_vertex_allocators([&light_vertex_buffers]() {
                light_vertex_buffers.push_back(std::make_unique<BufferResource>(1024 * 1024));
                BufferResource* ptr = light_vertex_buffers.back().get();
                return Allocator(ptr);
            });

            // Trace light sub-path and connect light vertex to camera vertex
            ParallelFor(0, path_count, [&](int32 begin, int32 end) {
                int8 mem[64];
                BufferResource buffer(mem, sizeof(mem));
                Allocator sampler_alloc(&buffer);
                Sampler* sampler = sampler_prototype->Clone(sampler_alloc);

                for (int32 path_index = begin; path_index < end; ++path_index)
                {
                    Point2i pixel(path_index % res.x, path_index / res.x);
                    sampler->StartPixelSample(-pixel, iteration);

                    std::vector<VCMLightVertex> vertices;
                    vertices.reserve(max_path_length);
                    Allocator& light_vertex_alloc = light_vertex_allocators.Get();

                    SampledLight sampled_light;
                    if (!light_sampler->Sample(&sampled_light, Intersection{}, sampler->Next1D()))
                    {
                        path_light_vertices[path_index] = std::move(vertices);
                        progress->phase_works_dones[2 * iteration].fetch_add(1, std::memory_order_relaxed);
                        continue;
                    }

                    LightSampleLe light_sample;
                    if (!sampled_light.light->Sample_Le(&light_sample, sampler->Next2D(), sampler->Next2D()))
                    {
                        path_light_vertices[path_index] = std::move(vertices);
                        progress->phase_works_dones[2 * iteration].fetch_add(1, std::memory_order_relaxed);
                        continue;
                    }

                    Float light_pdf = sampled_light.pmf * light_sample.pdf_p * light_sample.pdf_w;
                    if (light_pdf == 0)
                    {
                        path_light_vertices[path_index] = std::move(vertices);
                        progress->phase_works_dones[2 * iteration].fetch_add(1, std::memory_order_relaxed);
                        continue;
                    }

                    VCMSubPathState light_state;
                    light_state.origin = light_sample.ray.o;
                    light_state.direction = light_sample.ray.d;
                    light_state.path_length = 1;
                    light_state.specular_path = true;
                    light_state.eta_scale = 1;

                    light_state.is_finite_light = !sampled_light.light->IsInfiniteLight();

                    light_state.beta = light_sample.Le / light_pdf;
                    if (light_sample.normal != Vec3::zero)
                    {
                        light_state.beta *= AbsDot(light_sample.normal, light_state.direction);
                    }

                    Float direct_pdf_a = 0;
                    if (sampled_light.light->IsDeltaLight())
                    {
                        direct_pdf_a = sampled_light.pmf;
                    }
                    else if (sampled_light.light->IsInfiniteLight())
                    {
                        direct_pdf_a = sampled_light.pmf * light_sample.pdf_w;
                    }
                    else
                    {
                        direct_pdf_a = sampled_light.pmf * light_sample.pdf_p;
                    }

                    light_state.d_vcm = Mis(direct_pdf_a / light_pdf);

                    if (!sampled_light.light->IsDeltaLight())
                    {
                        Float cos_light =
                            (light_sample.normal != Vec3::zero) ? AbsDot(light_sample.normal, light_state.direction) : 1;
                        if (sampled_light.light->IsInfiniteLight())
                        {
                            cos_light = 1;
                        }

                        light_state.d_vc = Mis(cos_light / light_pdf);
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
                        if (!isect.GetBSDF(&bsdf, wo, light_vertex_alloc))
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
                            VCMLightVertex v;
                            v.p = isect.point;
                            v.wo = wo;
                            v.normal = isect.normal;
                            v.shading_normal = isect.shading.normal;

                            v.beta = light_state.beta;
                            v.bsdf = bsdf;
                            v.path_length = light_state.path_length;
                            v.d_vcm = light_state.d_vcm;
                            v.d_vc = light_state.d_vc;
                            v.d_vm = light_state.d_vm;
                            v.cont_prob = vertex_cont_prob;

                            vertices.push_back(v);

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

                    path_light_vertices[path_index] = std::move(vertices);
                    progress->phase_works_dones[2 * iteration].fetch_add(1, std::memory_order_relaxed);
                }
            });

            std::vector<VCMLightVertex> light_vertices;
            std::vector<int32> path_ends(path_count);

            size_t total_light_vertices = 0;
            for (const std::vector<VCMLightVertex>& path : path_light_vertices)
            {
                total_light_vertices += path.size();
            }

            light_vertices.reserve(total_light_vertices);
            for (int32 i = 0; i < path_count; ++i)
            {
                const std::vector<VCMLightVertex>& path = path_light_vertices[i];
                light_vertices.insert(light_vertices.end(), path.begin(), path.end());
                path_ends[i] = int32(light_vertices.size());
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

                        Float camera_pdf_p = 0;
                        Float camera_pdf_w = 0;
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
                                        Float light_pdf = LightPDF(light, light_pmf, ray.o, Vec3::zero, -ray.d);

                                        Float w_camera =
                                            Mis(direct_pdf_a) * camera_state.d_vcm + Mis(light_pdf) * camera_state.d_vc;
                                        Float mis_weight = 1 / (1 + w_camera);

                                        L += camera_state.beta * mis_weight * Le;
                                    }
                                }
                                break;
                            }

                            Vec3 wo = Normalize(-ray.d);

                            if (const Light* area_light = GetAreaLight(isect); area_light)
                            {
                                if (camera_state.path_length <= max_path_length)
                                {
                                    L += camera_state.beta *
                                         EvaluateAreaLight(this, area_light, isect, wo, camera_state.origin, camera_state);
                                }
                            }

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

                            // Light sources are treated as non-scattering endpoints for VCM
                            if (GetAreaLight(isect))
                            {
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
        }

        progress->film.WeightSplats(1.0f / n_iterations);
        return true;
    });

    return progress;
}

} // namespace bulbit
