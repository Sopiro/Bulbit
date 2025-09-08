#include "bulbit/async_job.h"
#include "bulbit/bsdf.h"
#include "bulbit/bxdfs.h"
#include "bulbit/film.h"
#include "bulbit/integrators.h"
#include "bulbit/microfacet.h"
#include "bulbit/parallel_for.h"
#include "bulbit/progresses.h"
#include "bulbit/sampler.h"
#include "bulbit/visibility.h"

namespace bulbit
{

SPPMIntegrator::SPPMIntegrator(
    const Intersectable* accel,
    std::vector<Light*> lights,
    const Sampler* sampler,
    int32 max_bounces,
    int32 n_photons,
    Float radius
)
    : Integrator(accel, std::move(lights))
    , sampler_prototype{ sampler }
    , max_bounces{ max_bounces }
    , n_photons{ n_photons }
    , initial_radius{ radius }
{
    if (initial_radius <= 0)
    {
        AABB world_bounds = accel->GetAABB();
        Point3 world_center;
        Float world_radius;
        world_bounds.ComputeBoundingSphere(&world_center, &world_radius);

        initial_radius = 2 * world_radius * 5e-4f;
    }
}

Spectrum SPPMIntegrator::SampleDirectLight(
    const Vec3& wo, const Intersection& isect, BSDF* bsdf, Sampler& sampler, const Spectrum& beta
) const
{
    Float u0 = sampler.Next1D();
    Point2 u12 = sampler.Next2D();
    SampledLight sampled_light;
    if (!light_sampler.Sample(&sampled_light, isect, u0))
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

std::unique_ptr<Rendering> SPPMIntegrator::Render(const Camera* camera)
{
    const int32 n_interations = sampler_prototype->samples_per_pixel;
    const int32 tile_size = 16;

    Point2i res = camera->GetScreenResolution();
    int32 num_tiles_x = (res.x + tile_size - 1) / tile_size;
    int32 num_tiles_y = (res.y + tile_size - 1) / tile_size;
    int32 tile_count = num_tiles_x * num_tiles_y;

    std::vector<size_t> phase_works(2 * n_interations);
    for (size_t i = 0; i < phase_works.size(); i += 2)
    {
        phase_works[i] = size_t(tile_count);
        phase_works[i + 1] = size_t(n_photons);
    }

    MultiPhaseRendering* progress = new MultiPhaseRendering(camera, phase_works);
    progress->job = RunAsync([=, this]() {
        std::vector<std::unique_ptr<BufferResource>> thread_buffers;
        ThreadLocal<Allocator> thread_allocators([&thread_buffers]() {
            thread_buffers.push_back(std::make_unique<BufferResource>(1024 * 1024));
            BufferResource* ptr = thread_buffers.back().get();
            return Allocator(ptr);
        });

        int32 n_pixels = res.x * res.y;
        std::vector<VisiblePoint> visible_points(n_pixels);
        for (VisiblePoint& vp : visible_points)
        {
            vp.radius = initial_radius;
        }

        for (int32 iteration = 0; iteration < n_interations; ++iteration)
        {
            // Generate visible points
            ParallelFor2D(
                res,
                [&](AABB2i tile) {
                    std::unique_ptr<Sampler> sampler = sampler_prototype->Clone();

                    for (Point2i pixel : tile)
                    {
                        sampler->StartPixelSample(pixel, iteration);

                        PrimaryRay primary_ray;
                        camera->SampleRay(&primary_ray, pixel, sampler->Next2D(), sampler->Next2D());

                        int32 index = res.x * pixel.y + pixel.x;
                        VisiblePoint& vp = visible_points[index];

                        Float eta_scale = 1;
                        Float prev_bsdf_pdf = 0;

                        int32 bounce = 0;
                        bool specular_bounce = true;
                        bool found_visible_point = false;

                        Spectrum beta(primary_ray.weight);
                        Ray ray = primary_ray.ray;

                        // Trace camera ray and find appropriate visible point
                        while (true)
                        {
                            Intersection isect;
                            if (!Intersect(&isect, ray, Ray::epsilon, infinity))
                            {
                                Spectrum L(0);
                                if (bounce == 0 || specular_bounce)
                                {
                                    for (Light* light : infinite_lights)
                                    {
                                        L += beta * light->Le(ray);
                                    }
                                }
                                else
                                {
                                    // Evaluate BSDF sample MIS for infinite light
                                    for (Light* light : infinite_lights)
                                    {
                                        Float light_pdf = light->EvaluatePDF_Li(ray) * light_sampler.EvaluatePMF(light);
                                        Float mis_weight = PowerHeuristic(1, prev_bsdf_pdf, 1, light_pdf);

                                        L += beta * mis_weight * light->Le(ray);
                                    }
                                }

                                vp.Ld += L;
                                break;
                            }

                            Vec3 wo = Normalize(-ray.d);

                            if (Spectrum Le = isect.Le(wo); !Le.IsBlack())
                            {
                                Spectrum L(0);
                                bool has_area_light = area_lights.contains(isect.primitive);
                                if (bounce == 0 || specular_bounce || !has_area_light)
                                {
                                    L += beta * Le;
                                }
                                else if (has_area_light)
                                {
                                    // Evaluate BSDF sample with MIS for area light
                                    AreaLight* area_light = area_lights.at(isect.primitive);

                                    Float light_pdf =
                                        isect.primitive->GetShape()->PDF(isect, ray) * light_sampler.EvaluatePMF(area_light);
                                    Float mis_weight = PowerHeuristic(1, prev_bsdf_pdf, 1, light_pdf);

                                    L += beta * mis_weight * Le;
                                }

                                vp.Ld += L;
                            }

                            if (bounce++ >= max_bounces || found_visible_point)
                            {
                                break;
                            }

                            Allocator& alloc = thread_allocators.Get();
                            BSDF bsdf;
                            if (!isect.GetBSDF(&bsdf, wo, alloc))
                            {
                                ray = Ray(isect.point, -wo);
                                --bounce;
                                continue;
                            }

                            vp.Ld += SampleDirectLight(wo, isect, &bsdf, *sampler, beta);

                            BxDF_Flags flags = bsdf.Flags();
                            if (IsDiffuse(flags) || (IsGlossy(flags) && bounce == max_bounces))
                            {
                                // Create visible point
                                vp.p = isect.point;
                                vp.wo = wo;
                                vp.bsdf = bsdf;
                                vp.beta = beta;

                                found_visible_point = true;
                                // Trace one bounce more for the unidirectional MIS contribution
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

                            prev_bsdf_pdf = bsdf_sample.is_stochastic ? bsdf.PDF(wo, bsdf_sample.wi) : bsdf_sample.pdf;
                            beta *= bsdf_sample.f * AbsDot(isect.shading.normal, bsdf_sample.wi) / bsdf_sample.pdf;
                            ray = Ray(isect.point, bsdf_sample.wi);

                            // Terminate path with russian roulette
                            constexpr int32 min_bounces = 2;
                            if (bounce > min_bounces)
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
                    }

                    progress->phase_works_dones[2 * iteration]++;
                },
                tile_size
            );

            Float max_radius = 0;
            for (const VisiblePoint& vp : visible_points)
            {
                if (vp.beta.IsBlack())
                {
                    continue;
                }

                max_radius = std::max(max_radius, vp.radius);
            }

            // Build hash grid of visible points
            HashGrid grid;
            grid.Build(visible_points, max_radius);

            progress->phase_dones[2 * iteration] = true;
            progress->phase_dones[2 * iteration + 1] = true;

            for (size_t i = 0; i < thread_buffers.size(); ++i)
            {
                thread_buffers[i]->release();
            }
        }

        ParallelFor2D(
            res,
            [&](AABB2i tile) {
                for (Point2i pixel : tile)
                {
                    int32 index = res.x * pixel.y + pixel.x;
                    VisiblePoint& vp = visible_points[index];

                    progress->film.AddSample(pixel, vp.Ld / n_interations);
                }
            },
            tile_size
        );

        return true;
    });

    return std::unique_ptr<Rendering>(progress);
}

} // namespace bulbit
