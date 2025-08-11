#include "bulbit/integrators.h"

#include "bulbit/async_job.h"
#include "bulbit/parallel_for.h"
#include "bulbit/progress.h"

namespace bulbit
{

Integrator::Integrator(const Intersectable* accel, std::vector<Light*> lights)
    : accel{ accel }
    , all_lights{ std::move(lights) }
{
    AABB world_bounds = accel->GetAABB();
    for (size_t i = 0; i < all_lights.size(); i++)
    {
        all_lights[i]->Preprocess(world_bounds);
    }
}

bool Integrator::V(const Point3 p1, const Point3 p2) const
{
    Vec3 d = p2 - p1;
    Float visibility = d.Normalize();
    Ray ray(p1, d);

    Intersection isect;
    while (visibility > 0 && Intersect(&isect, ray, Ray::epsilon, visibility))
    {
        if (isect.primitive->GetMaterial())
        {
            return false;
        }

        ray.o = isect.point;
        visibility -= isect.t;
    }

    return true;
}

Spectrum Integrator::Tr(const Point3 p1, const Point3 p2, const Medium* medium, int32 wavelength) const
{
    Vec3 w = p2 - p1;
    Float visibility = w.Normalize();

    Ray ray(p1, w);

    Spectrum Tr(1);
    Spectrum r_pdf(1);

    RNG rng(Hash(p1, wavelength), Hash(p2, wavelength));

    while (visibility > 0)
    {
        Intersection isect;
        bool found_intersection = Intersect(&isect, ray, Ray::epsilon, visibility);

        // Intersects opaque boundary
        if (found_intersection && isect.primitive->GetMaterial())
        {
            return Spectrum::black;
        }

        if (medium)
        {
            Float t_max = found_intersection ? isect.t : visibility;

            // Estimate transmittance with ratio tracking
            Spectrum T_maj = Sample_MajorantTransmittance(
                medium, wavelength, ray, t_max, rng.NextFloat(), rng,
                [&](Point3 p, MediumSample ms, Spectrum sigma_maj, Spectrum T_maj) {
                    BulbitNotUsed(p);

                    Spectrum sigma_n = Max<Float>(sigma_maj - ms.sigma_a - ms.sigma_s, 0);
                    Float pdf = sigma_maj[wavelength] * T_maj[wavelength];

                    Tr *= sigma_n * T_maj / pdf;
                    r_pdf *= sigma_maj * T_maj / pdf;

                    return !Tr.IsBlack() && !r_pdf.IsBlack();
                }
            );

            Float pdf = T_maj[wavelength];
            Tr *= T_maj / pdf;
            r_pdf *= T_maj / pdf;
        }

        if (Tr.IsBlack())
        {
            return Spectrum::black;
        }

        if (!found_intersection)
        {
            break;
        }

        ray.o = isect.point;
        visibility -= isect.t;
        medium = isect.GetMedium(ray.d);
    }

    return Tr / r_pdf.Average();
}

UniDirectionalRayIntegrator::UniDirectionalRayIntegrator(
    const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler
)
    : Integrator(accel, std::move(lights))
    , sampler_prototype{ sampler }
{
}

std::unique_ptr<Rendering> UniDirectionalRayIntegrator::Render(const Camera* camera)
{
    ComoputeReflectanceTextures();

    Point2i resolution = camera->GetScreenResolution();

    const int32 spp = sampler_prototype->samples_per_pixel;
    const int32 tile_size = 16;

    std::unique_ptr<Rendering> progress = std::make_unique<Rendering>(camera, tile_size);

    progress->job = RunAsync([=, this, &progress]() {
        ParallelFor2D(
            resolution,
            [&](AABB2i tile) {
                // Thread local sampler for current tile
                std::unique_ptr<Sampler> sampler = sampler_prototype->Clone();

                for (Point2i pixel : tile)
                {
                    for (int32 sample = 0; sample < spp; ++sample)
                    {
                        sampler->StartPixelSample(pixel, sample);

                        PrimaryRay primary_ray;
                        camera->SampleRay(&primary_ray, pixel, sampler->Next2D(), sampler->Next2D());

                        Spectrum L = Li(primary_ray.ray, camera->GetMedium(), *sampler);
                        if (!L.IsNullish())
                        {
                            progress->film.AddSample(pixel, primary_ray.weight * L);
                        }
                    }
                }

                progress->tile_done++;
            },
            tile_size
        );

        progress->done = true;
        return true;
    });

    return progress;
}

BiDirectionalRayIntegrator::BiDirectionalRayIntegrator(
    const Intersectable* accel, std::vector<Light*> lights, const Sampler* sampler
)
    : Integrator(accel, std::move(lights))
    , sampler_prototype{ sampler }
{
}

std::unique_ptr<Rendering> BiDirectionalRayIntegrator::Render(const Camera* camera)
{
    ComoputeReflectanceTextures();

    Point2i resolution = camera->GetScreenResolution();

    const int32 spp = sampler_prototype->samples_per_pixel;
    const int32 tile_size = 16;

    std::unique_ptr<Rendering> progress = std::make_unique<Rendering>(camera, tile_size);

    progress->job = RunAsync([=, this, &progress]() {
        ParallelFor2D(
            resolution,
            [&](AABB2i tile) {
                // Thread local sampler for current tile
                std::unique_ptr<Sampler> sampler = sampler_prototype->Clone();

                for (Point2i pixel : tile)
                {
                    for (int32 sample = 0; sample < spp; ++sample)
                    {
                        sampler->StartPixelSample(pixel, sample);

                        PrimaryRay primary_ray;
                        camera->SampleRay(&primary_ray, pixel, sampler->Next2D(), sampler->Next2D());

                        Spectrum Li = L(primary_ray.ray, camera->GetMedium(), camera, progress->film, *sampler);
                        if (!Li.IsNullish())
                        {
                            progress->film.AddSample(pixel, primary_ray.weight * Li);
                        }
                    }
                }

                progress->tile_done++;
            },
            tile_size
        );

        progress->film.WeightSplats(1.0f / spp);
        progress->done = true;
        return true;
    });

    return progress;
}

} // namespace bulbit
