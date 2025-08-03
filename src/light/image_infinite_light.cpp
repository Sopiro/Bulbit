#include "bulbit/frame.h"
#include "bulbit/lights.h"

#include <memory>

namespace bulbit
{

ImageInfiniteLight::ImageInfiniteLight(const SpectrumImageTexture* l_map, const Transform& tf, Float l_scale)
    : Light(TypeIndexOf<ImageInfiniteLight>())
    , l_map{ l_map }
    , l_scale{ l_scale }
    , transform{ tf }
{
    int32 width = l_map->GetWidth();
    int32 height = l_map->GetHeight();

    std::unique_ptr<Float[]> image(new Float[width * height]);
    for (int32 v = 0; v < height; ++v)
    {
        Float vp = (v + 0.5f) / height;
        Float sin_theta = std::sin(pi * vp);

        for (int32 u = 0; u < width; ++u)
        {
            Float up = Float(u) / width;
            image[u + v * width] = (Float)std::fmax(0, sin_theta * l_map->Evaluate(Point2(up, vp)).Luminance());
        }
    }

    distribution = std::make_unique<Distribution2D>(image.get(), width, height);
}

void ImageInfiniteLight::Destroy()
{
    distribution.reset();
}

void ImageInfiniteLight::Preprocess(const AABB& world_bounds)
{
    world_bounds.ComputeBoundingSphere(&world_center, &world_radius);
}

Spectrum ImageInfiniteLight::Le(const Ray& ray) const
{
    Vec3 w = MulT(transform, Normalize(ray.d));
    Float theta = SphericalTheta(w), phi = SphericalPhi(w);
    Point2 uv(phi * inv_two_pi, 1 - theta * inv_pi);

    return l_scale * l_map->Evaluate(uv);
}

bool ImageInfiniteLight::Sample_Li(LightSampleLi* light_sample, const Intersection& ref, Point2 u) const
{
    BulbitNotUsed(ref);

    Float map_pdf;
    Point2 uv = distribution->SampleContinuous(u, &map_pdf);

    if (map_pdf == 0)
    {
        return false;
    }

    Float theta = (1 - uv[1]) * pi;
    Float phi = uv[0] * two_pi;

    Float cos_theta = std::cos(theta), sin_theta = std::sin(theta);
    Float sin_phi = std::sin(phi), cos_phi = std::cos(phi);

    light_sample->wi = Mul(transform, SphericalDirection(sin_theta, cos_theta, sin_phi, cos_phi));

    if (sin_theta == 0)
    {
        return false;
    }
    else
    {
        light_sample->pdf = map_pdf / (2 * Sqr(pi) * sin_theta);
    }

    light_sample->visibility = 2 * world_radius;
    light_sample->Li = l_scale * l_map->Evaluate(uv);

    return true;
}

Float ImageInfiniteLight::EvaluatePDF_Li(const Ray& ray) const
{
    Vec3 w = MulT(transform, Normalize(ray.d));
    Float theta = SphericalTheta(w), phi = SphericalPhi(w);
    Float sin_theta = std::sin(theta);
    if (sin_theta == 0)
    {
        return 0;
    }

    Point2 uv(phi * inv_two_pi, 1 - theta * inv_pi);
    return distribution->PDF(uv) / (2 * Sqr(pi) * sin_theta);
}

bool ImageInfiniteLight::Sample_Le(LightSampleLe* sample, Point2 u0, Point2 u1) const
{
    Float map_pdf;
    Point2 uv = distribution->SampleContinuous(u1, &map_pdf);

    if (map_pdf == 0)
    {
        return false;
    }

    Float theta = (1 - uv[1]) * pi;
    Float phi = uv[0] * two_pi;

    Float cos_theta = std::cos(theta), sin_theta = std::sin(theta);
    Float sin_phi = std::sin(phi), cos_phi = std::cos(phi);

    Vec3 wo = -Mul(transform, SphericalDirection(sin_theta, cos_theta, sin_phi, cos_phi));

    Frame frame(wo);

    Point2 u_disk = SampleUniformUnitDiskConcentric(u0);
    Point3 p_disk = world_center + world_radius * frame.FromLocal(Point3(u_disk, 0));

    sample->ray = Ray(p_disk - world_radius * wo, wo);
    sample->normal = wo;
    sample->pdf_p = 1 / (pi * Sqr(world_radius));
    sample->pdf_w = map_pdf / (2 * Sqr(pi) * sin_theta);
    sample->Le = l_scale * l_map->Evaluate(uv);
    sample->medium = nullptr;

    return true;
}

void ImageInfiniteLight::EvaluatePDF_Le(Float* pdf_p, Float* pdf_w, const Ray& ray) const
{
    Vec3 w = -MulT(transform, Normalize(ray.d));
    Float theta = SphericalTheta(w), phi = SphericalPhi(w);
    Float sin_theta = std::sin(theta);
    if (sin_theta == 0)
    {
        *pdf_w = 0;
    }
    else
    {
        Point2 uv(phi * inv_two_pi, 1 - theta * inv_pi);
        *pdf_w = distribution->PDF(uv) / (2 * Sqr(pi) * sin_theta);
    }

    *pdf_p = 1 / (pi * Sqr(world_radius));
}

void ImageInfiniteLight::PDF_Le(Float* pdf_p, Float* pdf_w, const Intersection& isect, const Vec3& w) const
{
    // This function should be called by AreaLight only
    BulbitNotUsed(pdf_p);
    BulbitNotUsed(pdf_w);
    BulbitNotUsed(isect);
    BulbitNotUsed(w);
    BulbitAssert(false);
}

} // namespace bulbit
