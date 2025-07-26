#include "bulbit/cameras.h"
#include "bulbit/ray.h"
#include "bulbit/sampling.h"

namespace bulbit
{

PerspectiveCamera::PerspectiveCamera(
    const Point3& look_from,
    const Point3& look_at,
    const Vec3& up,
    Float vfov,
    Float aperture,
    Float focus_distance,
    const Point2i& resolution,
    const Medium* medium,
    const Filter* pixel_filter
)
    : Camera(resolution, medium, pixel_filter)
    , focus_distance{ focus_distance }
{
    Float theta = DegToRad(vfov);
    Float h = std::tan(theta / 2);
    Float aspect_ratio = (Float)resolution.x / (Float)resolution.y;
    Float viewport_height = 2 * h;
    Float viewport_width = aspect_ratio * viewport_height;

    w = Normalize(look_from - look_at);
    u = Normalize(Cross(up, w));
    v = Cross(w, u);

    origin = look_from;
    horizontal = focus_distance * viewport_width * u;
    vertical = focus_distance * viewport_height * v;
    lower_left = origin - horizontal / 2 - vertical / 2 - focus_distance * w;

    lens_radius = aperture / 2;

    A_viewport = viewport_width * viewport_height;
    A_lens = lens_radius != 0 ? (pi * Sqr(lens_radius)) : 1;
}

Float PerspectiveCamera::SampleRay(Ray* ray, const Point2i& pixel, Point2 u0, Point2 u1) const
{
    Point2 pixel_offset = filter->Sample(u0) + Point2(Float(0.5), Float(0.5));
    Point3 pixel_center = lower_left + horizontal * (pixel.x + pixel_offset.x) / resolution.x +
                          vertical * (pixel.y + pixel_offset.y) / resolution.y;

    Point2 aperture_sample = lens_radius * SampleUniformUnitDiskConcentric(u1);
    Point3 camera_offset = u * aperture_sample.x + v * aperture_sample.y;
    Point3 camera_center = origin + camera_offset;

    ray->o = camera_center;
    ray->d = Normalize(pixel_center - camera_center);

    return 1;
}

Spectrum PerspectiveCamera::We(const Ray& ray, Point2* p_raster) const
{
    Float cos_theta = Dot(-w, ray.d);
    if (cos_theta <= 0)
    {
        return Spectrum::black;
    }

    Point3 p_focus = ray.At(focus_distance / cos_theta);

    Vec3 ll2p = p_focus - lower_left;
    Float w2 = Length2(horizontal);
    Float h2 = Length2(vertical);

    Float px = resolution.x * Dot(horizontal, ll2p) / w2;
    Float py = resolution.y * Dot(vertical, ll2p) / h2;

    if (p_raster)
    {
        *p_raster = Point2(px, py);
    }

    if (px < 0 || px >= resolution.x || py < 0 || py >= resolution.y)
    {
        return Spectrum::black;
    }

    return Spectrum(1 / (A_viewport * A_lens * Sqr(Sqr(cos_theta))));
}

void PerspectiveCamera::PDF_We(Float* pdf_p, Float* pdf_w, const Ray& ray) const
{
    Float cos_theta = Dot(-w, ray.d);
    if (cos_theta <= 0)
    {
        *pdf_p = 0;
        *pdf_w = 0;
        return;
    }

    Point3 p_focus = ray.At(focus_distance / cos_theta);

    Vec3 ll2p = p_focus - lower_left;
    Float w2 = Length2(horizontal);
    Float h2 = Length2(vertical);

    Float px = resolution.x * Dot(horizontal, ll2p) / w2;
    Float py = resolution.y * Dot(vertical, ll2p) / h2;

    if (px < 0 || px >= resolution.x || py < 0 || py >= resolution.y)
    {
        *pdf_p = 0;
        *pdf_w = 0;
        return;
    }

    *pdf_p = 1 / A_lens;
    *pdf_w = 1 / (A_viewport * cos_theta * cos_theta * cos_theta);
}

bool PerspectiveCamera::SampleWi(CameraSampleWi* sample, const Intersection& ref, Point2 u0) const
{
    Point2 aperture_sample = lens_radius * SampleUniformUnitDiskConcentric(u0);
    Point3 aperture_offset = u * aperture_sample.x + v * aperture_sample.y;
    Point3 p_aperture = origin + aperture_offset;

    Vec3 wi = p_aperture - ref.point;
    Float dist = wi.Normalize();

    Float leas_area = lens_radius != 0 ? (pi * Sqr(lens_radius)) : 1;
    Float pdf = Sqr(dist) / (AbsDot(-w, wi) * leas_area); // joint pdf

    Point2 p_raster;
    Spectrum Wi = We(Ray(p_aperture, -wi), &p_raster);
    if (Wi == Spectrum::black)
    {
        return false;
    }

    *sample = CameraSampleWi(Wi, wi, pdf, p_raster, p_aperture, -w, medium);

    return true;
}

} // namespace bulbit
