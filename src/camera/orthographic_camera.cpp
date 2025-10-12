#include "bulbit/cameras.h"
#include "bulbit/ray.h"
#include "bulbit/sampling.h"

namespace bulbit
{

OrthographicCamera::OrthographicCamera(
    const Transform& tf, const Point2& viewport_size, int32 resolution_x, const Medium* medium, const Filter* filter
)
    : Camera(Point2i(resolution_x, int32(resolution_x * viewport_size.y / viewport_size.x)), medium, filter)
{
    u = -tf.q.GetBasisX();
    v = tf.q.GetBasisY();
    w = tf.q.GetBasisZ();

    origin = tf.p;
    horizontal = viewport_size.x * u;
    vertical = viewport_size.y * v;
    lower_left = origin - horizontal / 2 - vertical / 2;

    // viewport(image plane) area in world units
    A_viewport = Length(horizontal) * Length(vertical);
}

void OrthographicCamera::SampleRay(PrimaryRay* ray, const Point2i& pixel, Point2 u0, Point2 u1) const
{
    BulbitNotUsed(u1);

    Point2 pixel_offset = filter->Sample(u0) + Point2(Float(0.5), Float(0.5));
    Point3 origin = lower_left + horizontal * (pixel.x + pixel_offset.x) / resolution.x +
                    vertical * (pixel.y + pixel_offset.y) / resolution.y;

    ray->ray = Ray(origin, w);
    ray->weight = 1;
}

Spectrum OrthographicCamera::We(const Ray& ray, Point2* p_raster) const
{
    BulbitNotUsed(ray);
    BulbitNotUsed(p_raster);
    return Spectrum::black;
}

void OrthographicCamera::PDF_We(Float* pdf_p, Float* pdf_w, const Ray& ray) const
{
    *pdf_w = 0;
    // *pdf_p = 0;

    Float cos_theta = Dot(w, ray.d);
    if (cos_theta <= 0)
    {
        *pdf_p = 0;
        return;
    }

    Float t = Dot(w, origin - ray.o) / cos_theta;

    Point3 p_image = ray.At(t);
    Vec3 ll2p = p_image - lower_left;
    Float w2 = Length2(horizontal);
    Float h2 = Length2(vertical);

    Float px = resolution.x * Dot(horizontal, ll2p) / w2;
    Float py = resolution.y * Dot(vertical, ll2p) / h2;

    if (px < 0 || px >= resolution.x || py < 0 || py >= resolution.y)
    {
        *pdf_p = 0;
    }
    else
    {
        *pdf_p = 1 / A_viewport;
    }
}

bool OrthographicCamera::SampleWi(CameraSampleWi* sample, const Intersection& ref, Point2 u) const
{
    BulbitNotUsed(u); // deterministic projection, it importance samples direc delta function

    Vec3 d = ref.point - lower_left;
    if (Dot(d, w) <= 0)
    {
        return false;
    }

    Point3 p = ref.point - Dot(w, ref.point - lower_left) * w;

    Vec3 ll2p = p - lower_left;
    Float w2 = Length2(horizontal);
    Float h2 = Length2(vertical);

    Float px = resolution.x * Dot(horizontal, ll2p) / w2;
    Float py = resolution.y * Dot(vertical, ll2p) / h2;

    if (px < 0 || px >= resolution.x || py < 0 || py >= resolution.y)
    {
        return false;
    }

    Spectrum Wi = Spectrum(1 / A_viewport);

    *sample = CameraSampleWi(Wi, -w, 1, Point2(px, py), p, w, medium);
    return true;
}

} // namespace bulbit
