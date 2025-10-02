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

} // namespace bulbit
