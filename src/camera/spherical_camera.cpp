#include "bulbit/cameras.h"
#include "bulbit/frame.h"

namespace bulbit
{

SphericalCamera::SphericalCamera(const Transform& tf, const Point2i& resolution, const Medium* medium, const Filter* pixel_filter)
    : Camera(resolution, medium, pixel_filter)
    , origin{ tf.p }
{
}

void SphericalCamera::SampleRay(PrimaryRay* ray, const Point2i& pixel, Point2 u0, Point2 u1) const
{
    BulbitNotUsed(pixel);
    BulbitNotUsed(u1);

    Float theta = (1 - u0[1] / Float(resolution.y)) * pi;
    Float phi = u0[0] / Float(resolution.x) * two_pi;

    ray->ray = Ray(origin, SphericalDirection(theta, phi));
    ray->weight = 1;
}

} // namespace bulbit
