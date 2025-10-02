#include "bulbit/cameras.h"
#include "bulbit/frame.h"

namespace bulbit
{

SphericalCamera::SphericalCamera(const Transform& tf, const Point2i& resolution, const Medium* medium, const Filter* filter)
    : Camera(resolution, medium, filter)
    , origin{ tf.p }
{
}

void SphericalCamera::SampleRay(PrimaryRay* ray, const Point2i& pixel, Point2 u0, Point2 u1) const
{
    BulbitNotUsed(u1);

    Point2 p_film = Point2(pixel.x + u0[0], pixel.y + u0[1]);
    Float theta = (1 - p_film.y / resolution.y) * pi;
    Float phi = p_film.x / resolution.x * two_pi;

    ray->ray = Ray(origin, SphericalDirection(theta, phi));
    ray->weight = 1;
}

} // namespace bulbit
