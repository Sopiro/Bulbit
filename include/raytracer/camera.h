#pragma once

#include "common.h"
#include "ray.h"

class Camera
{
public:
    Camera(Vec3 look_from,
           Vec3 look_at,
           Vec3 up,
           double vfov, // vertical field-of-view in degrees
           double aspect_ratio)
    {
        double theta = DegToRad(vfov);
        double h = tan(theta / 2.0);
        double viewport_height = 2.0 * h;
        double viewport_width = aspect_ratio * viewport_height;

        origin = look_from;
        dir = (look_at - look_from).Normalized();

        Vec3 u = Cross(dir, up).Normalized();
        Vec3 v = Cross(u, dir);

        horizontal = viewport_width * u;
        vertical = viewport_height * v;
        lower_left = origin - horizontal / 2.0 - vertical / 2.0 + dir;
    }

    Ray GetRay(double u, double v)
    {
        return Ray{ origin, lower_left + horizontal * u + vertical * v - origin };
    }

    Vec3 origin;
    Vec3 dir;

    Vec3 horizontal;
    Vec3 vertical;
    Vec3 lower_left;
};