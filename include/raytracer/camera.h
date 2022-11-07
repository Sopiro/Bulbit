#pragma once

#include "common.h"
#include "ray.h"

class Camera
{
public:
    Camera(double aspect_ratio)
    {
        double viewport_height = 2.0;
        double viewport_width = aspect_ratio * viewport_height;
        double focal_length = 1.0;

        origin = Vec3{ 0.0 };
        horizontal = Vec3{ viewport_width, 0.0, 0.0 };
        vertical = Vec3{ 0.0, viewport_height, 0.0 };
        lower_left = origin - horizontal / 2.0 - vertical / 2.0 - Vec3{ 0.0, 0.0, focal_length };
    }

    Ray GetRay(double u, double v)
    {
        return Ray{ origin, lower_left + horizontal * u + vertical * v - origin };
    }

    Vec3 origin;
    Vec3 horizontal;
    Vec3 vertical;
    Vec3 lower_left;
};