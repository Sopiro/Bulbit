#pragma once

#include "camera.h"

namespace bulbit
{

class OrthographicCamera : public Camera
{
public:
    OrthographicCamera(
        const Point3& look_from,
        const Point3& look_at,
        const Vec3& up,
        const Point2& viewport_size,
        int32 resolution_x,
        const Medium* medium = nullptr,
        const Filter* pixel_filter = Camera::default_filter.get()
    );

    virtual Float SampleRay(Ray* out_ray, const Point2i& pixel, Point2 u0, Point2 u1) const override;

private:
    Point3 origin;
    Point3 lower_left;
    Vec3 horizontal, vertical;

    // Local coordinate frame
    Vec3 u, v, w;
};

class PerspectiveCamera : public Camera
{
public:
    PerspectiveCamera(
        const Point3& look_from,
        const Point3& look_at,
        const Vec3& up,
        Float vfov, // vertical field-of-view. in degrees.
        Float aperture,
        Float focus_dist,
        const Point2i& resolution,
        const Medium* medium = nullptr,
        const Filter* pixel_filter = Camera::default_filter.get()
    );

    virtual Float SampleRay(Ray* out_ray, const Point2i& pixel, Point2 u0, Point2 u1) const override;

private:
    Point3 origin;
    Point3 lower_left;
    Vec3 horizontal, vertical;

    Float lens_radius;

    // Local coordinate frame
    Vec3 u, v, w;
};

class SphericalCamera : public Camera
{
public:
    SphericalCamera(
        const Point3& position,
        const Point2i& resolution,
        const Medium* medium = nullptr,
        const Filter* pixel_filter = Camera::default_filter.get()
    );

    virtual Float SampleRay(Ray* out_ray, const Point2i& pixel, Point2 u0, Point2 u1) const override;

private:
    Point3 origin;
};

} // namespace bulbit
