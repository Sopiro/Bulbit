#pragma once

#include "intersectable.h"
#include "ray.h"

namespace bulbit
{

struct ShapeSample
{
    Point3 point;
    Vec3 normal;
    Point2 uv;
    Float pdf;
};

class Shape : public Intersectable
{
public:
    // Sample random point on surface
    virtual ShapeSample Sample(Point2 u) const = 0;
    virtual Float PDF(const Intersection& isect) const = 0;

    // Sample random point relative to reference point
    virtual ShapeSample Sample(const Point3& ref, Point2 u) const = 0;
    virtual Float EvaluatePDF(const Ray& ray) const = 0;
    virtual Float PDF(const Intersection& isect, const Ray& isect_ray) const = 0;

    virtual Float Area() const = 0;

protected:
    static void SetFaceNormal(
        Intersection* isect, const Vec3& wi, const Vec3& outward_normal, const Vec3& shading_normal, const Vec3& shading_tangent
    )
    {
        bool front_face = Dot(wi, outward_normal) < 0;
        Float sign = front_face ? 1.0f : -1.0f;

        isect->front_face = front_face;
        isect->normal = sign * outward_normal;
        isect->shading.normal = sign * shading_normal;
        isect->shading.tangent = sign * shading_tangent;
    }
};

} // namespace bulbit
