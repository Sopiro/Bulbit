#include "bulbit/frame.h"
#include "bulbit/material.h"
#include "bulbit/sampling.h"
#include "bulbit/shapes.h"

namespace bulbit
{

bool Sphere::Intersect(Intersection* isect, const Ray& ray, Float t_min, Float t_max) const
{
    Ray r = MulT(transform, ray);

    Point3 oc = r.o;
    Float a = Length2(r.d);
    Float half_b = Dot(oc, r.d);
    Float c = Length2(oc) - radius * radius;

    Float discriminant = half_b * half_b - a * c;
    if (discriminant < 0)
    {
        return false;
    }
    Float sqrt_d = std::sqrt(discriminant);

    // Find the nearest root that lies in the acceptable range.
    Float root = (-half_b - sqrt_d) / a;
    if (root < t_min || t_max < root)
    {
        root = (-half_b + sqrt_d) / a;
        if (root < t_min || t_max < root)
        {
            return false;
        }
    }

    // Found intersection

    Point3 point = r.At(root);
    Vec3 normal = Normalize(point / radius);

    isect->t = root;
    isect->point = Mul(transform, point);
    isect->uv = ComputeTexCoord(normal);

    Vec3 n = transform.q.Rotate(normal);
    Vec3 t;
    if (n.y < 1 - 1e-4f)
    {
        t = Normalize(Cross(y_axis, n));
    }
    else
    {
        t = Normalize(Cross(n, x_axis));
    }

    SetFaceNormal(isect, ray.d, n, n, t);

    return true;
}

bool Sphere::IntersectAny(const Ray& ray, Float t_min, Float t_max) const
{
    Ray r = MulT(transform, ray);

    Point3 oc = r.o;
    Float a = Length2(r.d);
    Float half_b = Dot(oc, r.d);
    Float c = Length2(oc) - radius * radius;

    Float discriminant = half_b * half_b - a * c;
    if (discriminant < 0)
    {
        return false;
    }
    Float sqrt_d = std::sqrt(discriminant);

    // Find the nearest root that lies in the acceptable range.
    Float root = (-half_b - sqrt_d) / a;
    if (root < t_min || t_max < root)
    {
        root = (-half_b + sqrt_d) / a;
        if (root < t_min || t_max < root)
        {
            return false;
        }
    }

    return true;
}

ShapeSample Sphere::Sample(Point2 u) const
{
    ShapeSample sample;
    sample.normal = SampleUniformSphere(u);
    sample.point = transform.p + sample.normal * radius;

    Float area = four_pi * radius * radius;
    sample.pdf = 1 / area;
    // sample.uv = ComputeTexCoord(sample.normal);

    return sample;
}

ShapeSample Sphere::Sample(const Point3& ref, Point2 u) const
{
    Vec3 direction = transform.p - ref;
    Float distance = direction.Normalize();
    Float distance_squared = distance * distance;

    Float cos_theta_max = std::sqrt(1 - radius * radius / distance_squared);
    Float z = 1 + u[1] * (cos_theta_max - 1);

    Float phi = two_pi * u[0];

    Float sin_theta = std::sqrt(1 - z * z);
    Float x = std::cos(phi) * sin_theta;
    Float y = std::sin(phi) * sin_theta;

    Vec3 d(x, y, z);

    Float s = distance * z - std::sqrt(radius * radius - distance_squared * sin_theta * sin_theta);

    Frame frame(direction);

    Vec3 ref2p = frame.FromLocal(d) * s;

    Float solid_angle = two_pi * (1 - cos_theta_max);

    ShapeSample sample;
    sample.point = ref + ref2p;
    sample.normal = Normalize(sample.point - transform.p);
    sample.pdf = 1 / solid_angle;

    return sample;
}

} // namespace bulbit