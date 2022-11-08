#pragma once

#include "common.h"
#include "hittable.h"
#include "ray.h"

class Sphere : public Hittable
{
public:
    Sphere() = default;
    Sphere(Vec3 c, double r, std::shared_ptr<Material> m)
        : center{ c }
        , radius{ r }
        , material{ m } {};

    virtual bool Hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const override;

public:
    Vec3 center;
    double radius;
    std::shared_ptr<Material> material;
};

bool Sphere::Hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const
{
    Vec3 oc = r.origin - center;
    double a = r.dir.Length2();
    double half_b = Dot(oc, r.dir);
    double c = oc.Length2() - radius * radius;

    double discriminant = half_b * half_b - a * c;
    if (discriminant < 0)
    {
        return false;
    }
    double sqrt_d = sqrt(discriminant);

    // Find the nearest root that lies in the acceptable range.
    double root = (-half_b - sqrt_d) / a;
    if (root < t_min || t_max < root)
    {
        root = (-half_b + sqrt_d) / a;
        if (root < t_min || t_max < root)
        {
            return false;
        }
    }

    rec.t = root;
    rec.p = r.At(rec.t);
    Vec3 outward_normal = (rec.p - center) / radius;
    rec.SetFaceNormal(r, outward_normal);
    rec.mat = material;

    return true;
}