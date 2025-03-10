#pragma once

#include "mesh.h"
#include "ray.h"
#include "shape.h"

namespace bulbit
{

class Sphere : public Shape
{
public:
    Sphere(const Point3& center, Float radius);
    Sphere(const Transform& transform, Float radius);

    virtual AABB GetAABB() const override;
    virtual bool Intersect(Intersection* out_isect, const Ray& ray, Float t_min, Float t_max) const override;
    virtual bool IntersectAny(const Ray& ray, Float t_min, Float t_max) const override;

    virtual ShapeSample Sample(Point2 u) const override;
    virtual Float PDF(const Intersection& isect) const override;

    virtual ShapeSample Sample(const Point3& ref, Point2 u) const override;
    virtual Float EvaluatePDF(const Ray& ray) const override;
    virtual Float PDF(const Intersection& isect, const Ray& isect_ray) const override;

    Transform transform;
    Float radius;

private:
    static Point2 ComputeTexCoord(const Vec3& v);
};

inline Sphere::Sphere(const Point3& center, Float radius)
    : transform{ center, identity }
    , radius{ radius }
{
}

inline Sphere::Sphere(const Transform& transform, Float radius)
    : transform{ transform }
    , radius{ radius }
{
}

inline AABB Sphere::GetAABB() const
{
    return AABB(transform.p - Vec3(radius), transform.p + Vec3(radius));
}

inline Float Sphere::PDF(const Intersection& isect) const
{
    BulbitNotUsed(isect);

    return 1 / (four_pi * radius * radius);
}

inline Float Sphere::EvaluatePDF(const Ray& ray) const
{
    Intersection isect;
    if (Intersect(&isect, ray, Ray::epsilon, infinity) == false)
    {
        return 0.0f;
    }

    return PDF(isect, ray);
}

inline Float Sphere::PDF(const Intersection& isect, const Ray& isect_ray) const
{
    BulbitNotUsed(isect);

    Float distance_squared = Length2(transform.p - isect_ray.o);
    Float cos_theta_max = std::sqrt(1 - radius * radius / distance_squared);
    Float solid_angle = two_pi * (1 - cos_theta_max);

    return 1 / solid_angle;
}

inline Point2 Sphere::ComputeTexCoord(const Vec3& v)
{
    Float theta = std::acos(Clamp(v.y, -1, 1));
    Float r = std::atan2(v.z, v.x);
    Float phi = r < 0 ? r + two_pi : r;

    return Point2(phi * inv_two_pi, 1 - theta * inv_pi);
}

class Triangle : public Shape
{
public:
    Triangle(const Mesh* mesh, size_t tri_index);

    virtual AABB GetAABB() const override;
    virtual bool Intersect(Intersection* out_isect, const Ray& ray, Float t_min, Float t_max) const override;
    virtual bool IntersectAny(const Ray& ray, Float t_min, Float t_max) const override;

    virtual ShapeSample Sample(Point2 u) const override;
    virtual Float PDF(const Intersection& isect) const override;

    virtual ShapeSample Sample(const Point3& ref, Point2 u) const override;
    virtual Float EvaluatePDF(const Ray& ray) const override;
    virtual Float PDF(const Intersection& isect, const Ray& isect_ray) const override;

private:
    friend class Scene;

    Vec3 GetNormal(Float u, Float v, Float w) const;
    Vec3 GetTangent(Float u, Float v, Float w) const;
    Point2 GetTexCoord(Float u, Float v, Float w) const;

    const Mesh* mesh;
    const int32* v;
};

inline Triangle::Triangle(const Mesh* mesh, size_t tri_index)
    : mesh{ mesh }
{
    v = &mesh->indices[tri_index * 3];
}

inline AABB Triangle::GetAABB() const
{
    const Vec3 aabb_offset{ epsilon * 10 };

    const Point3& p0 = mesh->positions[v[0]];
    const Point3& p1 = mesh->positions[v[1]];
    const Point3& p2 = mesh->positions[v[2]];

    Vec3 min = Min(Min(p0, p1), p2) - aabb_offset;
    Vec3 max = Max(Max(p0, p1), p2) + aabb_offset;

    return AABB(min, max);
}

inline Float Triangle::PDF(const Intersection& isect) const
{
    BulbitNotUsed(isect);

    const Point3& p0 = mesh->positions[v[0]];
    const Point3& p1 = mesh->positions[v[1]];
    const Point3& p2 = mesh->positions[v[2]];

    Vec3 e1 = p1 - p0;
    Vec3 e2 = p2 - p0;

    Float area = 0.5f * Length(Cross(e1, e2));
    return 1 / area;
}

inline Float Triangle::EvaluatePDF(const Ray& ray) const
{
    Intersection isect;
    if (Intersect(&isect, ray, Ray::epsilon, infinity) == false)
    {
        return 0;
    }

    return PDF(isect, ray);
}

inline Float Triangle::PDF(const Intersection& isect, const Ray& isect_ray) const
{
    Float distance_squared = isect.t * isect.t * Length2(isect_ray.d);
    Float cosine = std::fabs(Dot(isect_ray.d, isect.normal) / Length(isect_ray.d));

    const Point3& p0 = mesh->positions[v[0]];
    const Point3& p1 = mesh->positions[v[1]];
    const Point3& p2 = mesh->positions[v[2]];

    Vec3 e1 = p1 - p0;
    Vec3 e2 = p2 - p0;

    Float area = 0.5f * Length(Cross(e1, e2));

    return distance_squared / (cosine * area);
}

inline Vec3 Triangle::GetNormal(Float tu, Float tv, Float tw) const
{
    const Vec3& n0 = mesh->normals[v[0]];
    const Vec3& n1 = mesh->normals[v[1]];
    const Vec3& n2 = mesh->normals[v[2]];

    return Normalize(tw * n0 + tu * n1 + tv * n2);
}

inline Vec3 Triangle::GetTangent(Float tu, Float tv, Float tw) const
{
    const Vec3& t0 = mesh->tangents[v[0]];
    const Vec3& t1 = mesh->tangents[v[1]];
    const Vec3& t2 = mesh->tangents[v[2]];

    return Normalize(tw * t0 + tu * t1 + tv * t2);
}

inline Point2 Triangle::GetTexCoord(Float tu, Float tv, Float tw) const
{
    Point2 u0 = mesh->texCoords[v[0]];
    Point2 u1 = mesh->texCoords[v[1]];
    Point2 u2 = mesh->texCoords[v[2]];

    return tw * u0 + tu * u1 + tv * u2;
}

} // namespace bulbit
