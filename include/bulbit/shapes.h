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

    virtual Float Area() const override;

    Transform transform;
    Float radius;
};

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

    virtual Float Area() const override;

private:
    friend class Scene;

    Vec3 GetNormal(Float u, Float v, Float w) const;
    Vec3 GetTangent(Float u, Float v, Float w) const;
    Point2 GetTexCoord(Float u, Float v, Float w) const;

    const Mesh* mesh;
    const int32* v;
};

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
