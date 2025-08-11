#pragma once

#include "shape.h"

namespace bulbit
{

struct Ray;
class Mesh;

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

} // namespace bulbit
