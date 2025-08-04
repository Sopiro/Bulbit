#include "bulbit/shapes.h"

namespace bulbit
{

Triangle::Triangle(const Mesh* mesh, size_t tri_index)
    : mesh{ mesh }
{
    v = &mesh->indices[tri_index * 3];
}

AABB Triangle::GetAABB() const
{
    const Vec3 aabb_offset{ epsilon * 10 };

    const Point3& p0 = mesh->positions[v[0]];
    const Point3& p1 = mesh->positions[v[1]];
    const Point3& p2 = mesh->positions[v[2]];

    Vec3 min = Min(Min(p0, p1), p2) - aabb_offset;
    Vec3 max = Max(Max(p0, p1), p2) + aabb_offset;

    return AABB(min, max);
}

// Möller-Trumbore algorithm
bool Triangle::Intersect(Intersection* isect, const Ray& ray, Float t_min, Float t_max) const
{
    const Point3& p0 = mesh->positions[v[0]];
    const Point3& p1 = mesh->positions[v[1]];
    const Point3& p2 = mesh->positions[v[2]];

    Vec3 e1 = p1 - p0;
    Vec3 e2 = p2 - p0;

    Vec3 d = ray.d;
    Float l = d.Normalize();
    Vec3 pvec = Cross(d, e2);

    Float det = Dot(e1, pvec);
    // bool backface = det < epsilon;

    // Ray and triangle are parallel
    if (std::fabs(det) < epsilon)
    {
        return false;
    }

    Float invDet = 1 / det;

    Vec3 tvec = ray.o - p0;
    Float u = Dot(tvec, pvec) * invDet;
    if (u < 0 || u > 1)
    {
        return false;
    }

    Vec3 qvec = Cross(tvec, e1);
    Float v = Dot(d, qvec) * invDet;
    if (v < 0 || u + v > 1)
    {
        return false;
    }

    Float t = Dot(e2, qvec) * invDet / l;
    if (t < t_min || t > t_max)
    {
        return false;
    }

    // Found intersection
    Float w = 1 - u - v;

    isect->t = t;
    isect->point = ray.At(t);
    isect->uv = GetTexCoord(u, v, w);

    Vec3 normal = Normalize(Cross(e1, e2));
    SetFaceNormal(isect, ray.d, normal, GetNormal(u, v, w), GetTangent(u, v, w));

    return true;
}

bool Triangle::IntersectAny(const Ray& ray, Float t_min, Float t_max) const
{
    const Point3& p0 = mesh->positions[v[0]];
    const Point3& p1 = mesh->positions[v[1]];
    const Point3& p2 = mesh->positions[v[2]];

    Vec3 e1 = p1 - p0;
    Vec3 e2 = p2 - p0;

    Vec3 d = ray.d;
    Float l = d.Normalize();
    Vec3 pvec = Cross(d, e2);

    Float det = Dot(e1, pvec);
    // bool backface = det < epsilon;

    // Ray and triangle are parallel
    if (std::fabs(det) < epsilon)
    {
        return false;
    }

    Float invDet = 1 / det;

    Vec3 tvec = ray.o - p0;
    Float u = Dot(tvec, pvec) * invDet;
    if (u < 0 || u > 1)
    {
        return false;
    }

    Vec3 qvec = Cross(tvec, e1);
    Float v = Dot(d, qvec) * invDet;
    if (v < 0 || u + v > 1)
    {
        return false;
    }

    Float t = Dot(e2, qvec) * invDet / l;
    if (t < t_min || t > t_max)
    {
        return false;
    }

    return true;
}

ShapeSample Triangle::Sample(Point2 u0) const
{
    const Point3& p0 = mesh->positions[v[0]];
    const Point3& p1 = mesh->positions[v[1]];
    const Point3& p2 = mesh->positions[v[2]];

    Vec3 e1 = p1 - p0;
    Vec3 e2 = p2 - p0;

#if 1
    Float u = u0[0];
    Float v = u0[1];

    if (u + v > 1)
    {
        u = 1 - u;
        v = 1 - v;
    }

    ShapeSample sample;
    sample.normal = Cross(e1, e2);
    sample.point = p0 + e1 * u + e2 * v;

    Float area = sample.normal.Normalize() * 0.5f;
    sample.pdf = 1 / area;

    Float w = 1 - u - v;
    sample.uv = GetTexCoord(u, v, w);

    return sample;
#else
    Float u1 = u0[0];
    Float u2 = u0[1];

    Float s = std::sqrt(u1);
    Float u = 1.0 - s;
    Float v = u2 * s;

    ShapeSample sample;
    sample.normal = Cross(e1, e2);
    sample.point = p0 + e1 * u + e2 * v;

    Float area = sample.normal.Normalize() * 0.5f;
    sample.pdf = 1 / area;

    Float w = 1 - u - v;
    sample.uv = GetTexCoord(u, v, w);

    return sample;
#endif
}

Float Triangle::PDF(const Intersection& isect) const
{
    BulbitNotUsed(isect);

    return 1 / Area();
}

ShapeSample Triangle::Sample(const Point3& ref, Point2 u) const
{
    ShapeSample sample = Sample(u);

    Vec3 d = sample.point - ref;
    Float distance_squared = Dot(d, d);

    Float cosine = Dot(d, sample.normal) / std::sqrt(distance_squared);
    if (cosine < 0)
    {
        cosine = -cosine;
    }

    sample.pdf *= distance_squared / cosine; // Convert to solid angle measure

    return sample;
}

Float Triangle::EvaluatePDF(const Ray& ray) const
{
    Intersection isect;
    if (Intersect(&isect, ray, Ray::epsilon, infinity) == false)
    {
        return 0;
    }

    return PDF(isect, ray);
}

Float Triangle::PDF(const Intersection& isect, const Ray& isect_ray) const
{
    Float distance_squared = isect.t * isect.t * Length2(isect_ray.d);
    Float cosine = std::fabs(Dot(isect_ray.d, isect.normal) / Length(isect_ray.d));

    return distance_squared / (cosine * Area());
}

Float Triangle::Area() const
{
    const Point3& p0 = mesh->positions[v[0]];
    const Point3& p1 = mesh->positions[v[1]];
    const Point3& p2 = mesh->positions[v[2]];

    Vec3 e1 = p1 - p0;
    Vec3 e2 = p2 - p0;

    return 0.5f * Length(Cross(e1, e2));
}

} // namespace bulbit