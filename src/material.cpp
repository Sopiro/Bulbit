#include "raytracer/material.h"
#include "raytracer/hittable.h"

bool Lambertian::Scatter(const Ray& r_in, const HitRecord& rec, Color& attenuation, Ray& scattered) const
{
    Vec3 scatter_direction = rec.normal + RandomUnitVector();

    // Catch degenerate scatter direction
    if (scatter_direction.Length2() < DBL_EPSILON * DBL_EPSILON)
    {
        scatter_direction = rec.normal;
    }

    scattered = Ray{ rec.p, scatter_direction };
    attenuation = albedo;

    return true;
}

bool Metal::Scatter(const Ray& r_in, const HitRecord& rec, Color& attenuation, Ray& scattered) const
{
    Vec3 reflected = Reflect(r_in.dir.Normalized(), rec.normal);
    scattered = Ray{ rec.p, reflected + fuzziness * RandomInUnitSphere() };
    attenuation = albedo;

    return Dot(scattered.dir, rec.normal) > 0;
}