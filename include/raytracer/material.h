#pragma once

#include "common.h"
#include "ray.h"

struct HitRecord;

class Material
{
public:
    virtual bool Scatter(const Ray& r_in, const HitRecord& rec, Color& attenuation, Ray& scattered) const = 0;
};

class Lambertian : public Material
{
public:
    Lambertian(const Color& a)
        : albedo{ a }
    {
    }

    virtual bool Scatter(const Ray& r_in, const HitRecord& rec, Color& attenuation, Ray& scattered) const override;

public:
    Color albedo;
};

class Metal : public Material
{
public:
    Metal(const Color& a, double f)
        : albedo{ a }
        , fuzziness{ f }
    {
    }

    virtual bool Scatter(const Ray& r_in, const HitRecord& rec, Color& attenuation, Ray& scattered) const override;

public:
    Color albedo;
    double fuzziness;
};