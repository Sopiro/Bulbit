#pragma once

#include "common.h"
#include "hittable.h"
#include "material.h"

class ConstantDensityMedium : public Hittable
{
public:
    ConstantDensityMedium(std::shared_ptr<Hittable> boundary_object, double density, std::shared_ptr<Texture> albedo)
        : boundary{ boundary_object }
        , neg_inv_density{ -1.0 / density }
        , phase_function{ std::make_shared<Isotropic>(albedo) }
    {
    }

    ConstantDensityMedium(std::shared_ptr<Hittable> boundary_object, double density, Color color)
        : boundary{ boundary_object }
        , neg_inv_density{ -1.0 / density }
        , phase_function{ std::make_shared<Isotropic>(color) }
    {
    }

    virtual bool Hit(const Ray& ray, double t_min, double t_max, HitRecord& rec) const override;
    virtual bool GetAABB(AABB& outAABB) const override;

public:
    std::shared_ptr<Hittable> boundary;
    std::shared_ptr<Material> phase_function;
    double neg_inv_density;
};

inline bool ConstantDensityMedium::GetAABB(AABB& outAABB) const
{
    return boundary->GetAABB(outAABB);
}