#pragma once

#include "common.h"
#include "hittable.h"

class HittableList : public Hittable
{
public:
    HittableList() = default;
    HittableList(std::shared_ptr<Hittable> object)
    {
        add(object);
    }

    void clear()
    {
        objects.clear();
    }
    void add(std::shared_ptr<Hittable> object)
    {
        objects.push_back(object);
    }

    virtual bool Hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const override;

    std::vector<std::shared_ptr<Hittable>> objects;
};

bool HittableList::Hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const
{
    HitRecord tmp;
    bool hit = false;
    double closest = t_max;

    for (const auto& object : objects)
    {
        if (object->Hit(r, t_min, closest, tmp))
        {
            hit = true;
            closest = tmp.t;
            rec = tmp;
        }
    }

    return hit;
}