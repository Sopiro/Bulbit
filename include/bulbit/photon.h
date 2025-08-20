#pragma once

#include "common.h"
#include "hash.h"
#include "spectrum.h"

namespace bulbit
{

class Primitive;

struct Photon
{
    const Primitive* primitive;
    Point3 position;
    Vec3 normal;

    Vec3 wi;
    Spectrum beta;
};

// Hash grid
class PhotonMap
{
public:
    PhotonMap() = default;

    void Build(std::vector<Photon>&& photons, Float gather_radius);
    void Query(const Vec3& position, Float radius, std::function<void(const Photon&)>&& callback) const;

private:
    std::vector<Photon> photons;
    Float cell_size;

    struct PhotonRange
    {
        size_t begin;
        size_t count;
    };

    struct Hasher
    {
        size_t operator()(const Point3i& v) const
        {
            return Hash(v);
        }
    };

    std::unordered_map<Point3i, PhotonRange, Hasher> photon_ranges;
};

} // namespace bulbit
