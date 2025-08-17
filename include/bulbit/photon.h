#pragma once

#include "common.h"
#include "spectrum.h"

namespace bulbit
{

struct Photon
{
    Vec3 position;
    Vec3 wi;
    Spectrum beta;
};

// Hash grid
class PhotonMap
{
public:
    PhotonMap() = default;

    void Store(const Photon& photon);
    void Build(Float gather_radius);
    void Query(const Vec3& position, Float radius, std::function<void(const Photon&)> callback) const;

private:
    std::vector<Photon> photons;
    std::unordered_map<size_t, std::vector<int32>> cells;
    Float cell_size;
};

} // namespace bulbit
