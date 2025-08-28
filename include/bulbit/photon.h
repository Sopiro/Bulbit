#pragma once

#include "bounding_box.h"
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

    void Build(const std::vector<Photon>& photons, Float gather_radius);
    void Query(
        const std::vector<Photon>& photons, const Point3& position, Float radius, std::function<void(const Photon&)>&& callback
    ) const;

private:
    Point3i PosToCell(Point3 p) const;
    int32 CellToIndex(Point3i cell) const;

    Float inv_cell_size;

    std::vector<int32> photon_indices;
    std::vector<int32> cell_ends;
};

} // namespace bulbit
