#include "bulbit/photon.h"
#include "bulbit/hash.h"

namespace bulbit
{

static inline Vec3i PosToCell(const Vec3& p, Float cell_size)
{
    int32 x = int32(std::floor(p.x / cell_size));
    int32 y = int32(std::floor(p.y / cell_size));
    int32 z = int32(std::floor(p.z / cell_size));
    return Vec3i(x, y, z);
}

void PhotonMap::Store(const Photon& photon)
{
    photons.push_back(photon);
}

void PhotonMap::Build(Float gather_radius)
{
    cell_size = gather_radius;
    cells.clear();

    for (size_t i = 0; i < photons.size(); ++i)
    {
        Vec3i cell = PosToCell(photons[i].position, cell_size);

        size_t hash = Hash(cell);
        cells[hash].push_back(i);
    }
}

void PhotonMap::Query(const Vec3& pos, Float radius, std::function<void(const Photon&)> callback) const
{
    const Float radius2 = Sqr(radius);

    Vec3i middle = PosToCell(pos, cell_size);
    int32 r = int32(std::ceil(radius / cell_size));

    for (int32 dx = -r; dx <= r; ++dx)
    {
        for (int32 dy = -r; dy <= r; ++dy)
        {
            for (int32 dz = -r; dz <= r; ++dz)
            {
                Vec3i cell(middle.x + dx, middle.y + dy, middle.z + dz);

                size_t hash = Hash(cell);
                if (!cells.contains(hash))
                {
                    continue;
                }

                const std::vector<int32>& photon_indices = cells.at(hash);
                for (int32 idx : photon_indices)
                {
                    const Photon& p = photons[idx];
                    if (Dist2(p.position, pos) <= radius2)
                    {
                        callback(p);
                    }
                }
            }
        }
    }
}

} // namespace bulbit
