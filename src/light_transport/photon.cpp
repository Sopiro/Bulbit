#include "bulbit/photon.h"
#include "bulbit/hash.h"
#include "bulbit/parallel_for.h"

namespace bulbit
{

static inline Vec3i PosToCell(const Vec3& p, Float cell_size)
{
    int32 x = int32(std::floor(p.x / cell_size));
    int32 y = int32(std::floor(p.y / cell_size));
    int32 z = int32(std::floor(p.z / cell_size));
    return Vec3i(x, y, z);
}

void PhotonMap::Build(std::vector<Photon>&& ps, Float gather_radius)
{
    photons = std::move(ps);
    cell_size = gather_radius;

    std::sort(std::execution::par, photons.begin(), photons.end(), [&](const Photon& p1, const Photon& p2) {
        Vec3i cell1 = PosToCell(p1.position, cell_size);
        Vec3i cell2 = PosToCell(p2.position, cell_size);
        return Hash(cell1) < Hash(cell2);
    });

    photon_ranges.clear();
    Point3i current_cell = PosToCell(photons[0].position, cell_size);
    size_t begin = 0;
    for (size_t i = 1; i < photons.size(); ++i)
    {
        Point3i cell = PosToCell(photons[i].position, cell_size);
        if (cell != current_cell)
        {
            photon_ranges[current_cell] = { begin, i - begin };
            current_cell = cell;
            begin = i;
        }
    }
    photon_ranges[current_cell] = { begin, photons.size() - begin };
}

void PhotonMap::Query(const Vec3& pos, Float radius, std::function<void(const Photon&)>&& callback) const
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
                Vec3i cell = middle + Vec3i(dx, dy, dz);

                auto it = photon_ranges.find(cell);
                if (it == photon_ranges.end())
                {
                    continue;
                }

                const PhotonRange& range = it->second;
                for (size_t i = range.begin; i < range.begin + range.count; ++i)
                {
                    const Photon& p = photons[i];
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
