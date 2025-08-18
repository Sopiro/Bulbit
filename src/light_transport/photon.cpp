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
        size_t hash1 = Hash(cell1);
        size_t hash2 = Hash(cell2);

        return hash1 < hash2;
    });

    size_t i0 = 0;
    size_t prev = 0;
    for (size_t i = 0; i < photons.size(); ++i)
    {
        const Photon& p = photons[i];
        Vec3i cell = PosToCell(p.position, cell_size);
        size_t hash = Hash(cell);
        if (hash_to_begin.contains(hash))
        {
            continue;
        }

        if (i0 > 0)
        {
            hash_to_begin[prev].count = i - i0;
        }

        PhotonRange range{ i, 0 };
        hash_to_begin[hash] = range;
        i0 = i;
        prev = hash;
    }
    hash_to_begin[prev].count = photons.size() - i0;
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
                Vec3i cell(middle.x + dx, middle.y + dy, middle.z + dz);
                size_t hash = Hash(cell);
                if (!hash_to_begin.contains(hash))
                {
                    continue;
                }

                PhotonRange range = hash_to_begin.at(hash);
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
