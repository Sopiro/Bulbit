#include "bulbit/photon.h"
#include "bulbit/hash.h"
#include "bulbit/parallel_for.h"

namespace bulbit
{

static inline Point3i PosToCell(const Point3& p, Float cell_size)
{
    int32 x = int32(std::floor(p.x / cell_size));
    int32 y = int32(std::floor(p.y / cell_size));
    int32 z = int32(std::floor(p.z / cell_size));
    return Point3i(x, y, z);
}

void PhotonMap::Build(const std::vector<Photon>& photons, Float gather_radius)
{
    cell_size = gather_radius;

    size_t num_photons = photons.size();
    photon_indices.resize(num_photons);

    cell_ends.resize(num_photons);
    memset(&cell_ends[0], 0, num_photons * sizeof(int32));

    // Count how many photons are there in the cell
    for (size_t i = 0; i < num_photons; ++i)
    {
        const Photon& p = photons[i];
        Point3i cell = PosToCell(p.position, cell_size);
        size_t index = Hash(cell) % num_photons;

        cell_ends[index]++;
    }

    // Run exclusive prefix sum
    int32 sum = 0;
    for (size_t i = 0; i < num_photons; ++i)
    {
        int32 count = cell_ends[i];
        cell_ends[i] = sum;
        sum += count;
    }
    // Now cell_ends[i] indicates cell's starting index

    for (size_t i = 0; i < num_photons; ++i)
    {
        const Photon& p = photons[i];
        Point3i cell = PosToCell(p.position, cell_size);
        size_t index = Hash(cell) % num_photons;

        int32 insert_index = cell_ends[index]++;
        photon_indices[insert_index] = int32(i);
    }
    // Now cell_ends[i] points to the index right after the last element of cell x
}

void PhotonMap::Query(
    const std::vector<Photon>& photons, const Point3& pos, Float radius, std::function<void(const Photon&)>&& callback
) const
{
    const Float radius2 = Sqr(radius);

    Point3i middle = PosToCell(pos, cell_size);
    int32 r = int32(std::ceil(radius / cell_size));

    for (int32 dx = -r; dx <= r; ++dx)
    {
        for (int32 dy = -r; dy <= r; ++dy)
        {
            for (int32 dz = -r; dz <= r; ++dz)
            {
                Point3i cell = middle + Point3i(dx, dy, dz);
                size_t index = Hash(cell) % photons.size();

                int32 begin = cell_ends[index - 1];
                int32 end = cell_ends[index];

                for (int32 i = begin; i < end; ++i)
                {
                    int32 photon_index = photon_indices[i];
                    const Photon& p = photons[photon_index];
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
