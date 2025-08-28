#include "bulbit/photon.h"
#include "bulbit/hash.h"
#include "bulbit/parallel_for.h"

namespace bulbit
{

Point3i PhotonMap::PosToCell(Point3 p) const
{
    int32 x = int32(std::floor(p.x * inv_cell_size));
    int32 y = int32(std::floor(p.y * inv_cell_size));
    int32 z = int32(std::floor(p.z * inv_cell_size));

    return Point3i(x, y, z);
}

int32 PhotonMap::CellToIndex(Point3i cell) const
{
    return Hash(cell) % int32(cell_ends.size());
}

void PhotonMap::Build(const std::vector<Photon>& photons, Float gather_radius)
{
    inv_cell_size = 1 / gather_radius;

    int32 num_photons = int32(photons.size());
    photon_indices.resize(num_photons);

    cell_ends.resize(num_photons);
    memset(&cell_ends[0], 0, num_photons * sizeof(int32));

    // Count how many photons are there in the cell
    for (int32 i = 0; i < num_photons; ++i)
    {
        const Photon& p = photons[i];
        int32 index = CellToIndex(PosToCell(p.position));
        cell_ends[index]++;
    }

    // Run exclusive prefix sum
    int32 sum = 0;
    for (int32 i = 0; i < num_photons; ++i)
    {
        int32 count = cell_ends[i];
        cell_ends[i] = sum;
        sum += count;
    }
    // Now cell_ends[i] indicates cell's starting index

    for (int32 i = 0; i < num_photons; ++i)
    {
        const Photon& p = photons[i];
        int32 index = CellToIndex(PosToCell(p.position));

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

    Point3i middle = PosToCell(pos);
    int32 r = int32(std::ceil(radius * inv_cell_size));

    for (int32 dx = -r; dx <= r; ++dx)
    {
        for (int32 dy = -r; dy <= r; ++dy)
        {
            for (int32 dz = -r; dz <= r; ++dz)
            {
                Point3i cell = middle + Point3i(dx, dy, dz);
                int32 index = CellToIndex(cell);

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
