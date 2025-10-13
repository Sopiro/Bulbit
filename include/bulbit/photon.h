#pragma once

#include "bounding_box.h"
#include "bsdf.h"
#include "common.h"
#include "hash.h"
#include "spectrum.h"

namespace bulbit
{

class Primitive;

struct Photon
{
    const Primitive* primitive;
    Point3 p;
    Vec3 normal;

    Vec3 wi;
    Spectrum beta;
};

struct VisiblePoint
{
    Float radius, radius_vol;

    const Primitive* primitive;
    Point3 p;
    Vec3 normal;

    Vec3 wo;
    BSDF bsdf;
    const PhaseFunction* phase;

    Spectrum Ld{ 0 };
    Spectrum beta{ 0 };

    Float n = 0;
    Float n_vol = 0;
    std::atomic<int32> m{ 0 };
    std::atomic<int32> m_vol{ 0 };

    std::atomic<Float> phi_i[3]{ 0, 0, 0 };
    std::atomic<Float> phi_i_vol[3]{ 0, 0, 0 };
    Spectrum tau{ 0 };
    Spectrum tau_vol{ 0 };
};

// Spatial hash grid implementation adapted from
// https://github.com/SmallVCM/SmallVCM/blob/master/src/hashgrid.hxx
class HashGrid
{
public:
    HashGrid() = default;

    template <typename T>
    void Build(const std::vector<T>& points, Float cell_size)
    {
        inv_cell_size = 1 / cell_size;

        int32 num_photons = int32(points.size());
        photon_indices.resize(num_photons);

        cell_ends.resize(num_photons);
        memset(&cell_ends[0], 0, num_photons * sizeof(int32));

        // Count how many points are there in the cell
        for (int32 i = 0; i < num_photons; ++i)
        {
            const T& p = points[i];
            int32 index = CellToIndex(PosToCell(p.p));
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
            const T& p = points[i];
            int32 index = CellToIndex(PosToCell(p.p));

            int32 insert_index = cell_ends[index]++;
            photon_indices[insert_index] = int32(i);
        }
        // Now cell_ends[i] points to the index right after the last element of cell x
    }

    template <typename T>
    void Query(const std::vector<T>& points, const Point3& position, Float radius, std::function<void(const T&)>&& callback) const
    {
        const Float radius2 = Sqr(radius);

        Point3i middle = PosToCell(position);
        int32 r = int32(std::ceil(radius * inv_cell_size));

        for (int32 dx = -r; dx <= r; ++dx)
        {
            for (int32 dy = -r; dy <= r; ++dy)
            {
                for (int32 dz = -r; dz <= r; ++dz)
                {
                    Point3i cell = middle + Point3i(dx, dy, dz);
                    int32 index = CellToIndex(cell);

                    int32 begin, end;
                    if (index == 0)
                    {
                        begin = 0;
                        end = cell_ends[0];
                    }
                    else
                    {
                        begin = cell_ends[index - 1];
                        end = cell_ends[index];
                    }

                    for (int32 i = begin; i < end; ++i)
                    {
                        int32 photon_index = photon_indices[i];
                        const T& p = points[photon_index];
                        if (Dist2(p.p, position) <= radius2)
                        {
                            callback(p);
                        }
                    }
                }
            }
        }
    }

    template <typename T>
    void Query(std::vector<T>& points, const Point3& position, std::function<void(T&)>&& callback) const
    {
        Point3i middle = PosToCell(position);
        const int32 r = 1;

        for (int32 dx = -r; dx <= r; ++dx)
        {
            for (int32 dy = -r; dy <= r; ++dy)
            {
                for (int32 dz = -r; dz <= r; ++dz)
                {
                    Point3i cell = middle + Point3i(dx, dy, dz);
                    int32 index = CellToIndex(cell);

                    int32 begin, end;
                    if (index == 0)
                    {
                        begin = 0;
                        end = cell_ends[0];
                    }
                    else
                    {
                        begin = cell_ends[index - 1];
                        end = cell_ends[index];
                    }

                    for (int32 i = begin; i < end; ++i)
                    {
                        int32 photon_index = photon_indices[i];
                        callback(points[photon_index]);
                    }
                }
            }
        }
    }

private:
    Point3i PosToCell(Point3 p) const
    {
        int32 x = int32(std::floor(p.x * inv_cell_size));
        int32 y = int32(std::floor(p.y * inv_cell_size));
        int32 z = int32(std::floor(p.z * inv_cell_size));

        return Point3i(x, y, z);
    }

    int32 CellToIndex(Point3i cell) const
    {
        return Hash(cell) % int32(cell_ends.size());
    }

    Float inv_cell_size;

    std::vector<int32> photon_indices;
    std::vector<int32> cell_ends;
};

} // namespace bulbit
