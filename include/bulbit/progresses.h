#pragma once

#include "progress.h"

namespace bulbit
{

class SinglePhaseRendering : public Rendering
{
public:
    SinglePhaseRendering(const Camera* camera, int32 tile_size)
        : Rendering(camera)
        , tile_size{ tile_size }
        , num_tiles_done{ 0 }
        , done{ false }
    {
        Point2i res = camera->GetScreenResolution();
        int32 num_tiles_x = (res.x + tile_size - 1) / tile_size;
        int32 num_tiles_y = (res.y + tile_size - 1) / tile_size;

        tile_count = num_tiles_x * num_tiles_y;
    }

    virtual bool IsDone() const override
    {
        return done.load();
    }

    virtual void LogProgress() const override
    {
        int32 t = num_tiles_done.load();
        float p = 100.0f * t / tile_count;
        std::fprintf(stdout, "\rRendering.. %.2f%% [%d/%d]", p, t, tile_count);
    }

    int32 GetTileCount() const
    {
        return tile_count;
    }

    int32 GetTileSize() const
    {
        return tile_size;
    }

    int32 GetNumTilesDone() const
    {
        return num_tiles_done.load();
    }

private:
    friend class UniDirectionalRayIntegrator;
    friend class BiDirectionalRayIntegrator;

    int32 tile_size;
    int32 tile_count;

    std::atomic<int32> num_tiles_done;
    std::atomic<bool> done;
};

class MultiPhaseRendering : public Rendering
{
public:
    MultiPhaseRendering(const Camera* camera, std::span<size_t> num_phase_works)
        : Rendering(camera)
        , num_phases{ num_phase_works.size() }
        , num_phase_works{ num_phase_works.begin(), num_phase_works.end() }
        , phase_works_dones(new std::atomic<size_t>[num_phases])
        , phase_dones(new std::atomic<bool>[num_phases])
    {
        for (size_t i = 0; i < num_phases; ++i)
        {
            phase_works_dones[i].store(0);
            phase_dones[i].store(false);
        }
    }

    virtual bool IsDone() const override
    {
        for (size_t i = 0; i < num_phases; ++i)
        {
            if (!phase_dones[i].load())
            {
                return false;
            }
        }

        return true;
    }

    virtual void LogProgress() const override
    {
        static size_t current_phase = 0;
        if (phase_dones[current_phase])
        {
            ++current_phase;
            std::fprintf(stdout, "\n");
        }

        size_t t = phase_works_dones[current_phase].load();
        size_t total_works = num_phase_works[current_phase];
        float p = 100.0f * t / num_phase_works[current_phase];
        std::fprintf(stdout, "\rPhase[%zu] Rendering.. %.2f%% [%zu/%zu]", current_phase, p, t, total_works);
    }

    size_t NumPhases() const
    {
        return num_phases;
    }

private:
    friend class PhotonMappingIntegrator;

    size_t num_phases;
    std::vector<size_t> num_phase_works;

    std::unique_ptr<std::atomic<size_t>[]> phase_works_dones;
    std::unique_ptr<std::atomic<bool>[]> phase_dones;
};

} // namespace bulbit
