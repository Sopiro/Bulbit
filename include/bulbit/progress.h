#pragma once

#include "async_job.h"

namespace bulbit
{

class RenderingProgress
{
public:
    RenderingProgress(const Camera* camera, int32 tile_size)
        : camera{ camera }
        , film(camera)
        , tile_size{ tile_size }
        , tile_done{ 0 }
        , done{ false }
    {
        Point2i res = camera->GetScreenResolution();
        int32 num_tiles_x = (res.x + tile_size - 1) / tile_size;
        int32 num_tiles_y = (res.y + tile_size - 1) / tile_size;

        tile_count = num_tiles_x * num_tiles_y;
    }

    ~RenderingProgress() = default;

    const Film& Wait() const
    {
        job->Wait();
        return film;
    }

    const Film& WaitAndLogProgress() const
    {
        while (!IsDone())
        {
            using namespace std::chrono_literals;

            int32 t = tile_done.load();
            float p = 100.0f * t / tile_count;
            std::fprintf(stdout, "\rRendering.. %.2f%% [%d/%d]", p, t, tile_count);
            std::this_thread::sleep_for(50ms);
        }

        BulbitAssert(done);
        return film;
    }

    int32 GetTileCount() const
    {
        return tile_count;
    }

    int32 GetTileSize() const
    {
        return tile_size;
    }

    int32 GetNumTileDone() const
    {
        return tile_done.load();
    }

    int32 IsDone() const
    {
        return done.load();
    }

    const Film& GetFilm() const
    {
        return film;
    }

private:
    friend class UniDirectionalRayIntegrator;
    friend class BiDirectionalRayIntegrator;

    const Camera* camera;
    Film film;

    int32 tile_size;
    int32 tile_count;

    std::atomic<int32> tile_done;
    std::atomic<bool> done;

    std::unique_ptr<AsyncJob<bool>> job;
};

} // namespace bulbit
