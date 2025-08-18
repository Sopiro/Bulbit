#pragma once

#include "async_job.h"
#include "camera.h"
#include "film.h"

namespace bulbit
{

class Rendering
{
public:
    Rendering(const Camera* camera)
        : camera{ camera }
        , film(camera)
        , job{ nullptr }
    {
    }

    virtual ~Rendering() = default;

    virtual bool IsDone() const = 0;
    virtual void LogProgress() const = 0;

    bool Start(std::unique_ptr<AsyncJob<bool>> rendering);
    void Wait() const;
    void WaitAndLogProgress() const;
    const Film& GetFilm() const;

    const Camera* camera;

protected:
    Film film;

    std::unique_ptr<AsyncJob<bool>> job;
};

inline bool Rendering::Start(std::unique_ptr<AsyncJob<bool>> rendering)
{
    if (!job)
    {
        job = std::move(rendering);
        return true;
    }
    else
    {
        return false;
    }
}

inline void Rendering::Wait() const
{
    job->Wait();
}

inline void Rendering::WaitAndLogProgress() const
{
    while (!IsDone())
    {
        LogProgress();

        using namespace std::chrono_literals;
        std::this_thread::sleep_for(50ms);
    }
}

inline const Film& Rendering::GetFilm() const
{
    return film;
}

} // namespace bulbit
