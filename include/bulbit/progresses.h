#pragma once

#include "progress.h"

namespace bulbit
{

class SinglePhaseRendering : public Rendering
{
public:
    SinglePhaseRendering(const Camera* camera, size_t works)
        : Rendering(camera)
        , works{ works }
        , work_dones{ 0 }
        , done{ false }
    {
    }

    virtual bool IsDone() const override
    {
        return done.load(std::memory_order_acquire);
    }

    virtual void LogProgress() const override
    {
        size_t t = work_dones.load(std::memory_order_relaxed);
        float p = 100.0f * t / works;
        std::fprintf(stdout, "\rRendering.. %.2f%% [%zu/%zu]", p, t, works);
    }

private:
    friend class UniDirectionalRayIntegrator;
    friend class BiDirectionalRayIntegrator;
    friend class ReSTIRDIIntegrator;
    friend class ReSTIRPTIntegrator;

    size_t works;

    std::atomic<size_t> work_dones;
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
            if (!phase_dones[i].load(std::memory_order_acquire))
            {
                return false;
            }
        }

        return true;
    }

    virtual void LogProgress() const override
    {
        static size_t current_phase = 0;
        while (phase_dones[current_phase].load(std::memory_order_acquire))
        {
            ++current_phase;
        }

        size_t t = phase_works_dones[current_phase].load(std::memory_order_relaxed);
        size_t total_works = num_phase_works[current_phase];
        float p = 100.0f * t / num_phase_works[current_phase];
        std::fprintf(
            stdout, "\rPhase[%zu/%zu] Rendering.. %.2f%% [%zu/%zu]\033[K", current_phase + 1, num_phases, p, t, total_works
        );
    }

    size_t NumPhases() const
    {
        return num_phases;
    }

private:
    friend class PhotonMappingIntegrator;
    friend class VolPhotonMappingIntegrator;
    friend class SPPMIntegrator;
    friend class VolSPPMIntegrator;

    size_t num_phases;
    std::vector<size_t> num_phase_works;

    std::unique_ptr<std::atomic<size_t>[]> phase_works_dones;
    std::unique_ptr<std::atomic<bool>[]> phase_dones;
};

} // namespace bulbit
