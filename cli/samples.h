#pragma once

#include "bulbit/bulbit.h"

#include "light_builder.h"
#include "material_builder.h"
#include "model_loader.h"
#include "scene_builder.h"
#include "scene_loader.h"
#include "texture_builder.h"
#include "util.h"

#include <map>

using namespace bulbit;

struct Sample
{
    using Func = std::function<void(RendererInfo*)>;

    static int32 Register(std::string name, Func func);
    static bool Get(RendererInfo* renderer_info, std::string name);

    static inline std::map<std::string, Func> samples;

private:
    static inline int32 count = 0;
};

inline int32 Sample::Register(std::string name, Func func)
{
    samples.insert(std::make_pair(name, func));

    return ++count;
}

inline bool Sample::Get(RendererInfo* renderer_info, std::string name)
{
    auto iter = samples.find(name);
    if (iter == samples.end())
    {
        return false;
    }

    iter->second(renderer_info);
    return true;
}

inline constexpr std::array<std::string_view, size_t(IntegratorType::count)> integrator_list = {
    "Path tracing",
    "Volumetric path tracing",
    "Light path tracing",
    "Volumetric light path tracing",
    "Bidirectional path tracing",
    "Volumetric bidirectional path tracing",
    "Photon mapping",
    "Volumetric photon mapping",
    "Stochastic progressive photon mapping",
    "Naive path tracing",
    "Naive volumetric path tracing",
    "Random walk ray tracing",
    "Ambient occlusion",
    "Albedo",
    "Debug",
};