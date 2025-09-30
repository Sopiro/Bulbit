#pragma once

#include "bulbit/bulbit.h"

#include "light_builder.h"
#include "material_builder.h"
#include "model_loader.h"
#include "scene_builder.h"
#include "scene_loader.h"
#include "texture_builder.h"

#include <map>

using namespace bulbit;

struct Sample
{
    using Func = std::function<SceneInfo(void)>;

    static int32 Register(std::string name, Func func);
    static SceneInfo Get(std::string name);

    static inline std::map<std::string, Func> samples;
    static inline int32 count = 0;
};

inline int32 Sample::Register(std::string name, Func func)
{
    samples.insert(std::make_pair(name, func));

    return ++count;
}

inline SceneInfo Sample::Get(std::string name)
{
    if (!samples.contains(name))
    {
        return {};
    }

    return samples.at(name)();
}
