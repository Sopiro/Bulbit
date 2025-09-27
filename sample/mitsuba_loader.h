#pragma once

#include "bulbit/bulbit.h"
#include <optional>

namespace bulbit
{

struct MitsubaScene
{
    std::unique_ptr<Scene> scene;
    std::unique_ptr<Intersectable> accel;
    std::unique_ptr<Sampler> sampler;
    std::unique_ptr<Camera> camera;
    std::unique_ptr<Integrator> integrator;
};

std::optional<MitsubaScene> LoadMitsubaScene(std::filesystem::path filename);

} // namespace bulbit
