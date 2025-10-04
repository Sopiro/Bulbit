#pragma once

#include "bulbit/bulbit.h"

namespace bulbit
{

struct ModelLoaderOptions
{
    bool flip_normal = false;
    bool gen_smooth_normal = false;
    bool flip_tex_coord = false;
    bool use_fallback_material = false;

    const Material* fallback_material = nullptr;
    MediumInterface fallback_medium_interface = {};
};

void LoadModel(Scene& scene, std::filesystem::path filename, const Transform& transform, const ModelLoaderOptions& options = {});
void LoadGLTF(Scene& scene, std::filesystem::path filename, const Transform& transform, const ModelLoaderOptions& options = {});
void LoadOBJ(Scene& scene, std::filesystem::path filename, const Transform& transform, const ModelLoaderOptions& options = {});

} // namespace bulbit
