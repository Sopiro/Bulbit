#pragma once

#include "bulbit/bulbit.h"

namespace bulbit
{

void SetLoaderFlipNormal(bool flip_normal);
void SetLoaderFlipTexcoord(bool flip_texcoord);
void SetLoaderUseForceFallbackMaterial(bool force_use_fallback_material);
void SetLoaderFallbackMaterial(const Material* fallback_material);
void SetLoaderFallbackMediumInterface(const MediumInterface& fallback_medium_interface);

void LoadModel(Scene& scene, std::filesystem::path filename, const Transform& transform);
void LoadGLTF(Scene& scene, std::filesystem::path filename, const Transform& transform);
void LoadOBJ(Scene& scene, std::filesystem::path filename, const Transform& transform);

} // namespace bulbit
