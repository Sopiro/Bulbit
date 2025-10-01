#pragma once

#include "bulbit/renderer_info.h"

namespace bulbit
{

// Load Mitsuba3 scene file
// Parser implementation is based on:
// https://github.com/BachiLi/lajolla_public/blob/main/src/parsers/parse_scene.cpp
bool LoadScene(RendererInfo* render_info, std::filesystem::path filename);

} // namespace bulbit
