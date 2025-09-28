#pragma once

#include "bulbit/bulbit.h"

namespace bulbit
{

struct FilmInfo
{
    std::string filename;
    Point2i resolution;
};

enum class SamplerType
{
    independent,
    stratified,
};

struct SamplerInfo
{
    SamplerType type;
    int32 spp;
};

enum class CameraType
{
    perspective,
    orthographic,
    spherical,
};

struct CameraInfo
{
    CameraType type;

    Transform tf;
    Float fov;
    Float aperture;
    Float focus_distance;

    FilmInfo film_info;
    SamplerInfo sampler_info;
};

enum class IntegratorType
{
    path,
    vol_path,
    light_path,
    light_vol_path,
    bdpt,
    vol_bdpt,
    ao,
    albedo,
    debug,
    pm,
    sppm,
};

struct RendererInfo
{
    IntegratorType type;
    int32 max_bounces;
    int32 rr_depth;
    Float ao_range;
    int32 n_photons;
    Float initial_radius;
};

struct SceneInfo
{
    std::unique_ptr<Scene> scene;

    CameraInfo camera_info;
    RendererInfo renderer_info;

    operator bool() const;
};

// Load Mitsuba3 scene file
// Parser implementation is based on:
// https://github.com/BachiLi/lajolla_public/blob/main/src/parsers/parse_scene.cpp
SceneInfo LoadScene(std::filesystem::path filename);

} // namespace bulbit
