#pragma once

#include "bulbit/bulbit.h"

namespace bulbit
{

struct FilmInfo
{
    std::string filename = "bulbit_render.hdr";
    Point2i resolution = { 1280, 720 };
};

enum class SamplerType
{
    independent,
    stratified,
};

struct SamplerInfo
{
    SamplerType type = SamplerType::independent;
    int32 spp = 64;
};

enum class CameraType
{
    perspective,
    orthographic,
    spherical,
};

struct CameraInfo
{
    CameraType type = CameraType::perspective;

    Transform transform = identity;
    Float fov = 35.0f;
    Float aperture_radius = 0.0f;
    Float focus_distance = 1.0f;

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
    pm,
    sppm,
    naive_path,
    naive_vol_path,
    random_walk,
    ao,
    albedo,
    debug,
};

struct RendererInfo
{
    IntegratorType type = IntegratorType::path;
    int32 max_bounces = 16;
    int32 rr_min_bounces = 1;
    bool regularize_bsdf = false;

    Float ao_range = 0.1f;

    int32 n_photons = 100000;
    Float initial_radius = -1;
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
