#pragma once

#include "scene.h"

namespace bulbit
{

enum class FilterType
{
    box,
    tent,
    gaussian,
};

struct FilterInfo
{
    FilterType type = FilterType::gaussian;
    Float extent = 3;
    Float gaussian_stddev = 0.5f;
};

struct FilmInfo
{
    std::string filename = "";
    Point2i resolution = { 1280, 720 };

    FilterInfo filter_info;
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
    Point2 viewport_size = { 1, 1 }; // Used for orthographic camera

    FilmInfo film_info;
    SamplerInfo sampler_info;

    const Medium* medium = nullptr;
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
    count,
};

struct IntegratorInfo
{
    IntegratorType type = IntegratorType::path;

    int32 max_bounces = 16;
    int32 rr_min_bounces = 1;
    bool regularize_bsdf = false;

    Float ao_range = 0.1f;

    int32 n_photons = 100000;
    Float initial_radius = -1;
};

struct RendererInfo
{
    Scene scene;

    CameraInfo camera_info;
    IntegratorInfo integrator_info;
};

} // namespace bulbit
