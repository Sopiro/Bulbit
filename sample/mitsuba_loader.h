#pragma once

#include "bulbit/bulbit.h"
#include <optional>

namespace bulbit
{

struct FilmInfo
{
    int32 width;
    int32 height;
    std::string filename;
};

enum SamplerType
{
    independent,
    stratified,
};

struct SamplerInfo
{
    SamplerType type;
    int32 spp;
};

enum CameraType
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

enum IntegratorType
{
    path,
    vol_path,
    light_path,
    light_vol_path,
    bdpt,
    vol_bdpt,
    ao,
    debug,
    pm,
    sppm,
};

struct RendererInfo
{
    IntegratorType type;
    int32 max_bounces;
    int32 rr_depth;
};

struct SceneInfo
{
    std::unique_ptr<Scene> scene;

    CameraInfo camera_info;
    RendererInfo renderer_info;
};

std::optional<SceneInfo> LoadMitsubaScene(std::filesystem::path filename);

} // namespace bulbit
