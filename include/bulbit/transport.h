#pragma once

#include "bulbit/path.h"
#include "bulbit/ray.h"

namespace bulbit
{

struct Vertex;
class Medium;
class Sampler;
class Integrator;

int32 RandomWalk(
    const Integrator* integrator,
    Vertex* path,
    Ray ray,
    WavelengthSample& lambda,
    SpectrumSample beta,
    Float pdf,
    int32 max_bounces,
    int32 rr_min_bounces,
    TransportDirection direction,
    Sampler& sampler,
    Allocator& alloc
);

int32 RandomWalkVol(
    const Integrator* integrator,
    Vertex* path,
    Ray ray,
    const Medium* medium,
    WavelengthSample& lambda,
    SpectrumSample beta,
    Float pdf,
    int32 max_bounces,
    int32 rr_min_bounces,
    TransportDirection direction,
    Sampler& sampler,
    Allocator& alloc
);

SpectrumSample ConnectPaths(
    const Integrator* integrator,
    Vertex* light_path,
    Vertex* camera_path,
    int32 s,
    int32 t,
    const Camera* camera,
    const WavelengthSample& lambda,
    Sampler& sampler,
    Point2* p_raster
);

SpectrumSample ConnectPathsVol(
    const Integrator* integrator,
    Vertex* light_path,
    Vertex* camera_path,
    int32 s,
    int32 t,
    const Camera* camera,
    const WavelengthSample& lambda,
    Sampler& sampler,
    Point2* p_raster
);

} // namespace bulbit
