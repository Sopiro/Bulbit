#pragma once

#include "bulbit/path.h"
#include "bulbit/ray.h"

namespace bulbit
{

class Sampler;
class Integrator;
struct Vertex;

int32 RandomWalk(
    const Integrator* integrator,
    Vertex* path,
    Ray ray,
    Spectrum beta,
    Float pdf,
    int32 bounces,
    TransportDirection direction,
    Sampler& sampler,
    Allocator& alloc
);

Spectrum ConnectPaths(
    const Integrator* integrator,
    Vertex* light_path,
    Vertex* camera_path,
    int32 s,
    int32 t,
    const Camera* camera,
    Sampler& sampler,
    Point2* p_raster
);

} // namespace bulbit
