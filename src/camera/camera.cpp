#include "bulbit/cameras.h"
#include "bulbit/renderer_info.h"

namespace bulbit
{

Camera* Camera::Create(Allocator& alloc, const CameraInfo& ci, const Filter* filter)
{
    const Medium* camera_medium = nullptr; // TODO: handle it correctly
    switch (ci.type)
    {
    case CameraType::perspective:
        return alloc.new_object<PerspectiveCamera>(
            ci.transform, ci.fov, ci.aperture_radius, ci.focus_distance, ci.film_info.resolution, camera_medium, filter
        );
    case CameraType::orthographic:
        return alloc.new_object<OrthographicCamera>(
            ci.transform, ci.viewport_size, ci.film_info.resolution.x, camera_medium, filter
        );
    case CameraType::spherical:
        return alloc.new_object<SphericalCamera>(ci.transform, ci.film_info.resolution, camera_medium, filter);

    default:
        return nullptr;
    }
}

} // namespace bulbit
