#pragma once

#include "filters.h"
#include "intersectable.h"
#include "ray.h"

namespace bulbit
{

class Medium;

// Camera importance equivalant to LightSampleLi
struct CameraSampleWi
{
    CameraSampleWi() = default;

    Spectrum Wi;
    Vec3 wi;
    Float pdf;
    Point2 p_raster;
    Intersection p_aperture;
};

class Camera
{
public:
    static inline std::unique_ptr<Filter> default_filter = std::make_unique<GaussianFilter>(0.5f);

    Camera(const Point2i& resolution, const Medium* medium, const Filter* pixel_filter)
        : resolution{ resolution }
        , medium{ medium }
        , filter{ pixel_filter }
    {
    }
    virtual ~Camera() = default;

    // Sample a primary ray by importance sampling a reconstruction filter
    virtual Float SampleRay(Ray* out_ray, const Point2i& pixel, const Point2& u0, const Point2& u1) const = 0;

    // Image measurement importance functions
    virtual Spectrum We(const Ray& ray, Point2* p_raster = nullptr) const;
    virtual void PDF_We(Float* pdf_p, Float* pdf_w, const Ray& ray) const;
    virtual CameraSampleWi SampleWi(const Intersection& ref, const Point2& u) const;

    const Point2i& GetScreenResolution() const;
    const Medium* GetMedium() const;
    const Filter* GetFilter() const;

protected:
    Point2i resolution;
    const Medium* medium;
    const Filter* filter;
};

inline const Point2i& Camera::GetScreenResolution() const
{
    return resolution;
}

inline const Medium* Camera::GetMedium() const
{
    return medium;
}

inline const Filter* Camera::GetFilter() const
{
    return filter;
}

inline Spectrum Camera::We(const Ray& ray, Point2* p_raster) const
{
    BulbitAssert(false && "Not implemented");
    BulbitNotUsed(ray);
    BulbitNotUsed(p_raster);
    return Spectrum::black;
}

inline void Camera::PDF_We(Float* pdf_p, Float* pdf_w, const Ray& ray) const
{
    BulbitAssert(false && "Not implemented");
    BulbitNotUsed(pdf_p);
    BulbitNotUsed(pdf_w);
    BulbitNotUsed(ray);
}

inline CameraSampleWi Camera::SampleWi(const Intersection& ref, const Point2& u) const
{
    BulbitAssert(false && "Not implemented");
    BulbitNotUsed(ref);
    BulbitNotUsed(u);
    return {};
}

} // namespace bulbit