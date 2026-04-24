#pragma once

#include "bulbit/scene.h"
#include "bulbit/textures.h"

namespace bulbit
{

// clang-format off
ConstantTexture<Float>* CreateFloatConstantTexture(
    Scene& scene,
    Float value
);
ConstantTexture<Float3>* CreateFloat3ConstantTexture(
    Scene& scene,
    const Float3& value
);
SpectrumConstantTexture* CreateSpectrumConstantTexture(
    Scene& scene,
    const Spectrum&
    value
);
SpectrumConstantTexture* CreateSpectrumConstantTexture(
    Scene& scene,
    Float value
);
SpectrumConstantTexture* CreateSpectrumConstantTexture(
    Scene& scene,
    Float r,
    Float g,
    Float b
);

ImageTexture<Float>* CreateFloatImageTexture(
    Scene& scene,
    std::string filename,
    int32 channel,
    bool non_color = false,
    std::function<Float(Float)> transform = {}
);
ImageTexture<Float3>* CreateFloat3ImageTexture(
    Scene& scene,
    std::string filename,
    bool non_color = false,
    std::function<Float3(Float3)> transform = {}
);
SpectrumImageTexture* CreateSpectrumImageTexture(
    Scene& scene,
    std::string filename,
    bool non_color = false,
    std::function<Vec3(Vec3)> transform = {}
);
SpectrumImageTexture* CreateSpectrumIlluminantImageTexture(
    Scene& scene,
    std::string filename,
    bool non_color = false,
    std::function<Vec3(Vec3)> transform = {}
);

CheckerTexture<Float>* CreateFloatCheckerTexture(
    Scene& scene,
    Float a,
    Float b,
    const Point2& resolution
);
CheckerTexture<Float>* CreateFloatCheckerTexture(
    Scene& scene,
    const Texture<Float>* a,
    const Texture<Float>* b,
    const Point2& resolution
);

CheckerTexture<Float3>* CreateFloat3CheckerTexture(
    Scene& scene,
    const Float3& a,
    const Float3& b,
    const Point2& resolution
);
CheckerTexture<Float3>* CreateFloat3CheckerTexture(
    Scene& scene,
    const Texture<Float3>* a,
    const Texture<Float3>* b,
    const Point2& resolution
);

SpectrumCheckerTexture* CreateSpectrumCheckerTexture(
    Scene& scene,
    Float a,
    Float b,
    const Point2& resolution
);
SpectrumCheckerTexture* CreateSpectrumCheckerTexture(
    Scene& scene,
    const Spectrum& a,
    const Spectrum& b,
    const Point2& resolution
);
SpectrumCheckerTexture* CreateSpectrumCheckerTexture(
    Scene& scene,
    const SpectrumTexture* a,
    const SpectrumTexture* b,
    const Point2& resolution
);
// clang-format on

} // namespace bulbit
