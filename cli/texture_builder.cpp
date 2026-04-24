#include "texture_builder.h"

namespace bulbit
{

namespace
{

enum class SpectrumTextureSemantic : uint8_t
{
    reflectance,
    illuminant,
};

Image<Spectrum> ConvertToSpectrumImage(Image<Float3> image, SpectrumTextureSemantic semantic)
{
    if (!image)
    {
        return {};
    }

    Image<Spectrum> result(image.width, image.height);
    for (int32 i = 0; i < image.width * image.height; ++i)
    {
        switch (semantic)
        {
        case SpectrumTextureSemantic::reflectance:
            result[i] = Spectrum::FromRGB(image[i]);
            break;
        case SpectrumTextureSemantic::illuminant:
            result[i] = Spectrum::FromIlluminantRGB(image[i]);
            break;
        }
    }

    return result;
}

} // namespace

ConstantTexture<Float>* CreateFloatConstantTexture(Scene& scene, Float value)
{
    return scene.CreateTexture<ConstantTexture, Float>(value);
}

ConstantTexture<Float3>* CreateFloat3ConstantTexture(Scene& scene, const Float3& value)
{
    return scene.CreateTexture<ConstantTexture, Float3>(value);
}

SpectrumConstantTexture* CreateSpectrumConstantTexture(Scene& scene, const Spectrum& value)
{
    return scene.CreateSpectrumTexture<SpectrumConstantTexture>(value);
}

SpectrumConstantTexture* CreateSpectrumConstantTexture(Scene& scene, Float value)
{
    return scene.CreateSpectrumTexture<SpectrumConstantTexture>(Spectrum::Constant(value));
}

SpectrumConstantTexture* CreateSpectrumConstantTexture(Scene& scene, Float r, Float g, Float b)
{
    return scene.CreateSpectrumTexture<SpectrumConstantTexture>(Spectrum::FromRGB(Vec3(r, g, b)));
}

ImageTexture<Float>* CreateFloatImageTexture(
    Scene& scene, std::string filename, int32 channel, bool non_color, std::function<Float(Float)> transform
)
{
    return scene.CreateTexture<ImageTexture, Float>(ReadImage1(filename, channel, non_color, transform));
}

ImageTexture<Float3>* CreateFloat3ImageTexture(
    Scene& scene, std::string filename, bool non_color, std::function<Float3(Float3)> transform
)
{
    return scene.CreateTexture<ImageTexture, Float3>(ReadImage3(filename, non_color, transform));
}

SpectrumImageTexture* CreateSpectrumImageTexture(
    Scene& scene, std::string filename, bool non_color, std::function<Vec3(Vec3)> transform
)
{
    return scene.CreateSpectrumTexture<SpectrumImageTexture>(
        ConvertToSpectrumImage(ReadImage3(filename, non_color, std::move(transform)), SpectrumTextureSemantic::reflectance)
    );
}

SpectrumImageTexture* CreateSpectrumIlluminantImageTexture(
    Scene& scene, std::string filename, bool non_color, std::function<Vec3(Vec3)> transform
)
{
    return scene.CreateSpectrumTexture<SpectrumImageTexture>(
        ConvertToSpectrumImage(ReadImage3(filename, non_color, std::move(transform)), SpectrumTextureSemantic::illuminant)
    );
}

CheckerTexture<Float>* CreateFloatCheckerTexture(Scene& scene, Float a, Float b, const Point2& resolution)
{
    return scene.CreateTexture<CheckerTexture, Float>(
        CreateFloatConstantTexture(scene, a), CreateFloatConstantTexture(scene, b), resolution
    );
}

CheckerTexture<Float>* CreateFloatCheckerTexture(
    Scene& scene, const Texture<Float>* a, const Texture<Float>* b, const Point2& resolution
)
{
    return scene.CreateTexture<CheckerTexture, Float>(a, b, resolution);
}

CheckerTexture<Float3>* CreateFloat3CheckerTexture(Scene& scene, const Float3& a, const Float3& b, const Point2& resolution)
{
    return scene.CreateTexture<CheckerTexture, Float3>(
        CreateFloat3ConstantTexture(scene, a), CreateFloat3ConstantTexture(scene, b), resolution
    );
}

CheckerTexture<Float3>* CreateFloat3CheckerTexture(
    Scene& scene, const Texture<Float3>* a, const Texture<Float3>* b, const Point2& resolution
)
{
    return scene.CreateTexture<CheckerTexture, Float3>(a, b, resolution);
}

SpectrumCheckerTexture* CreateSpectrumCheckerTexture(Scene& scene, Float a, Float b, const Point2& resolution)
{
    return scene.CreateSpectrumTexture<SpectrumCheckerTexture>(
        CreateSpectrumConstantTexture(scene, a), CreateSpectrumConstantTexture(scene, b), resolution
    );
}

SpectrumCheckerTexture* CreateSpectrumCheckerTexture(Scene& scene, const Spectrum& a, const Spectrum& b, const Point2& resolution)
{
    return scene.CreateSpectrumTexture<SpectrumCheckerTexture>(
        CreateSpectrumConstantTexture(scene, a), CreateSpectrumConstantTexture(scene, b), resolution
    );
}

SpectrumCheckerTexture* CreateSpectrumCheckerTexture(
    Scene& scene, const SpectrumTexture* a, const SpectrumTexture* b, const Point2& resolution
)
{
    return scene.CreateSpectrumTexture<SpectrumCheckerTexture>(a, b, resolution);
}

} // namespace bulbit
