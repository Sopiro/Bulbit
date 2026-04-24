#pragma once

#include "hash.h"
#include "image.h"
#include "pool.h"
#include "texture.h"

namespace bulbit
{

namespace detail
{

template <typename T>
inline T BilinearMix(const T& v00, const T& v10, const T& v01, const T& v11, Float fu, Float fv)
{
    // clang-format off
    return (1-fu) * (1-fv) * v00 + (1-fu) * (fv) * v01 +
           (  fu) * (1-fv) * v10 + (  fu) * (fv) * v11;
    // clang-format on
}

template <typename T>
inline T AverageImage(const Image<T>& image)
{
    T sum(0);

    for (int32 y = 0; y < image.height; ++y)
    {
        for (int32 x = 0; x < image.width; ++x)
        {
            sum += image(x, y);
        }
    }

    return sum / (image.width * image.height);
}

template <typename T>
inline T AveragePair(const T& a, const T& b)
{
    return 0.5f * (a + b);
}

} // namespace detail

template <typename T>
class ConstantTexture : public Texture<T>
{
public:
    ConstantTexture(T value)
        : value{ std::move(value) }
    {
    }

    T Evaluate(const Point2& uv) const override
    {
        BulbitNotUsed(uv);
        return value;
    }

    T Average() const override
    {
        return value;
    }

private:
    T value;
};

template <typename T>
class ImageTexture : public Texture<T>
{
public:
    ImageTexture(Image<T> image, TexCoordFilter texcoord_filter)
        : image{ std::move(image) }
        , texcoord_filter{ texcoord_filter }
    {
    }

    T Evaluate(const Point2& uv) const override
    {
#if 0
        // Nearest sampling
        Float w = uv.x * image.width + 0.5f;
        Float h = uv.y * image.height + 0.5f;

        int32 i = int32(std::floor(w));
        int32 j = int32(std::floor(h));

        FilterTexCoord(&i, &j, image.width, image.height, texcoord_filter);

        return image(i, j);
#else
        // Bilinear sampling
        Float w = uv.x * image.width - 0.5f;
        Float h = uv.y * image.height - 0.5f;

        int32 i0 = int32(std::floor(w)), i1 = i0 + 1;
        int32 j0 = int32(std::floor(h)), j1 = j0 + 1;

        FilterTexCoord(&i0, &j0, image.width, image.height, texcoord_filter);
        FilterTexCoord(&i1, &j1, image.width, image.height, texcoord_filter);

        Float fu = w - std::floor(w);
        Float fv = h - std::floor(h);

        T v00 = image[i0 + j0 * image.width], v10 = image[i1 + j0 * image.width];
        T v01 = image[i0 + j1 * image.width], v11 = image[i1 + j1 * image.width];

        return detail::BilinearMix(v00, v10, v01, v11, fu, fv);
#endif
    }

    T Average() const override
    {
        return detail::AverageImage(image);
    }

    int32 GetWidth() const
    {
        return image.width;
    }

    int32 GetHeight() const
    {
        return image.height;
    }

private:
    Image<T> image;
    TexCoordFilter texcoord_filter;
};

template <typename T>
class CheckerTexture : public Texture<T>
{
public:
    CheckerTexture(const Texture<T>* a, const Texture<T>* b, const Point2& resolution)
        : a{ a }
        , b{ b }
        , resolution{ resolution }
    {
    }

    T Evaluate(const Point2& uv) const override
    {
        Point2i scale = uv * resolution;
        int32 c = scale.x + scale.y;

        if (c % 2)
        {
            return a->Evaluate(uv);
        }
        else
        {
            return b->Evaluate(uv);
        }
    }

    T Average() const override
    {
        return detail::AveragePair(a->Average(), b->Average());
    }

private:
    const Texture<T>* a;
    const Texture<T>* b;
    Point2 resolution;
};

class SpectrumConstantTexture : public SpectrumTexture
{
public:
    SpectrumConstantTexture(Spectrum value)
        : value{ std::move(value) }
    {
    }

    SpectrumSample Evaluate(const Point2& uv, const WavelengthSample& lambda) const override
    {
        BulbitNotUsed(uv);
        return value.Sample(lambda);
    }

    const Spectrum& GetValue() const
    {
        return value;
    }

    Float MeanLuminance() const override
    {
        return value.Luminance();
    }

private:
    Spectrum value;
};

class SpectrumImageTexture : public SpectrumTexture
{
public:
    SpectrumImageTexture(Image<Spectrum> image, TexCoordFilter texcoord_filter)
        : image{ std::move(image) }
        , texcoord_filter{ texcoord_filter }
        , mean_luminance{ ComputeMeanLuminance(this->image) }
    {
    }

    SpectrumSample Evaluate(const Point2& uv, const WavelengthSample& lambda) const override
    {
        Float w = uv.x * image.width - 0.5f;
        Float h = uv.y * image.height - 0.5f;

        int32 i0 = int32(std::floor(w)), i1 = i0 + 1;
        int32 j0 = int32(std::floor(h)), j1 = j0 + 1;

        FilterTexCoord(&i0, &j0, image.width, image.height, texcoord_filter);
        FilterTexCoord(&i1, &j1, image.width, image.height, texcoord_filter);

        Float fu = w - std::floor(w);
        Float fv = h - std::floor(h);

        SpectrumSample v00 = image[i0 + j0 * image.width].Sample(lambda);
        SpectrumSample v10 = image[i1 + j0 * image.width].Sample(lambda);
        SpectrumSample v01 = image[i0 + j1 * image.width].Sample(lambda);
        SpectrumSample v11 = image[i1 + j1 * image.width].Sample(lambda);

        return detail::BilinearMix(v00, v10, v01, v11, fu, fv);
    }

    int32 GetWidth() const
    {
        return image.width;
    }

    int32 GetHeight() const
    {
        return image.height;
    }

    Float MeanLuminance() const override
    {
        return mean_luminance;
    }

    const Image<Spectrum>& GetImage() const
    {
        return image;
    }

    const Spectrum& GetTexel(int32 x, int32 y) const
    {
        return image(x, y);
    }

private:
    static Float ComputeMeanLuminance(const Image<Spectrum>& image)
    {
        if (!image)
        {
            return 0.0f;
        }

        Float sum = 0.0f;
        for (int32 y = 0; y < image.height; ++y)
        {
            for (int32 x = 0; x < image.width; ++x)
            {
                sum += image(x, y).Luminance();
            }
        }

        return sum / Float(image.width * image.height);
    }

    Image<Spectrum> image;
    TexCoordFilter texcoord_filter;
    Float mean_luminance;
};

class SpectrumCheckerTexture : public SpectrumTexture
{
public:
    SpectrumCheckerTexture(const SpectrumTexture* a, const SpectrumTexture* b, const Point2& resolution)
        : a{ a }
        , b{ b }
        , resolution{ resolution }
        , mean_luminance{ 0.5f * (a->MeanLuminance() + b->MeanLuminance()) }
    {
    }

    SpectrumSample Evaluate(const Point2& uv, const WavelengthSample& lambda) const override
    {
        Point2i scale = uv * resolution;
        int32 c = scale.x + scale.y;

        if (c % 2)
        {
            return a->Evaluate(uv, lambda);
        }
        else
        {
            return b->Evaluate(uv, lambda);
        }
    }

    Float MeanLuminance() const override
    {
        return mean_luminance;
    }

private:
    const SpectrumTexture* a;
    const SpectrumTexture* b;
    Point2 resolution;
    Float mean_luminance;
};

using FloatConstantTexture = ConstantTexture<Float>;
using Float3ConstantTexture = ConstantTexture<Float3>;

using FloatImageTexture = ImageTexture<Float>;
using Float3ImageTexture = ImageTexture<Float3>;

using FloatCheckerTexture = CheckerTexture<Float>;
using Float3CheckerTexture = CheckerTexture<Float3>;

template <typename T>
concept BuiltInSpectrumTexture =
    std::same_as<T, SpectrumConstantTexture> || std::same_as<T, SpectrumImageTexture> || std::same_as<T, SpectrumCheckerTexture>;

namespace detail
{

template <typename T>
using key_0d = T;

template <typename T>
using key_2d = std::pair<const T*, int32>;

template <typename T>
using key_c = std::tuple<const Texture<T>*, const Texture<T>*, Point2>;

using spectrum_key_c = std::tuple<const SpectrumTexture*, const SpectrumTexture*, Point2>;

template <typename T>
struct hasher_0d
{
    size_t operator()(key_0d<T> key) const
    {
        return Hash(key);
    }
};

template <>
struct hasher_0d<Spectrum>
{
    size_t operator()(const Spectrum& key) const
    {
        return HashSpectrum(key);
    }
};

template <typename T>
struct hasher_2d
{
    size_t operator()(key_2d<T> image) const
    {
        const T* buffer = image.first;
        int32 size = image.second;

        return HashBuffer(buffer, size);
    }
};

template <>
struct hasher_2d<Spectrum>
{
    size_t operator()(key_2d<Spectrum> image) const
    {
        const Spectrum* buffer = image.first;
        int32 size = image.second;

        uint64_t h = 0;
        for (int32 i = 0; i < size; ++i)
        {
            h = Hash(h, HashSpectrum(buffer[i]));
        }
        return h;
    }
};

template <typename T>
struct hasher_c
{
    size_t operator()(key_c<T> key) const
    {
        const Texture<T>* a = std::get<0>(key);
        const Texture<T>* b = std::get<1>(key);
        Point2 resolution = std::get<2>(key);

        return Hash(a, b, resolution);
    }
};

struct spectrum_hasher_c
{
    size_t operator()(spectrum_key_c key) const
    {
        const SpectrumTexture* a = std::get<0>(key);
        const SpectrumTexture* b = std::get<1>(key);
        Point2 resolution = std::get<2>(key);

        return Hash(a, b, resolution);
    }
};

template <typename T>
using Pool0d = Pool<detail::key_0d<T>, ConstantTexture<T>, detail::hasher_0d<T>>;

template <typename T>
using Pool2d = Pool<detail::key_2d<T>, ImageTexture<T>, detail::hasher_2d<T>>;

template <typename T>
using PoolC = Pool<detail::key_c<T>, CheckerTexture<T>, detail::hasher_c<T>>;

using SpectrumPool0d = Pool<detail::key_0d<Spectrum>, SpectrumConstantTexture, detail::hasher_0d<Spectrum>>;
using SpectrumPool2d = Pool<detail::key_2d<Spectrum>, SpectrumImageTexture, detail::hasher_2d<Spectrum>>;
using SpectrumPoolC = Pool<detail::spectrum_key_c, SpectrumCheckerTexture, detail::spectrum_hasher_c>;

} // namespace detail

class TexturePool
{
public:
    template <template <typename> class TextureType, typename T, typename... Args>
    TextureType<T>* CreateTexture(Args&&... args)
    {
        static_assert(!std::is_same_v<T, Spectrum>, "Use Spectrum-specific TexturePool creation methods");

        if constexpr (std::is_same_v<TextureType<T>, ConstantTexture<T>>)
        {
            if constexpr (sizeof...(Args) == 1)
            {
                T value = std::get<0>(std::forward_as_tuple(args...));
                return GetPool<TextureType, T>().Create(value, value);
            }
            else
            {
                static_assert(sizeof(T) == 0, "Invalid arguments for ConstantTexture");
            }
        }
        else if constexpr (std::is_same_v<TextureType<T>, ImageTexture<T>>)
        {
            if constexpr (sizeof...(Args) == 1)
            {
                auto&& image = std::get<0>(std::forward_as_tuple(args...));
                if (image)
                {
                    return GetPool<TextureType, T>().Create(
                        { &image[0], image.width * image.height }, std::move(image), TexCoordFilter::repeat
                    );
                }
                else
                {
                    return nullptr;
                }
            }
            else
            {
                static_assert(sizeof(T) == 0, "Invalid arguments for ImageTexture");
            }
        }
        else if constexpr (std::is_same_v<TextureType<T>, CheckerTexture<T>>)
        {
            if constexpr (sizeof...(Args) == 3)
            {
                const Texture<T>* a = std::get<0>(std::forward_as_tuple(args...));
                const Texture<T>* b = std::get<1>(std::forward_as_tuple(args...));
                const Point2& resolution = std::get<2>(std::forward_as_tuple(args...));

                return GetPool<CheckerTexture, T>().Create({ a, b, resolution }, a, b, resolution);
            }
            else
            {
                static_assert(sizeof(T) == 0, "Invalid arguments for CheckerTexture");
            }
        }
        else
        {
            static_assert(sizeof(T) == 0, "Unsupported texture type");
        }
    }

    template <BuiltInSpectrumTexture TextureType, typename... Args>
    TextureType* CreateSpectrumTexture(Args&&... args)
    {
        if constexpr (std::is_same_v<TextureType, SpectrumConstantTexture>)
        {
            if constexpr (sizeof...(Args) == 1)
            {
                Spectrum value = std::get<0>(std::forward_as_tuple(args...));
                return spectrum_pool_0d.Create(value, value);
            }
            else
            {
                static_assert(sizeof(TextureType) == 0, "Invalid arguments for SpectrumConstantTexture");
            }
        }
        else if constexpr (std::is_same_v<TextureType, SpectrumImageTexture>)
        {
            if constexpr (sizeof...(Args) == 1)
            {
                auto&& image = std::get<0>(std::forward_as_tuple(args...));
                if (!image)
                {
                    return nullptr;
                }

                return spectrum_pool_2d.Create(
                    { &image[0], image.width * image.height }, std::move(image), TexCoordFilter::repeat
                );
            }
            else
            {
                static_assert(sizeof(TextureType) == 0, "Invalid arguments for SpectrumImageTexture");
            }
        }
        else if constexpr (std::is_same_v<TextureType, SpectrumCheckerTexture>)
        {
            if constexpr (sizeof...(Args) == 3)
            {
                const SpectrumTexture* a = std::get<0>(std::forward_as_tuple(args...));
                const SpectrumTexture* b = std::get<1>(std::forward_as_tuple(args...));
                const Point2& resolution = std::get<2>(std::forward_as_tuple(args...));

                return spectrum_pool_c.Create({ a, b, resolution }, a, b, resolution);
            }
            else
            {
                static_assert(sizeof(TextureType) == 0, "Invalid arguments for SpectrumCheckerTexture");
            }
        }
        else
        {
            static_assert(sizeof(TextureType) == 0, "Unsupported spectrum texture type");
        }
    }

    void Clear()
    {
        pool_0d1f.Clear();
        pool_0d3v.Clear();
        pool_2d1f.Clear();
        pool_2d3v.Clear();
        pool_C1f.Clear();
        pool_C3v.Clear();

        spectrum_pool_0d.Clear();
        spectrum_pool_2d.Clear();
        spectrum_pool_c.Clear();
    }

private:
    template <template <typename> class TextureType, typename T>
    auto& GetPool()
    {
        if constexpr (std::is_same_v<TextureType<T>, ConstantTexture<T>>)
        {
            if constexpr (std::is_same_v<T, Float>)
            {
                return pool_0d1f;
            }
            else if constexpr (std::is_same_v<T, Float3>)
            {
                return pool_0d3v;
            }
            else
            {
                static_assert(sizeof(T) == 0, "Unsupported type for ConstantTexture");
            }
        }
        else if constexpr (std::is_same_v<TextureType<T>, ImageTexture<T>>)
        {
            if constexpr (std::is_same_v<T, Float>)
            {
                return pool_2d1f;
            }
            else if constexpr (std::is_same_v<T, Float3>)
            {
                return pool_2d3v;
            }
            else
            {
                static_assert(sizeof(T) == 0, "Unsupported type for ImageTexture");
            }
        }
        else if constexpr (std::is_same_v<TextureType<T>, CheckerTexture<T>>)
        {
            if constexpr (std::is_same_v<T, Float>)
            {
                return pool_C1f;
            }
            else if constexpr (std::is_same_v<T, Float3>)
            {
                return pool_C3v;
            }
            else
            {
                static_assert(sizeof(T) == 0, "Unsupported type for CheckerTexture");
            }
        }
        else
        {
            static_assert(sizeof(T) == 0, "Unsupported texture type");
        }
    }

    detail::Pool0d<Float> pool_0d1f;
    detail::Pool0d<Float3> pool_0d3v;

    detail::Pool2d<Float> pool_2d1f;
    detail::Pool2d<Float3> pool_2d3v;

    detail::PoolC<Float> pool_C1f;
    detail::PoolC<Float3> pool_C3v;

    detail::SpectrumPool0d spectrum_pool_0d;
    detail::SpectrumPool2d spectrum_pool_2d;
    detail::SpectrumPoolC spectrum_pool_c;
};

} // namespace bulbit
