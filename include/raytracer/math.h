// Simple linear math library
// Highly inspired by Box2d, glm math codes

#pragma once

#include <float.h>
#include <limits>
#include <math.h>
#include <stdint.h>

#include "types.h"

#define PI 3.14159265359f

#define precision double

struct Vec2
{
    precision x, y;

    Vec2() = default;

    constexpr Vec2(precision _v)
        : x{ _v }
        , y{ _v }
    {
    }

    constexpr Vec2(precision _x, precision _y)
        : x{ _x }
        , y{ _y }
    {
    }

    void SetZero()
    {
        x = (precision)0.0;
        y = (precision)0.0;
    }

    void Set(precision _x, precision _y)
    {
        x = _x;
        y = _y;
    }

    precision& operator[](uint32 i)
    {
        return (&x)[i];
    }

    Vec2 operator-() const
    {
        return Vec2{ -x, -y };
    }

    void operator+=(const Vec2& v)
    {
        x += v.x;
        y += v.y;
    }

    void operator-=(const Vec2& v)
    {
        x -= v.x;
        y -= v.y;
    }

    void operator+=(precision s)
    {
        x += s;
        y += s;
    }

    void operator-=(precision s)
    {
        x -= s;
        y -= s;
    }

    void operator*=(precision s)
    {
        x *= s;
        y *= s;
    }

    void operator/=(precision s)
    {
        operator*=((precision)1.0 / s);
    }

    precision Length() const
    {
        return (precision)sqrt(x * x + y * y);
    }

    precision Length2() const
    {
        return x * x + y * y;
    }

    // Optimized to not check length == 0
    precision Normalize()
    {
        precision length = Length();
        precision invLength = (precision)1.0 / length;
        x *= invLength;
        y *= invLength;

        return length;
    }

    // Optimized to not check length == 0
    Vec2 Normalized() const
    {
        precision invLength = (precision)1.0 / Length();

        return Vec2{ x * invLength, y * invLength };
    }

    // Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
    // return cross(1, *this);
    Vec2 Skew() const
    {
        return Vec2{ -y, x };
    }
};

struct Vec3
{
    precision x, y, z;

    Vec3() = default;

    constexpr Vec3(precision _v)
        : x{ _v }
        , y{ _v }
        , z{ _v }
    {
    }

    constexpr Vec3(precision _x, precision _y, precision _z)
        : x{ _x }
        , y{ _y }
        , z{ _z }
    {
    }

    Vec3(const Vec2& _v)
        : x{ _v.x }
        , y{ _v.y }
        , z{ 0.0f }
    {
    }

    void SetZero()
    {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
    }

    void Set(precision _x, precision _y, precision _z)
    {
        x = _x;
        y = _y;
        z = _z;
    }

    precision& operator[](uint32 i)
    {
        return (&x)[i];
    }

    Vec3 operator-() const
    {
        return Vec3{ -x, -y, -z };
    }

    void operator+=(const Vec3& v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
    }

    void operator-=(const Vec3& v)
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
    }

    void operator+=(precision s)
    {
        x += s;
        y += s;
        z += s;
    }

    void operator-=(precision s)
    {
        x -= s;
        y -= s;
        z -= s;
    }

    void operator*=(precision s)
    {
        x *= s;
        y *= s;
        z *= s;
    }

    void operator/=(precision s)
    {
        operator*=((precision)1.0 / s);
    }

    precision Length() const
    {
        return (precision)sqrt(x * x + y * y + z * z);
    }

    precision Length2() const
    {
        return x * x + y * y + z * z;
    }

    void Normalize()
    {
        precision length = Length();
        if (length < FLT_EPSILON)
        {
            return;
        }

        precision invLength = 1.0f / length;
        x *= invLength;
        y *= invLength;
        z *= invLength;
    }

    Vec3 Normalized() const
    {
        precision length = Length();
        if (length < FLT_EPSILON)
        {
            return Vec3{ (precision)0.0, (precision)0.0, (precision)0.0 };
        }

        precision invLength = (precision)1.0 / length;

        return Vec3{ x * invLength, y * invLength, z * invLength };
    }
};

struct Vec4
{
    precision x, y, z, w;

    Vec4() = default;

    constexpr Vec4(precision _v, precision _w)
        : x{ _v }
        , y{ _v }
        , z{ _v }
        , w{ _w }
    {
    }

    constexpr Vec4(precision _x, precision _y, precision _z, precision _w)
        : x{ _x }
        , y{ _y }
        , z{ _z }
        , w{ _w }
    {
    }

    precision& operator[](uint32 i)
    {
        return (&x)[i];
    }
};

inline precision Dot(const Vec2& a, const Vec2& b)
{
    return a.x * b.x + a.y * b.y;
}

inline precision Cross(const Vec2& a, const Vec2& b)
{
    return a.x * b.y - a.y * b.x;
}

inline Vec2 Cross(precision s, const Vec2& v)
{
    return Vec2{ -s * v.y, s * v.x };
}

inline Vec2 Cross(const Vec2& v, precision s)
{
    return Vec2{ s * v.y, -s * v.x };
}

inline Vec2 operator+(const Vec2& a, const Vec2& b)
{
    return Vec2{ a.x + b.x, a.y + b.y };
}

inline Vec2 operator-(const Vec2& a, const Vec2& b)
{
    return Vec2{ a.x - b.x, a.y - b.y };
}

inline Vec2 operator*(const Vec2& v, precision s)
{
    return Vec2{ v.x * s, v.y * s };
}

inline Vec2 operator/(const Vec2& v, precision s)
{
    return v * (1.0f / s);
}

inline bool operator==(const Vec2& a, const Vec2& b)
{
    return a.x == b.x && a.y == b.y;
}

inline bool operator!=(const Vec2& a, const Vec2& b)
{
    return a.x != b.x || a.y != b.y;
}

inline precision Dist(const Vec2& a, const Vec2& b)
{
    return (a - b).Length();
}

inline precision Dist2(const Vec2& a, const Vec2& b)
{
    return (a - b).Length2();
}

inline precision Length(const Vec2& v)
{
    return v.Length();
}

inline precision Length2(const Vec2& v)
{
    return v.Length2();
}

inline Vec2 Normalize(const Vec2& v)
{
    return v.Normalized();
}

// Vec2 functions end

// Vec3 functions begin

inline precision Dot(const Vec3& a, const Vec3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 Cross(const Vec3& a, const Vec3& b)
{
    return Vec3{ a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}

inline Vec3 operator+(const Vec3& a, const Vec3& b)
{
    return Vec3{ a.x + b.x, a.y + b.y, a.z + b.z };
}

inline Vec3 operator-(const Vec3& a, const Vec3& b)
{
    return Vec3{ a.x - b.x, a.y - b.y, a.z - b.z };
}

inline Vec3 operator*(const Vec3& v, precision s)
{
    return Vec3{ v.x * s, v.y * s, v.z * s };
}

inline Vec3 operator/(const Vec3& v, precision s)
{
    return v * (1.0f / s);
}

inline bool operator==(const Vec3& a, const Vec3& b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

inline bool operator!=(const Vec3& a, const Vec3& b)
{
    return a.x != b.x || a.y != b.y || a.z != b.z;
}

inline precision Dist(const Vec3& a, const Vec3& b)
{
    return (b - a).Length();
}

inline precision Dist2(const Vec3& a, const Vec3& b)
{
    return (b - a).Length2();
}

// Vec3 functions end

// Vec4 functions begin

inline precision Dot(const Vec4& a, const Vec4& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

inline Vec4 operator+(const Vec4& a, const Vec4& b)
{
    return Vec4{ a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w };
}

inline Vec4 operator-(const Vec4& a, const Vec4& b)
{
    return Vec4{ a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w };
}

inline bool operator==(const Vec4& a, const Vec4& b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}

inline bool operator!=(const Vec4& a, const Vec4& b)
{
    return a.x != b.x || a.y != b.y || a.z != b.z || a.w != b.w;
}

template <typename T>
inline T Lerp(const T& a, const T& b, precision t)
{
    return a * (1.0f - t) + b * t;
}

template <typename T>
inline T Slerp(const T& start, const T& end, precision percent)
{
    precision dot = Clamp(Dot(start, end), -1.0f, 1.0f);
    precision angle = acosf(dot) * percent;

    T rv = end - start * dot;
    rv.Normalize();

    return start * Cos(angle) + rv * Sin(angle);
}