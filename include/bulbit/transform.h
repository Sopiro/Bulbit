#pragma once

#include "quaternion.h"
#include "vectors.h"

namespace bulbit
{

struct Transform
{
    Vec3 p; // position
    Quat q; // orientation
    Vec3 s; // scale

    constexpr Transform() = default;

    constexpr Transform(Identity)
        : p{ 0 }
        , q{ identity }
        , s{ 1 }
    {
    }

    constexpr Transform(const Vec3& position)
        : p{ position }
        , q{ identity }
        , s{ 1 }
    {
    }

    constexpr Transform(const Quat& orientation)
        : p{ 0 }
        , q{ orientation }
        , s{ 1 }
    {
    }

    constexpr Transform(const Vec3& position, const Quat& orientation)
        : p{ position }
        , q{ orientation }
        , s{ 1 }
    {
    }

    constexpr Transform(const Vec3& position, const Quat& orientation, const Vec3& scale)
        : p{ position }
        , q{ orientation }
        , s{ scale }
    {
    }

    constexpr Transform(Float x, Float y, Float z, const Quat& orientation = Quat(1), const Vec3& scale = Vec3(1))
        : p{ x, y, z }
        , q{ orientation }
        , s{ scale }
    {
    }

    Transform(const Mat4& m)
    {
        s.x = std::sqrt(m[0][0] * m[0][0] + m[0][1] * m[0][1] + m[0][2] * m[0][2]);
        s.y = std::sqrt(m[1][0] * m[1][0] + m[1][1] * m[1][1] + m[1][2] * m[1][2]);
        s.z = std::sqrt(m[2][0] * m[2][0] + m[2][1] * m[2][1] + m[2][2] * m[2][2]);

        q = Mat3(
            Vec3(m[0][0], m[0][1], m[0][2]) / s.x, Vec3(m[1][0], m[1][1], m[1][2]) / s.y, Vec3(m[2][0], m[2][1], m[2][2]) / s.z
        );

        p.x = m[3][0];
        p.y = m[3][1];
        p.z = m[3][2];
    }

    constexpr void Set(const Vec3& position, const Quat& orientation, const Vec3& scale)
    {
        p = position;
        q = orientation;
        s = scale;
    }

    constexpr void SetIdentity()
    {
        p.SetZero();
        q.SetIdentity();
        s.Set(1, 1, 1);
    }

    std::string ToString() const
    {
        return std::format("p:{}\nq:{}\ns:{}", p.ToString(), q.ToString(), s.ToString());
    }

    constexpr Transform& operator*=(const Transform& other);

    constexpr Transform GetInverse() const
    {
        Quat inv_q = q.GetConjugate();
        Vec3 inv_s = Vec3(1) / s;
        return Transform{ inv_q.Rotate(-(p * inv_s)), inv_q, inv_s };
    }

    static Transform Translate(const Vec3& position)
    {
        return Transform(position, Quat(1), Vec3(1));
    }

    static Transform Rotate(const Vec3& rotation)
    {
        return Transform(Vec3(0), Quat::FromEuler(rotation), Vec3(1));
    }

    static Transform Scale(const Vec3& scale)
    {
        return Transform(Vec3(0), Quat(1), scale);
    }

    static Transform LookAt(const Vec3& position, const Vec3& target, const Vec3& up)
    {
        Vec3 w = target - position;
        w.Normalize();
        return Transform(position, Quat(w, up));
    }
};

constexpr inline bool operator==(const Transform& a, const Transform& b)
{
    return a.p == b.p && a.q == b.q && a.s == b.s;
}

constexpr inline Vec3 operator*(const Transform& t, const Vec3& v)
{
    return t.q.Rotate(t.s * v) + t.p;
}

// A * V
constexpr inline Vec3 Mul(const Transform& t, const Vec3& v)
{
    return t.q.Rotate(t.s * v) + t.p;
}

// A^{-1} * V
constexpr inline Vec3 MulT(const Transform& t, const Vec3& v)
{
    return t.q.RotateInv(v - t.p) / t.s;
}

constexpr inline Transform operator*(const Transform& a, const Transform& b)
{
    return Transform{ a.q.Rotate(a.s * b.p) + a.p, a.q * b.q, a.s * b.s };
}

// A * B
constexpr inline Transform Mul(const Transform& a, const Transform& b)
{
    return Transform{ a.q.Rotate(a.s * b.p) + a.p, a.q * b.q, a.s * b.s };
}

// A^{-1} * B
constexpr inline Transform MulT(const Transform& a, const Transform& b)
{
    Quat inv_q = a.q.GetConjugate();
    Vec3 inv_s = Vec3(1) / a.s;
    return Transform{ inv_q.Rotate((b.p - a.p) * inv_s), inv_q * b.q, inv_s * b.s };
}

constexpr inline Transform& Transform::operator*=(const Transform& other)
{
    *this = Mul(*this, other);
    return *this;
}

} // namespace bulbit