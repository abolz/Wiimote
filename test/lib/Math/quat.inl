// This file is distributed under the MIT license.
// See the LICENSE file for details.

#include "lib/Math/mat.h"

namespace math
{

//--------------------------------------------------------------------------------------------------
// Implementation
//

inline quat::quat(float w, float x, float y, float z)
    : w(w)
    , x(x)
    , y(y)
    , z(z)
{
}

inline quat::quat(float w, vec3 const& v)
    : w(w)
    , x(v.x)
    , y(v.y)
    , z(v.z)
{
}

inline quat::quat(vec3 const& v)
    : w(0)
    , x(v.x)
    , y(v.y)
    , z(v.z)
{
}

inline quat quat::identity()
{
    return { 1, 0, 0, 0 };
}

inline quat quat::rotation(mat4 const& m)
{
    auto w = 0.5f * sqrt(max(0.0f, 1.0f + m(0, 0) + m(1, 1) + m(2, 2)));
    auto x = 0.5f * sqrt(max(0.0f, 1.0f + m(0, 0) - m(1, 1) - m(2, 2)));
    auto y = 0.5f * sqrt(max(0.0f, 1.0f - m(0, 0) + m(1, 1) - m(2, 2)));
    auto z = 0.5f * sqrt(max(0.0f, 1.0f - m(0, 0) - m(1, 1) + m(2, 2)));

    x = copysign(x, m(2, 1) - m(1, 2));
    y = copysign(y, m(0, 2) - m(2, 0));
    z = copysign(z, m(1, 0) - m(0, 1));

    return { w, x, y, z };
}

inline quat quat::rotation(vec3 const& axis, float angle)
{
    auto s = sin(0.5f * angle) / length(axis);
    auto c = cos(0.5f * angle);

    return { c, s * axis.x, s * axis.y, s * axis.z };
}

inline quat quat::rotation(vec3 const& from, vec3 const& to)
{
    auto f = sqrt(2.0f + 2.0f * dot(from, to));

    auto v = cross(from, to);

    // NOTE:
    // Result is normalized if <from> and <to> are normalized.
    return { 0.5f * f, v.x / f, v.y / f, v.z / f };
}

//--------------------------------------------------------------------------------------------------
// quat operators
//

inline quat operator-(quat const& p)
{
    return { -p.w, -p.x, -p.y, -p.z };
}

inline quat operator+(quat const& p, quat const& q)
{
    return { p.w + q.w, p.x + q.x, p.y + q.y, p.z + q.z };
}

inline quat operator-(quat const& p, quat const& q)
{
    return { p.w - q.w, p.x - q.x, p.y - q.y, p.z - q.z };
}

inline quat operator*(quat const& p, quat const& q)
{
    return { p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z,
             p.w * q.x + q.w * p.x + p.y * q.z - p.z * q.y,
             p.w * q.y + q.w * p.y + p.z * q.x - p.x * q.z,
             p.w * q.z + q.w * p.z + p.x * q.y - p.y * q.x };
}

inline quat operator*(quat const& p, float s)
{
    return { p.w * s, p.x * s, p.y * s, p.z * s };
}

inline quat operator/(quat const& p, float s)
{
    return { p.w / s, p.x / s, p.y / s, p.z / s };
}

inline quat operator*(float s, quat const& p)
{
    return p * s;
}

inline quat& operator+=(quat& u, quat const& s)
{
    return u = u + s;
}

inline quat& operator-=(quat& u, quat const& s)
{
    return u = u - s;
}

inline quat& operator*=(quat& u, quat const& s)
{
    return u = u * s;
}

inline quat& operator*=(quat& u, float s)
{
    return u = u * s;
}

inline quat& operator/=(quat& u, float s)
{
    return u = u / s;
}

//--------------------------------------------------------------------------------------------------
// Geometric functions
//

// Computes the dot-product of the two quats
inline float dot(quat const& p, quat const& q)
{
    return p.w * q.w + p.x * q.x + p.y * q.y + p.z * q.z;
}

// Returns the norm of the quat
inline float length_squared(quat const& q)
{
    return dot(q, q);
}

// Returns the length of the quat
inline float length(quat const& q)
{
    return sqrt(length_squared(q));
}

// Returns a normalized version of the given quat
inline quat normalize(quat const& q)
{
    return q / length(q);
}

// Returns the conjugate of the quat
inline quat conjugate(quat const& q) // reverse
{
    return { q.w, -q.x, -q.y, -q.z };
}

// Returns the inverse of the quat
inline quat inverse(quat const& q)
{
    return conjugate(q) / length_squared(q);
}

// Computes the rotation angle
inline float rotation_angle(quat const& q)
{
    return 2.0f * acos(q.w);
}

// Computes the rotation axis
inline vec3 rotation_axis(quat const& q)
{
    return normalize(vec3(q.x, q.y, q.z));
}

// Performs a spherical linear interpolation
// A and B must be normalized!
inline quat slerp(quat const& a, quat const& b, float t)
{
    auto cosphi = dot(a, b);
    auto phi = acos(clamp(cosphi, -1.0f, 1.0f));

    return cos(t * phi) * a + sin(t * phi) * normalize(a - cosphi * b);
}

// quaternion integration step
// ^^  ^^         ^       ^
inline quat quergs(quat const& dq /* = dt/2 q w */)
{
    auto s = length(dq);

    return (tan(s) / s) * dq;
}

} // namespace math
