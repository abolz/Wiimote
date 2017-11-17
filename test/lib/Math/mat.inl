// This file is distributed under the MIT license.
// See the LICENSE file for details.

#include "lib/Math/quat.h"
#if !LIB_MATH_NO_ALIGNED_VECTOR_TYPES
#include <xmmintrin.h>
#endif

namespace math
{

//==================================================================================================
// mat4
//==================================================================================================

//--------------------------------------------------------------------------------------------------
// Implementation
//

inline mat4::mat4(float m00, float m10, float m20, float m30,
                  float m01, float m11, float m21, float m31,
                  float m02, float m12, float m22, float m32,
                  float m03, float m13, float m23, float m33)
    : col0(m00, m10, m20, m30)
    , col1(m01, m11, m21, m31)
    , col2(m02, m12, m22, m32)
    , col3(m03, m13, m23, m33)
{
}

inline mat4::mat4(float m00, float m11, float m22, float m33)
    : col0(m00, 0, 0, 0)
    , col1(0, m11, 0, 0)
    , col2(0, 0, m22, 0)
    , col3(0, 0, 0, m33)
{
}

inline mat4::mat4(float const p[16])
    : col0(&p[0])
    , col1(&p[4])
    , col2(&p[8])
    , col3(&p[12])
{
}

inline mat4::mat4(vec4 const& c0, vec4 const& c1, vec4 const& c2, vec4 const& c3)
    : col0(c0)
    , col1(c1)
    , col2(c2)
    , col3(c3)
{
}

inline mat4::mat4(vec3 const& c0, vec3 const& c1, vec3 const& c2)
    : col0(c0, 0)
    , col1(c1, 0)
    , col2(c2, 0)
    , col3(0, 0, 0, 1)
{
}

inline mat4::mat4(mat3 const& m)
    : col0(m.col0, 0)
    , col1(m.col1, 0)
    , col2(m.col2, 0)
    , col3(0, 0, 0, 1)
{
}

inline float* mat4::data()
{
    return col0.data();
}

inline float const* mat4::data() const
{
    return col0.data();
}

inline vec4& mat4::operator()(size_t col)
{
    return (&col0)[col];
}

inline vec4 const& mat4::operator()(size_t col) const
{
    return (&col0)[col];
}

inline float& mat4::operator()(size_t row, size_t col)
{
    return (operator()(col))[row];
}

inline float const& mat4::operator()(size_t row, size_t col) const
{
    return (operator()(col))[row];
}

inline float& mat4::operator[](size_t index)
{
    return data()[index];
}

inline float const& mat4::operator[](size_t index) const
{
    return data()[index];
}

inline mat4 mat4::identity()
{
    return { 1.0f, 1.0f, 1.0f, 1.0f };
}

inline mat4 mat4::zero()
{
    return { 0.0f, 0.0f, 0.0f, 0.0f };
}

inline mat4 mat4::translation(vec3 const& t)
{
    return { 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             t.x, t.y, t.z, 1 };
}

inline mat4 mat4::scaling(vec3 const& s)
{
    return { s.x, s.y, s.z, 1 };
}

inline mat4 mat4::rotation(vec3 const& axis, float angle)
{
    auto v = normalize(axis);
    auto s = sin(angle);
    auto c = cos(angle);

    return { v.x * v.x * (1 - c) + c,
             v.x * v.y * (1 - c) + s * v.z,
             v.x * v.z * (1 - c) - s * v.y,
             0,
             v.y * v.x * (1 - c) - s * v.z,
             v.y * v.y * (1 - c) + c,
             v.y * v.z * (1 - c) + s * v.x,
             0,
             v.z * v.x * (1 - c) + s * v.y,
             v.z * v.y * (1 - c) - s * v.x,
             v.z * v.z * (1 - c) + c,
             0,
             0,
             0,
             0,
             1 };
}

inline mat4 mat4::rotation(vec3 const& from, vec3 const& to)
{
    auto v = cross(from, to);
    auto e = dot(from, to);
    auto h = 1 / (1 + e);

    return { h * v.x * v.x + e,
             h * v.x * v.y + v.z,
             h * v.x * v.z - v.y,
             0,
             h * v.x * v.y - v.z,
             h * v.y * v.y + e,
             h * v.y * v.z + v.x,
             0,
             h * v.x * v.z + v.y,
             h * v.y * v.z - v.x,
             h * v.z * v.z + e,
             0,
             0,
             0,
             0,
             1 };
}

inline mat4 mat4::rotation(quat const& q)
{
    return { 2 * (q.w * q.w + q.x * q.x) - 1,
             2 * (q.x * q.y + q.w * q.z),
             2 * (q.x * q.z - q.w * q.y),
             0,
             2 * (q.x * q.y - q.w * q.z),
             2 * (q.w * q.w + q.y * q.y) - 1,
             2 * (q.y * q.z + q.w * q.x),
             0,
             2 * (q.x * q.z + q.w * q.y),
             2 * (q.y * q.z - q.w * q.x),
             2 * (q.w * q.w + q.z * q.z) - 1,
             0,
             0,
             0,
             0,
             1 };
}

inline mat4 mat4::ortho(float left, float right, float bottom, float top, float znear, float zfar)
{
    auto dx = right - left;
    auto dy = top - bottom;
    auto dz = zfar - znear;

    return { 2 / dx,
             0,
             0,
             0,
             0,
             2 / dy,
             0,
             0,
             0,
             0,
             2 / dz,
             0,
             -(right + left) / dx,
             -(top + bottom) / dy,
             -(zfar + znear) / dz,
             1 };
}

inline mat4 mat4::frustum(float left, float right, float bottom, float top, float znear, float zfar)
{
    auto dx = right - left;
    auto dy = top - bottom;
    auto dz = zfar - znear;

    return { 2 * znear / dx,
             0,
             0,
             0,
             0,
             2 * znear / dy,
             0,
             0,
             (right + left) / dx,
             (top + bottom) / dy,
             -(zfar + znear) / dz,
             -1,
             0,
             0,
             -2 * zfar * znear / dz,
             0 };
}

inline mat4 mat4::perspective(float fov, float aspect, float znear, float zfar)
{
    auto f = 1 / tan(0.5f * fov);

    return { f / aspect,
             0,
             0,
             0,
             0,
             f,
             0,
             0,
             0,
             0,
             -(zfar + znear) / (zfar - znear),
             -1,
             0,
             0,
             -2 * zfar * znear / (zfar - znear),
             0 };
}

inline mat4 mat4::lookAt(vec3 const& position, vec3 const& target, vec3 const& up)
{
    auto z = normalize(position - target);
    auto x = normalize(cross(up, z));
    auto y = cross(z, x);

    return { x.x,
             y.x,
             z.x,
             0,
             x.y,
             y.y,
             z.y,
             0,
             x.z,
             y.z,
             z.z,
             0,
             -dot(position, x),
             -dot(position, y),
             -dot(position, z),
             1 };
}

//--------------------------------------------------------------------------------------------------
// Operators
//

inline mat4 operator-(mat4 const& a)
{
    return { -a(0), -a(1), -a(2), -a(3) };
}

inline mat4 operator+(mat4 const& a, mat4 const& b)
{
    return { a(0) + b(0), a(1) + b(1), a(2) + b(2), a(3) + b(3) };
}

inline mat4 operator-(mat4 const& a, mat4 const& b)
{
    return { a(0) - b(0), a(1) - b(1), a(2) - b(2), a(3) - b(3) };
}

#if !LIB_MATH_NO_ALIGNED_VECTOR_TYPES

inline mat4 operator*(mat4 const& a, mat4 const& b)
{
    mat4 out;

    auto p = out.data();

    auto a0 = _mm_load_ps(a.col0.data());
    auto a1 = _mm_load_ps(a.col1.data());
    auto a2 = _mm_load_ps(a.col2.data());
    auto a3 = _mm_load_ps(a.col3.data());

    for (int k = 3; k >= 0; --k)
    {
        // _mm_set1_ps should be optimized to _mm_load_ss with a subsequent _mm_shuffle_ps

        auto b0 = _mm_mul_ps(a0, _mm_set1_ps(b(0,k)));
        auto b1 = _mm_mul_ps(a1, _mm_set1_ps(b(1,k)));
        auto b2 = _mm_mul_ps(a2, _mm_set1_ps(b(2,k)));
        auto b3 = _mm_mul_ps(a3, _mm_set1_ps(b(3,k)));

        _mm_store_ps(p + 4 * k, _mm_add_ps(b0, _mm_add_ps(b1, _mm_add_ps(b2, b3))));
    }

    return out;
}

#else // !LIB_MATH_NO_ALIGNED_VECTOR_TYPES

inline mat4 operator*(mat4 const& a, mat4 const& b)
{
    mat4 out;

    for (int c = 0; c < 4; ++c)
    {
        for (int r = 0; r < 4; ++r)
        {
            out(r, c) = a(r,0) * b(0,c) + a(r,1) * b(1,c) + a(r,2) * b(2,c) + a(r,3) * b(3,c);
        }
    }

    return out;
}

#endif // LIB_MATH_NO_ALIGNED_VECTOR_TYPES

inline vec4 operator*(mat4 const& m, vec4 const& v)
{
    return { m(0,0) * v.x + m(0,1) * v.y + m(0,2) * v.z + m(0,3) * v.w,
             m(1,0) * v.x + m(1,1) * v.y + m(1,2) * v.z + m(1,3) * v.w,
             m(2,0) * v.x + m(2,1) * v.y + m(2,2) * v.z + m(2,3) * v.w,
             m(3,0) * v.x + m(3,1) * v.y + m(3,2) * v.z + m(3,3) * v.w };
}

inline mat4 operator*(mat4 const& a, float s)
{
    return { s * a(0), s * a(1), s * a(2), s * a(3) };
}

inline mat4 operator*(float s, mat4 const& a)
{
    return a * s;
}

inline mat4& operator+=(mat4& a, mat4 const& b)
{
    return a = a + b;
}

inline mat4& operator-=(mat4& a, mat4 const& b)
{
    return a = a - b;
}

inline mat4& operator*=(mat4& a, mat4 const& b)
{
    return a = a * b;
}

inline mat4& operator*=(mat4& a, float b)
{
    return a = a * b;
}

//--------------------------------------------------------------------------------------------------
// Geometric functions
//

// Returns the inf-norm of the given matrix A, i.e. the maximum of its elements
inline float norm_inf(mat4 const& a)
{
    float a0 = max_element(abs(a(0)));
    float a1 = max_element(abs(a(1)));
    float a2 = max_element(abs(a(2)));
    float a3 = max_element(abs(a(3)));

    return max(a0, a1, a2, a3);
}

// Returns the transpose of the given matrix A
inline mat4 transpose(mat4 const& a)
{
    return { a(0,0), a(0,1), a(0,2), a(0,3),
             a(1,0), a(1,1), a(1,2), a(1,3),
             a(2,0), a(2,1), a(2,2), a(2,3),
             a(3,0), a(3,1), a(3,2), a(3,3) };
}

// Returns the inverse of the given matrix A.
// NOTE: The matrix must be invertible!
inline mat4 inverse(mat4 const& a)
{
    auto s0 = det2(a(0,0), a(0,1), a(1,0), a(1,1));
    auto s1 = det2(a(0,0), a(0,2), a(1,0), a(1,2));
    auto s2 = det2(a(0,0), a(0,3), a(1,0), a(1,3));
    auto s3 = det2(a(0,1), a(0,2), a(1,1), a(1,2));
    auto s4 = det2(a(0,1), a(0,3), a(1,1), a(1,3));
    auto s5 = det2(a(0,2), a(0,3), a(1,2), a(1,3));
    auto c5 = det2(a(2,2), a(2,3), a(3,2), a(3,3));
    auto c4 = det2(a(2,1), a(2,3), a(3,1), a(3,3));
    auto c3 = det2(a(2,1), a(2,2), a(3,1), a(3,2));
    auto c2 = det2(a(2,0), a(2,3), a(3,0), a(3,3));
    auto c1 = det2(a(2,0), a(2,2), a(3,0), a(3,2));
    auto c0 = det2(a(2,0), a(2,1), a(3,0), a(3,1));

    auto det = s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0;

    return { (+a(1,1) * c5 - a(1,2) * c4 + a(1,3) * c3) / det,
             (-a(1,0) * c5 + a(1,2) * c2 + a(1,3) * c1) / det,
             (+a(1,0) * c4 - a(1,1) * c2 + a(1,3) * c0) / det,
             (-a(1,0) * c3 + a(1,1) * c1 + a(1,2) * c0) / det,
             (-a(0,1) * c5 + a(0,2) * c4 - a(0,3) * c3) / det,
             (+a(0,0) * c5 - a(0,2) * c2 + a(0,3) * c1) / det,
             (-a(0,0) * c4 + a(0,1) * c2 - a(0,3) * c0) / det,
             (+a(0,0) * c3 - a(0,1) * c1 + a(0,2) * c0) / det,
             (+a(3,1) * s5 - a(3,2) * s4 + a(3,3) * s3) / det,
             (-a(3,0) * s5 + a(3,2) * s2 - a(3,3) * s1) / det,
             (+a(3,0) * s4 - a(3,1) * s2 + a(3,3) * s0) / det,
             (-a(3,0) * s3 + a(3,1) * s1 - a(3,2) * s0) / det,
             (-a(2,1) * s5 + a(2,2) * s4 - a(2,3) * s3) / det,
             (+a(2,0) * s5 - a(2,2) * s2 + a(2,3) * s1) / det,
             (-a(2,0) * s4 + a(2,1) * s2 - a(2,3) * s0) / det,
             (+a(2,0) * s3 - a(2,1) * s1 + a(2,2) * s0) / det };
}

// Multiplies the upper-left 3x3 submatrices of A and B
inline mat4 mul3x3(mat4 const& a, mat4 const& b)
{
    return { a(0,0) * b(0,0) + a(0,1) * b(1,0) + a(0,2) * b(2,0),
             a(1,0) * b(0,0) + a(1,1) * b(1,0) + a(1,2) * b(2,0),
             a(2,0) * b(0,0) + a(2,1) * b(1,0) + a(2,2) * b(2,0),
             0,
             a(0,0) * b(0,1) + a(0,1) * b(1,1) + a(0,2) * b(2,1),
             a(1,0) * b(0,1) + a(1,1) * b(1,1) + a(1,2) * b(2,1),
             a(2,0) * b(0,1) + a(2,1) * b(1,1) + a(2,2) * b(2,1),
             0,
             a(0,0) * b(0,2) + a(0,1) * b(1,2) + a(0,2) * b(2,2),
             a(1,0) * b(0,2) + a(1,1) * b(1,2) + a(1,2) * b(2,2),
             a(2,0) * b(0,2) + a(2,1) * b(1,2) + a(2,2) * b(2,2),
             0,
             0,
             0,
             0,
             1 };
}

// Returns the transpose of the upper-left 3x3 submat4 of a
inline mat4 transpose3x3(mat4 const& a)
{
    return { a(0,0), a(0,1), a(0,2), 0,
             a(1,0), a(1,1), a(1,2), 0,
             a(2,0), a(2,1), a(2,2), 0,
             0, 0, 0, 1 };
}

// Returns the inverse of the upper-left 3x3 submat4 of a
inline mat4 inverse3x3(mat4 const& a)
{
    auto t00 =  det2(a(1,1), a(1,2), a(2,1), a(2,2));
    auto t10 = -det2(a(0,1), a(0,2), a(2,1), a(2,2));
    auto t20 =  det2(a(0,1), a(0,2), a(1,1), a(1,2));
    auto t01 = -det2(a(1,0), a(1,2), a(2,0), a(2,2));
    auto t11 =  det2(a(0,0), a(0,2), a(2,0), a(2,2));
    auto t21 = -det2(a(0,0), a(0,2), a(1,0), a(1,2));
    auto t02 =  det2(a(1,0), a(1,1), a(2,0), a(2,1));
    auto t12 = -det2(a(0,0), a(0,1), a(2,0), a(2,1));
    auto t22 =  det2(a(0,0), a(0,1), a(1,0), a(1,1));

    auto det = a(0,0) * t00 + a(1,0) * t10 + a(2,0) * t20;

    return { t00 / det, t01 / det, t02 / det, 0,
             t10 / det, t11 / det, t12 / det, 0,
             t20 / det, t21 / det, t22 / det, 0,
             0, 0, 0, 1 };
}

// Returns the upper-left 3x3 submat4 of M.
inline mat4 upper3x3(mat4 const& a)
{
    return { a(0,0), a(1,0), a(2,0), 0,
             a(0,1), a(1,1), a(2,1), 0,
             a(0,2), a(1,2), a(2,2), 0,
             0, 0, 0, 1 };
}

// Returns the rotation angle for the given rotation matrix.
// Only the upper-left 3x3 submat4 is used and assumed to be a proper rotation matrix.
inline float rotation_angle(mat4 const& m)
{
    return acos(0.5f * (m(0,0) + m(1,1) + m(2,2) - 1));
}

// Returns the rotation axis for the rotation matrix.
// Only the upper-left 3x3 submat4 is used and assumed to be a proper rotation matrix.
inline vec3 rotation_axis(mat4 const& m)
{
    return normalize(vec3(m(2,1) - m(1,2), m(0,2) - m(2,0), m(1,0) - m(0,1)));
}

//--------------------------------------------------------------------------------------------------
// Vector transformations
//

// Returns M * (v,0)
inline vec4 xform(mat4 const& m, vec3 const& v)
{
    return { m(0,0) * v.x + m(0,1) * v.y + m(0,2) * v.z + m(0,3),
             m(1,0) * v.x + m(1,1) * v.y + m(1,2) * v.z + m(1,3),
             m(2,0) * v.x + m(2,1) * v.y + m(2,2) * v.z + m(2,3),
             m(3,0) * v.x + m(3,1) * v.y + m(3,2) * v.z + m(3,3) };
}

// Returns M^T * (v,0)
inline vec4 xform_tr(mat4 const& m, vec3 const& v)
{
    return { m(0,0) * v.x + m(1,0) * v.y + m(2,0) * v.z + m(3,0),
             m(0,1) * v.x + m(1,1) * v.y + m(2,1) * v.z + m(3,1),
             m(0,2) * v.x + m(1,2) * v.y + m(2,2) * v.z + m(3,2),
             m(0,3) * v.x + m(1,3) * v.y + m(2,3) * v.z + m(3,3) };
}

// Returns M * (v,0) projected back onto w = 1
inline vec3 xform_point(mat4 const& m, vec3 const& v)
{
    auto t = xform(m, v);

    return t.xyz() / t.w;
}

// Returns M^T * (v,0) projected back onto w = 1
inline vec3 xform_point_tr(mat4 const& m, vec3 const& v)
{
    auto t = xform_tr(m, v);

    return t.xyz() / t.w;
}

// Returns A * v, where A is the upper-left 3x3 submat4 of M
inline vec3 xform_vector(mat4 const& m, vec3 const& v)
{
    return { m(0,0) * v.x + m(0,1) * v.y + m(0,2) * v.z,
             m(1,0) * v.x + m(1,1) * v.y + m(1,2) * v.z,
             m(2,0) * v.x + m(2,1) * v.y + m(2,2) * v.z };
}

// Returns A^T * v, where A is the upper-left 3x3 submat4 of M
inline vec3 xform_vector_tr(mat4 const& m, vec3 const& v)
{
    return { m(0,0) * v.x + m(1,0) * v.y + m(2,0) * v.z,
             m(0,1) * v.x + m(1,1) * v.y + m(2,1) * v.z,
             m(0,2) * v.x + m(1,2) * v.y + m(2,2) * v.z };
}

//==================================================================================================
// mat3
//==================================================================================================

//--------------------------------------------------------------------------------------------------
// Implementation
//

inline mat3::mat3(float m00, float m10, float m20,
                  float m01, float m11, float m21,
                  float m02, float m12, float m22)
    : col0(m00, m10, m20)
    , col1(m01, m11, m21)
    , col2(m02, m12, m22)
{
}

inline mat3::mat3(float m00, float m11, float m22)
    : col0(m00, 0, 0)
    , col1(0, m11, 0)
    , col2(0, 0, m22)
{
}

inline mat3::mat3(float const p[9])
    : col0(&p[0])
    , col1(&p[3])
    , col2(&p[6])
{
}

inline mat3::mat3(vec3 const& c0, vec3 const& c1, vec3 const& c2)
    : col0(c0)
    , col1(c1)
    , col2(c2)
{
}

inline mat3::mat3(mat4 const& m)
    : col0(m.col0)
    , col1(m.col1)
    , col2(m.col2)
{
}

inline float* mat3::data()
{
    return col0.data();
}

inline float const* mat3::data() const
{
    return col0.data();
}

inline vec3& mat3::operator()(size_t col)
{
    return (&col0)[col];
}

inline vec3 const& mat3::operator()(size_t col) const
{
    return (&col0)[col];
}

inline float& mat3::operator()(size_t row, size_t col)
{
    return (operator()(col))[row];
}

inline float const& mat3::operator()(size_t row, size_t col) const
{
    return (operator()(col))[row];
}

inline float& mat3::operator[](size_t index)
{
    return data()[index];
}

inline float const& mat3::operator[](size_t index) const
{
    return data()[index];
}

//--------------------------------------------------------------------------------------------------
// Operators
//

inline mat3 operator-(mat3 const& a)
{
    return { -a(0), -a(1), -a(2) };
}

inline mat3 operator+(mat3 const& a, mat3 const& b)
{
    return { a(0) + b(0), a(1) + b(1), a(2) + b(2) };
}

inline mat3 operator-(mat3 const& a, mat3 const& b)
{
    return { a(0) - b(0), a(1) - b(1), a(2) - b(2) };
}

inline mat3 operator*(mat3 const& a, mat3 const& b)
{
    mat3 out;

    for (int c = 0; c < 3; ++c)
    {
        for (int r = 0; r < 3; ++r)
        {
            out(r, c) = a(r,0) * b(0,c) + a(r,1) * b(1,c) + a(r,2) * b(2,c);
        }
    }

    return out;
}

inline vec3 operator*(mat3 const& m, vec3 const& v)
{
    return { m(0,0) * v.x + m(0,1) * v.y + m(0,2) * v.z,
             m(1,0) * v.x + m(1,1) * v.y + m(1,2) * v.z,
             m(2,0) * v.x + m(2,1) * v.y + m(2,2) * v.z };
}

inline mat3 operator*(mat3 const& a, float s)
{
    return { s * a(0), s * a(1), s * a(2) };
}

inline mat3 operator*(float s, mat3 const& a)
{
    return a * s;
}

inline mat3& operator+=(mat3& a, mat3 const& b)
{
    return a = a + b;
}

inline mat3& operator-=(mat3& a, mat3 const& b)
{
    return a = a - b;
}

inline mat3& operator*=(mat3& a, mat3 const& b)
{
    return a = a * b;
}

inline mat3& operator*=(mat3& a, float b)
{
    return a = a * b;
}

//--------------------------------------------------------------------------------------------------
// Geometric functions
//

// Returns the transpose of the given matrix A
inline mat3 transpose(mat3 const& a)
{
    return { a(0,0), a(0,1), a(0,2),
             a(1,0), a(1,1), a(1,2),
             a(2,0), a(2,1), a(2,2) };
}

// Returns the inverse of the given matrix A.
// NOTE: The matrix must be invertible!
inline mat3 inverse(mat3 const& a)
{
    auto t00 =  det2(a(1,1), a(1,2), a(2,1), a(2,2));
    auto t10 = -det2(a(0,1), a(0,2), a(2,1), a(2,2));
    auto t20 =  det2(a(0,1), a(0,2), a(1,1), a(1,2));
    auto t01 = -det2(a(1,0), a(1,2), a(2,0), a(2,2));
    auto t11 =  det2(a(0,0), a(0,2), a(2,0), a(2,2));
    auto t21 = -det2(a(0,0), a(0,2), a(1,0), a(1,2));
    auto t02 =  det2(a(1,0), a(1,1), a(2,0), a(2,1));
    auto t12 = -det2(a(0,0), a(0,1), a(2,0), a(2,1));
    auto t22 =  det2(a(0,0), a(0,1), a(1,0), a(1,1));

    auto det = a(0,0) * t00 + a(1,0) * t10 + a(2,0) * t20;

    return { t00 / det, t01 / det, t02 / det,
             t10 / det, t11 / det, t12 / det,
             t20 / det, t21 / det, t22 / det };
}

// Returns the rotation angle for the given rotation matrix.
// Only the upper-left 3x3 submat4 is used and assumed to be a proper rotation matrix.
inline float rotation_angle(mat3 const& m)
{
    return acos(0.5f * (m(0,0) + m(1,1) + m(2,2) - 1));
}

// Returns the rotation axis for the rotation matrix.
// Only the upper-left 3x3 submat4 is used and assumed to be a proper rotation matrix.
inline vec3 rotation_axis(mat3 const& m)
{
    return normalize(vec3(m(2,1) - m(1,2), m(0,2) - m(2,0), m(1,0) - m(0,1)));
}

} // namespace math
