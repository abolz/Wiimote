// This file is distributed under the MIT license.
// See the LICENSE file for details.

#pragma once

#define LIB_MATH_MAT_H 1

#include "lib/Math/vec.h"

namespace math
{

//--------------------------------------------------------------------------------------------------
// mat4
//

struct mat4 : aligned_base<mat4>
{
    vec4 col0;
    vec4 col1;
    vec4 col2;
    vec4 col3;

    // Default constructor
    mat4() = default;

    // Construct from elements
    mat4(float m00, float m10, float m20, float m30,
         float m01, float m11, float m21, float m31,
         float m02, float m12, float m22, float m32,
         float m03, float m13, float m23, float m33);

    // Construct from diagonal
    mat4(float m00, float m11, float m22, float m33);

    // Construct from elements
    explicit mat4(float const p[16]);

    // Construct from columns
    mat4(vec4 const& c0, vec4 const& c1, vec4 const& c2, vec4 const& c3);

    // Construct from columns
    mat4(vec3 const& c0, vec3 const& c1, vec3 const& c2);

    // Construct from another matrix
    explicit mat4(mat3 const& m);

    // Returns a pointer to the matrix elements
    float* data();

    // Returns a pointer to the matrix elements
    float const* data() const;

    // Returns a reference to column n
    vec4& operator()(size_t n);

    // Returns a reference to column n
    vec4 const& operator()(size_t n) const;

    // Returns a reference to the element at (row, col)
    float& operator()(size_t row, size_t col);

    // Returns a reference to the element at (row, col)
    float const& operator()(size_t row, size_t col) const;

    // Linear index into the matrix
    float& operator[](size_t index);

    // Linear index into the matrix
    float const& operator[](size_t index) const;

    // Returns the identity matrix
    static mat4 identity();

    // Returns the zero matrix
    static mat4 zero();

    // Constructs a translation matrix
    static mat4 translation(vec3 const& t);

    // Constructs a scaling matrix
    static mat4 scaling(vec3 const& s);

    // Constructs a rotation matrix from a rotation tvec and an angle
    static mat4 rotation(vec3 const& axis, float angle);

    // Constructs a rotation matrix, which rotates <from> into <to>
    static mat4 rotation(vec3 const& from, vec3 const& to);

    // Constructs a rotation matrix M from a quaternion, such that M v = q v q*
    static mat4 rotation(quat const& q);

    // Constructs an orthogonal projection matrix (compatible with OpenGL)
    static mat4 ortho(float left, float right, float bottom, float top, float znear, float zfar);

    // Constructs a perspective projection matrix (compatible with OpenGL)
    static mat4 frustum(float left, float right, float bottom, float top, float znear, float zfar);

    // Constructs a perspective projection matrix (compatible with OpenGL)
    static mat4 perspective(float fov, float aspect, float znear, float zfar);

    // Constructs a look-at matrix (compatible with OpenGL)
    static mat4 lookAt(vec3 const& position, vec3 const& target, vec3 const& up);
};

#if !LIB_MATH_NO_ALIGNED_VECTOR_TYPES
static_assert(sizeof(mat4) == 64 && __alignof(mat4) == 64, "");
#endif

//--------------------------------------------------------------------------------------------------
// mat3
//

struct mat3
{
    vec3 col0;
    vec3 col1;
    vec3 col2;

    // Default constructor
    mat3() = default;

    // Construct from elements
    mat3(float m00, float m10, float m20,
         float m01, float m11, float m21,
         float m02, float m12, float m22);

    // Construct from diagonal
    mat3(float m00, float m11, float m22);

    // Construct from elements
    explicit mat3(float const p[9]);

    // Construct from columns
    mat3(vec3 const& c0, vec3 const& c1, vec3 const& c2);

    // Construct from another matrix
    explicit mat3(mat4 const& m);

    // Returns a pointer to the matrix elements
    float* data();

    // Returns a pointer to the matrix elements
    float const* data() const;

    // Returns a reference to column n
    vec3& operator()(size_t n);

    // Returns a reference to column n
    vec3 const& operator()(size_t n) const;

    // Returns a reference to the element at (row, col)
    float& operator()(size_t row, size_t col);

    // Returns a reference to the element at (row, col)
    float const& operator()(size_t row, size_t col) const;

    // Linear index into the matrix
    float& operator[](size_t index);

    // Linear index into the matrix
    float const& operator[](size_t index) const;
};

} // namespace math

#include "lib/Math/mat.inl"
