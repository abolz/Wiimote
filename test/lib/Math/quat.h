// This file is distributed under the MIT license.
// See the LICENSE file for details.

#pragma once

#define LIB_MATH_QUAT_H 1

#include "lib/Math/vec.h"

namespace math
{

//--------------------------------------------------------------------------------------------------
// quat
//

struct quat : aligned_base<quat>
{
    float w; // scalar part
    float x; // vector part
    float y;
    float z;

    // Default constructor
    quat() = default;

    // Construct from components
    quat(float w, float x, float y, float z);

    // Construct from scalar and vector part
    quat(float w, vec3 const& v);

    // Construct from a 3D vector
    explicit quat(vec3 const& v);

    // Returns the unit quaternion
    static quat identity();

    // Constructs a quaternion q from a rotation matrix M such that q v q* = M v.
    // NOTE: Only the upper-left 3x3 submatrix of m is used and is assumed to be a proper rotation matrix.
    static quat rotation(mat4 const& m);

    // Constructs a quaternion, which represents a rotation around the given axis
    static quat rotation(vec3 const& axis, float angle);

    // Constructs a quaternion, which rotates <from> into <to>
    static quat rotation(vec3 const& from, vec3 const& to);
};

#if !LIB_MATH_NO_ALIGNED_VECTOR_TYPES
static_assert(sizeof(quat) == 16 && __alignof(quat) == 16, "");
#endif

} // namespace math

#include "lib/Math/quat.inl"
