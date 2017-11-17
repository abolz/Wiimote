// This file is distributed under the MIT license.
// See the LICENSE file for details.

#pragma once

#define LIB_MATH_VEC_H 1

#include "lib/Math/common.h"

#include <type_traits>

namespace math
{

//==================================================================================================
//
//==================================================================================================

//--------------------------------------------------------------------------------------------------
// vec2
//

template <class T>
struct vec<T, 2> : aligned_base<vec<T, 2>>
{
    using value_type = T;

    T x;
    T y;

    // Default constructor
    vec() = default;

    // Construct from components
    vec(T x, T y);

    // Construct from scalar
    explicit vec(T s);

    // Construct from components
    explicit vec(T const p[2]);

    // Construct from a 2d-vec
    template <class U>
    explicit vec(vec<U, 2> const& u);

    // Construct from a 3d-vec
    template <class U>
    explicit vec(vec<U, 3> const& u);

    // Construct from a 4d-vec
    template <class U>
    explicit vec(vec<U, 4> const& u);

    // Returns a pointer to the vec data
    T* data();

    // Returns a pointer to the vec data
    T const* data() const;

    // Array access
    T& operator[](size_t index);

    // Array access
    T const& operator[](size_t index) const;

    // Construct from components
    template <class U>
    static vec from_coords(U const& u);

    // Construct from components
    template <class U>
    static vec from_array(U const& u);
};

#if !LIB_MATH_NO_ALIGNED_VECTOR_TYPES
static_assert(sizeof(vec2 ) ==  8 && __alignof(vec2 ) ==  8, "");
static_assert(sizeof(vec2d) == 16 && __alignof(vec2d) == 16, "");
static_assert(sizeof(vec2i) ==  8 && __alignof(vec2i) ==  8, "");
static_assert(sizeof(vec2u) ==  8 && __alignof(vec2u) ==  8, "");
#endif

//--------------------------------------------------------------------------------------------------
// vec3
//

template <class T>
struct vec<T, 3> : aligned_base<vec<T, 3>>
{
    using value_type = T;

    T x;
    T y;
    T z;

    // Default constructor
    vec() = default;

    // Construct from components
    vec(T x, T y, T z);

    // Construct from scalar
    explicit vec(T s);

    // Construct from components
    explicit vec(T const p[3]);

    // Construct from a 2d-vec
    template <class U>
    explicit vec(vec<U, 2> const& u, Id<U> z = U(0));

    // Construct from a 3d-vec
    template <class U>
    explicit vec(vec<U, 3> const& u);

    // Construct from a 4d-vec
    template <class U>
    explicit vec(vec<U, 4> const& u);

    // Returns a pointer to the vec data
    T* data();

    // Returns a pointer to the vec data
    T const* data() const;

    // Array access
    T& operator[](size_t index);

    // Array access
    T const& operator[](size_t index) const;

    // Returns the xy-components of the vec
    vec<T, 2>& xy();

    // Returns the xy-components of the vec
    vec<T, 2> const& xy() const;

    // Construct from components
    template <class U>
    static vec from_coords(U const& u);

    // Construct from components
    template <class U>
    static vec from_array(U const& u);
};

//--------------------------------------------------------------------------------------------------
// vec4
//

template <class T>
struct vec<T, 4> : aligned_base<vec<T, 4>>
{
    using value_type = T;

    T x;
    T y;
    T z;
    T w;

    // Default constructor
    vec() = default;

    // Construct from components
    vec(T x, T y, T z, T w);

    // Construct from scalar
    explicit vec(T s);

    // Construct from components
    explicit vec(T const p[4]);

    // Construct from a 2d-vec
    template <class U>
    explicit vec(vec<U, 2> const& u, Id<U> z = U(0), Id<U> w = U(0));

    // Construct from a 3d-vec
    template <class U>
    explicit vec(vec<U, 3> const& u, Id<U> w = U(0));

    // Construct from a 4d-vec
    template <class U>
    explicit vec(vec<U, 4> const& u);

    // Returns a pointer to the vec data
    T* data();

    // Returns a pointer to the vec data
    T const* data() const;

    // Array access
    T& operator[](size_t index);

    // Array access
    T const& operator[](size_t index) const;

    // Returns the xy-components of the vec
    vec<T, 2>& xy();

    // Returns the xy-components of the vec
    vec<T, 2> const& xy() const;

    // Returns the xyz-components of the vec
    vec<T, 3>& xyz();

    // Returns the xyz-components of the vec
    vec<T, 3> const& xyz() const;

    // Construct from components
    template <class U>
    static vec from_coords(U const& u);

    // Construct from components
    template <class U>
    static vec from_array(U const& u);
};

#if !LIB_MATH_NO_ALIGNED_VECTOR_TYPES
static_assert(sizeof(vec4 ) == 16 && __alignof(vec4 ) == 16, "");
static_assert(sizeof(vec4d) == 32 && __alignof(vec4d) == 32, "");
static_assert(sizeof(vec4i) == 16 && __alignof(vec4i) == 16, "");
static_assert(sizeof(vec4u) == 16 && __alignof(vec4u) == 16, "");
#endif

//==================================================================================================
// vec2
//==================================================================================================

//--------------------------------------------------------------------------------------------------
// Implementation
//

template <class T>
vec<T, 2>::vec(T x, T y)
    : x(x)
    , y(y)
{
}

template <class T>
vec<T, 2>::vec(T s)
    : x(s)
    , y(s)
{
}

template <class T>
vec<T, 2>::vec(T const p[2])
    : x(p[0])
    , y(p[1])
{
}

template <class T>
template <class U>
vec<T, 2>::vec(vec<U, 2> const& u)
    : x(static_cast<T>(u.x))
    , y(static_cast<T>(u.y))
{
}

template <class T>
template <class U>
vec<T, 2>::vec(vec<U, 3> const& u)
    : x(static_cast<T>(u.x))
    , y(static_cast<T>(u.y))
{
}

template <class T>
template <class U>
vec<T, 2>::vec(vec<U, 4> const& u)
    : x(static_cast<T>(u.x))
    , y(static_cast<T>(u.y))
{
}

template <class T>
T* vec<T, 2>::data()
{
    return &x;
}

template <class T>
T const* vec<T, 2>::data() const
{
    return &x;
}

template <class T>
T& vec<T, 2>::operator[](size_t index)
{
    return data()[index];
}

template <class T>
T const& vec<T, 2>::operator[](size_t index) const
{
    return data()[index];
}

template <class T>
template <class U>
vec<T, 2> vec<T, 2>::from_coords(U const& u)
{
    return { static_cast<T>(u.x), static_cast<T>(u.y) };
}

template <class T>
template <class U>
vec<T, 2> vec<T, 2>::from_array(U const& u)
{
    return { static_cast<T>(u[0]), static_cast<T>(u[1]) };
}

//--------------------------------------------------------------------------------------------------
// Comparison operators
//

template <class T>
bool operator==(vec<T, 2> const& u, vec<T, 2> const& v)
{
    return u.x == v.x && u.y == v.y;
}

template <class T>
bool operator<(vec<T, 2> const& u, vec<T, 2> const& v)
{
    return u.x < v.x || (u.x == v.x && (u.y < v.y));
}

//--------------------------------------------------------------------------------------------------
// Arithmetic operators
//

template <class T>
vec<T, 2> operator-(vec<T, 2> const& u)
{
    return { -u.x, -u.y };
}

template <class T>
vec<T, 2> operator+(vec<T, 2> const& u, vec<T, 2> const& v)
{
    return { u.x + v.x, u.y + v.y };
}

template <class T>
vec<T, 2> operator-(vec<T, 2> const& u, vec<T, 2> const& v)
{
    return { u.x - v.x, u.y - v.y };
}

template <class T>
vec<T, 2> operator*(vec<T, 2> const& u, vec<T, 2> const& v)
{
    return { u.x * v.x, u.y * v.y };
}

template <class T>
vec<T, 2> operator/(vec<T, 2> const& u, vec<T, 2> const& v)
{
    return { u.x / v.x, u.y / v.y };
}

template <class T>
vec<T, 2> operator+(vec<T, 2> const& u, Id<T> v)
{
    return { u.x + v, u.y + v };
}

template <class T>
vec<T, 2> operator-(vec<T, 2> const& u, Id<T> v)
{
    return { u.x - v, u.y - v };
}

template <class T>
vec<T, 2> operator*(vec<T, 2> const& u, Id<T> v)
{
    return { u.x * v, u.y * v };
}

template <class T>
vec<T, 2> operator/(vec<T, 2> const& u, Id<T> v)
{
    return { u.x / v, u.y / v };
}

template <class T>
vec<T, 2> operator+(Id<T> u, vec<T, 2> const& v)
{
    return { u + v.x, u + v.y };
}

template <class T>
vec<T, 2> operator-(Id<T> u, vec<T, 2> const& v)
{
    return { u - v.x, u - v.y };
}

template <class T>
vec<T, 2> operator*(Id<T> u, vec<T, 2> const& v)
{
    return { u * v.x, u * v.y };
}

template <class T>
vec<T, 2> operator/(Id<T> u, vec<T, 2> const& v)
{
    return { u / v.x, u / v.y };
}

//--------------------------------------------------------------------------------------------------
// Component-wise operations
//

// Returns the sum of the components
template <class T>
T hadd(vec<T, 2> const& u)
{
    return u.x + u.y;
}

// Returns the product of all components
template <class T>
T hmul(vec<T, 2> const& u)
{
    return u.x * u.y;
}

// Returns the smallest element of the vec
template <class T>
T min_element(vec<T, 2> const& u)
{
    return min(u.x, u.y);
}

// Returns the largest element of the vec
template <class T>
T max_element(vec<T, 2> const& u)
{
    return max(u.x, u.y);
}

// Returns the component-wise minimum of the two vectors
template <class T>
vec<T, 2> min(vec<T, 2> const& u, vec<T, 2> const& v)
{
    return { min(u.x, v.x), min(u.y, v.y) };
}

// Returns the component-wise maximum of the two vectors
template <class T>
vec<T, 2> max(vec<T, 2> const& u, vec<T, 2> const& v)
{
    return { max(u.x, v.x), max(u.y, v.y) };
}

//--------------------------------------------------------------------------------------------------
// Geometric functions
//

// Returns the dot product of the two vecs
template <class T>
T dot(vec<T, 2> const& u, vec<T, 2> const& v)
{
    return u.x * v.x + u.y * v.y;
}

//==================================================================================================
// vec3
//==================================================================================================

//--------------------------------------------------------------------------------------------------
// Implementation
//

template <class T>
vec<T, 3>::vec(T x, T y, T z)
    : x(x)
    , y(y)
    , z(z)
{
}

template <class T>
vec<T, 3>::vec(T s)
    : x(s)
    , y(s)
    , z(s)
{
}

template <class T>
vec<T, 3>::vec(T const p[3])
    : x(p[0])
    , y(p[1])
    , z(p[2])
{
}

template <class T>
template <class U>
vec<T, 3>::vec(vec<U, 2> const& u, Id<U> z)
    : x(static_cast<T>(u.x))
    , y(static_cast<T>(u.y))
    , z(static_cast<T>(z))
{
}

template <class T>
template <class U>
vec<T, 3>::vec(vec<U, 3> const& u)
    : x(static_cast<T>(u.x))
    , y(static_cast<T>(u.y))
    , z(static_cast<T>(u.z))
{
}

template <class T>
template <class U>
vec<T, 3>::vec(vec<U, 4> const& u)
    : x(static_cast<T>(u.x))
    , y(static_cast<T>(u.y))
    , z(static_cast<T>(u.z))
{
}

template <class T>
T* vec<T, 3>::data()
{
    return &x;
}

template <class T>
T const* vec<T, 3>::data() const
{
    return &x;
}

template <class T>
T& vec<T, 3>::operator[](size_t index)
{
    return data()[index];
}

template <class T>
T const& vec<T, 3>::operator[](size_t index) const
{
    return data()[index];
}

template <class T>
vec<T, 2>& vec<T, 3>::xy()
{
    return *reinterpret_cast<vec<T, 2>*>(data());
}

template <class T>
vec<T, 2> const& vec<T, 3>::xy() const
{
    return *reinterpret_cast<vec<T, 2> const*>(data());
}

template <class T>
template <class U>
vec<T, 3> vec<T, 3>::from_coords(U const& u)
{
    return { static_cast<T>(u.x), static_cast<T>(u.y), static_cast<T>(u.z) };
}

template <class T>
template <class U>
vec<T, 3> vec<T, 3>::from_array(U const& u)
{
    return { static_cast<T>(u[0]), static_cast<T>(u[1]), static_cast<T>(u[2]) };
}

//--------------------------------------------------------------------------------------------------
// Comparison operators
//

template <class T>
bool operator==(vec<T, 3> const& u, vec<T, 3> const& v)
{
    return u.x == v.x && u.y == v.y && u.z == v.z;
}

template <class T>
bool operator<(vec<T, 3> const& u, vec<T, 3> const& v)
{
    return u.x < v.x || (u.x == v.x && (u.y < v.y || (u.y == v.y && (u.z < v.z))));
}

//--------------------------------------------------------------------------------------------------
// Arithmetic operators
//

template <class T>
vec<T, 3> operator-(vec<T, 3> const& u)
{
    return { -u.x, -u.y, -u.z };
}

template <class T>
vec<T, 3> operator+(vec<T, 3> const& u, vec<T, 3> const& v)
{
    return { u.x + v.x, u.y + v.y, u.z + v.z };
}

template <class T>
vec<T, 3> operator-(vec<T, 3> const& u, vec<T, 3> const& v)
{
    return { u.x - v.x, u.y - v.y, u.z - v.z };
}

template <class T>
vec<T, 3> operator*(vec<T, 3> const& u, vec<T, 3> const& v)
{
    return { u.x * v.x, u.y * v.y, u.z * v.z };
}

template <class T>
vec<T, 3> operator/(vec<T, 3> const& u, vec<T, 3> const& v)
{
    return { u.x / v.x, u.y / v.y, u.z / v.z };
}

template <class T>
vec<T, 3> operator+(vec<T, 3> const& u, Id<T> v)
{
    return { u.x + v, u.y + v, u.z + v };
}

template <class T>
vec<T, 3> operator-(vec<T, 3> const& u, Id<T> v)
{
    return { u.x - v, u.y - v, u.z - v };
}

template <class T>
vec<T, 3> operator*(vec<T, 3> const& u, Id<T> v)
{
    return { u.x * v, u.y * v, u.z * v };
}

template <class T>
vec<T, 3> operator/(vec<T, 3> const& u, Id<T> v)
{
    return { u.x / v, u.y / v, u.z / v };
}

template <class T>
vec<T, 3> operator+(Id<T> u, vec<T, 3> const& v)
{
    return { u + v.x, u + v.y, u + v.z };
}

template <class T>
vec<T, 3> operator-(Id<T> u, vec<T, 3> const& v)
{
    return { u - v.x, u - v.y, u - v.z };
}

template <class T>
vec<T, 3> operator*(Id<T> u, vec<T, 3> const& v)
{
    return { u * v.x, u * v.y, u * v.z };
}

template <class T>
vec<T, 3> operator/(Id<T> u, vec<T, 3> const& v)
{
    return { u / v.x, u / v.y, u / v.z };
}

//--------------------------------------------------------------------------------------------------
// Component-wise operations
//

// Returns the sum of the components
template <class T>
T hadd(vec<T, 3> const& u)
{
    return u.x + u.y + u.z;
}

// Returns the product of all components
template <class T>
T hmul(vec<T, 3> const& u)
{
    return u.x * u.y * u.z;
}

// Returns the smallest element of the vecor
template <class T>
T min_element(vec<T, 3> const& u)
{
    return min(min(u.x, u.y), u.z);
}

// Returns the largest element of the vecor
template <class T>
T max_element(vec<T, 3> const& u)
{
    return max(max(u.x, u.y), u.z);
}

// Returns the component-wise minimum of the two vectors
template <class T>
vec<T, 3> min(vec<T, 3> const& u, vec<T, 3> const& v)
{
    return { min(u.x, v.x), min(u.y, v.y), min(u.z, v.z) };
}

// Returns the component-wise maximum of the two vectors
template <class T>
vec<T, 3> max(vec<T, 3> const& u, vec<T, 3> const& v)
{
    return { max(u.x, v.x), max(u.y, v.y), max(u.z, v.z) };
}

//--------------------------------------------------------------------------------------------------
// Geometric functions
//

// Returns the dot product of the two vecor
template <class T>
T dot(vec<T, 3> const& u, vec<T, 3> const& v)
{
    return u.x * v.x + u.y * v.y + u.z * v.z;
}

// Returns a vecor perpendicular to the given vecors
template <class T>
vec<T, 3> cross(vec<T, 3> const& u, vec<T, 3> const& v)
{
    return { u.y * v.z - u.z * v.y,
             u.z * v.x - u.x * v.z,
             u.x * v.y - u.y * v.x };
}

// Map cube coordinates [-1,1] to sphere coordinates
template <class T>
vec<T, 3> map_cube_to_sphere(vec<T, 3> const& p)
{
    vec<T, 3> p2 = p * p;
    vec<T, 3> p0(p2.y, p2.z, p2.x);     // p2.yzx
    vec<T, 3> p1(p2.z, p2.x, p2.y);     // p2.zxy

    return p * sqrt(1.0 - 0.5 * p0 - 0.5 * p1 - (1.0/3.0) * (p0 * p1));
}

//==================================================================================================
// vec4
//==================================================================================================

//--------------------------------------------------------------------------------------------------
// Implementation
//

template <class T>
vec<T, 4>::vec(T x, T y, T z, T w)
    : x(x)
    , y(y)
    , z(z)
    , w(w)
{
}

template <class T>
vec<T, 4>::vec(T s)
    : x(s)
    , y(s)
    , z(s)
    , w(s)
{
}

template <class T>
vec<T, 4>::vec(T const p[4])
    : x(p[0])
    , y(p[1])
    , z(p[2])
    , w(p[3])
{
}

template <class T>
template <class U>
vec<T, 4>::vec(vec<U, 2> const& u, Id<U> z, Id<U> w)
    : x(static_cast<T>(u.x))
    , y(static_cast<T>(u.y))
    , z(static_cast<T>(z))
    , w(static_cast<T>(w))
{
}

template <class T>
template <class U>
vec<T, 4>::vec(vec<U, 3> const& u, Id<U> w)
    : x(static_cast<T>(u.x))
    , y(static_cast<T>(u.y))
    , z(static_cast<T>(u.z))
    , w(static_cast<T>(w))
{
}

template <class T>
template <class U>
vec<T, 4>::vec(vec<U, 4> const& u)
    : x(static_cast<T>(u.x))
    , y(static_cast<T>(u.y))
    , z(static_cast<T>(u.z))
    , w(static_cast<T>(u.w))
{
}

template <class T>
T* vec<T, 4>::data()
{
    return &x;
}

template <class T>
T const* vec<T, 4>::data() const
{
    return &x;
}

template <class T>
T& vec<T, 4>::operator[](size_t index)
{
    return data()[index];
}

template <class T>
T const& vec<T, 4>::operator[](size_t index) const
{
    return data()[index];
}

template <class T>
vec<T, 2>& vec<T, 4>::xy()
{
    return *reinterpret_cast<vec<T, 2>*>(data());
}

template <class T>
vec<T, 2> const& vec<T, 4>::xy() const
{
    return *reinterpret_cast<vec<T, 2> const*>(data());
}

template <class T>
vec<T, 3>& vec<T, 4>::xyz()
{
    return *reinterpret_cast<vec<T, 3>*>(data());
}

template <class T>
vec<T, 3> const& vec<T, 4>::xyz() const
{
    return *reinterpret_cast<vec<T, 3> const*>(data());
}

template <class T>
template <class U>
vec<T, 4> vec<T, 4>::from_coords(U const& u)
{
    return { static_cast<T>(u.x), static_cast<T>(u.y), static_cast<T>(u.z), static_cast<T>(u.w) };
}

template <class T>
template <class U>
vec<T, 4> vec<T, 4>::from_array(U const& u)
{
    return { static_cast<T>(u[0]), static_cast<T>(u[1]), static_cast<T>(u[2]), static_cast<T>(u[3]) };
}

//--------------------------------------------------------------------------------------------------
// Comparison operators
//

template <class T>
bool operator==(vec<T, 4> const& u, vec<T, 4> const& v)
{
    return u.x == v.x && u.y == v.y && u.z == v.z && u.w == v.w;
}

template <class T>
bool operator<(vec<T, 4> const& u, vec<T, 4> const& v)
{
    return u.x < v.x || (u.x == v.x && (u.y < v.y || (u.y == v.y && (u.z < v.z || (u.z == v.z && (u.w < v.w))))));
}

//--------------------------------------------------------------------------------------------------
// Arithmetic operators
//

template <class T>
vec<T, 4> operator-(vec<T, 4> const& u)
{
    return { -u.x, -u.y, -u.z, -u.w };
}

template <class T>
vec<T, 4> operator+(vec<T, 4> const& u, vec<T, 4> const& v)
{
    return { u.x + v.x, u.y + v.y, u.z + v.z, u.w + v.w };
}

template <class T>
vec<T, 4> operator-(vec<T, 4> const& u, vec<T, 4> const& v)
{
    return { u.x - v.x, u.y - v.y, u.z - v.z, u.w - v.w };
}

template <class T>
vec<T, 4> operator*(vec<T, 4> const& u, vec<T, 4> const& v)
{
    return { u.x * v.x, u.y * v.y, u.z * v.z, u.w * v.w };
}

template <class T>
vec<T, 4> operator/(vec<T, 4> const& u, vec<T, 4> const& v)
{
    return { u.x / v.x, u.y / v.y, u.z / v.z, u.w / v.w };
}

template <class T>
vec<T, 4> operator+(vec<T, 4> const& u, Id<T> v)
{
    return { u.x + v, u.y + v, u.z + v, u.w + v };
}

template <class T>
vec<T, 4> operator-(vec<T, 4> const& u, Id<T> v)
{
    return { u.x - v, u.y - v, u.z - v, u.w - v };
}

template <class T>
vec<T, 4> operator*(vec<T, 4> const& u, Id<T> v)
{
    return { u.x * v, u.y * v, u.z * v, u.w * v };
}

template <class T>
vec<T, 4> operator/(vec<T, 4> const& u, Id<T> v)
{
    return { u.x / v, u.y / v, u.z / v, u.w / v };
}

template <class T>
vec<T, 4> operator+(Id<T> u, vec<T, 4> const& v)
{
    return { u + v.x, u + v.y, u + v.z, u + v.w };
}

template <class T>
vec<T, 4> operator-(Id<T> u, vec<T, 4> const& v)
{
    return { u - v.x, u - v.y, u - v.z, u - v.w };
}

template <class T>
vec<T, 4> operator*(Id<T> u, vec<T, 4> const& v)
{
    return { u * v.x, u * v.y, u * v.z, u * v.w };
}

template <class T>
vec<T, 4> operator/(Id<T> u, vec<T, 4> const& v)
{
    return { u / v.x, u / v.y, u / v.z, u / v.w };
}

//--------------------------------------------------------------------------------------------------
// Component-wise operations
//

// Returns the sum of the components
template <class T>
T hadd(vec<T, 4> const& u)
{
    return u.x + u.y + u.z + u.w;
}

// Returns the product of all components
template <class T>
T hmul(vec<T, 4> const& u)
{
    return u.x * u.y * u.z * u.w;
}

// Returns the smallest element of the vec
template <class T>
T min_element(vec<T, 4> const& u)
{
    return min(min(u.x, u.y), min(u.z, u.w));
}

// Returns the largest element of the vec
template <class T>
T max_element(vec<T, 4> const& u)
{
    return max(max(u.x, u.y), max(u.z, u.w));
}

// Returns the component-wise minimum of the two vectors
template <class T>
vec<T, 4> min(vec<T, 4> const& u, vec<T, 4> const& v)
{
    return { min(u.x, v.x), min(u.y, v.y), min(u.z, v.z), min(u.w, v.w) };
}

// Returns the component-wise maximum of the two vectors
template <class T>
vec<T, 4> max(vec<T, 4> const& u, vec<T, 4> const& v)
{
    return { max(u.x, v.x), max(u.y, v.y), max(u.z, v.z), max(u.w, v.w) };
}

//--------------------------------------------------------------------------------------------------
// Geometric functions
//

// Returns the dot product of the two vecs
template <class T>
T dot(vec<T, 4> const& u, vec<T, 4> const& v)
{
    return u.x * v.x + u.y * v.y + u.z * v.z + u.w * v.w;
}

//==================================================================================================
// vec
//==================================================================================================

//--------------------------------------------------------------------------------------------------
// Comparison operators
//

template <class T, int N>
bool operator!=(vec<T, N> const& u, vec<T, N> const& v)
{
    return !(u == v);
}

template <class T, int N>
bool operator<=(vec<T, N> const& u, vec<T, N> const& v)
{
    return !(v < u);
}

template <class T, int N>
bool operator>(vec<T, N> const& u, vec<T, N> const& v)
{
    return v < u;
}

template <class T, int N>
bool operator>=(vec<T, N> const& u, vec<T, N> const& v)
{
    return !(u < v);
}

//--------------------------------------------------------------------------------------------------
// Arithmetic operators
//

template <class T, int N>
vec<T, N>& operator+=(vec<T, N>& u, vec<T, N> const& v)
{
    return u = u + v;
}

template <class T, int N>
vec<T, N>& operator-=(vec<T, N>& u, vec<T, N> const& v)
{
    return u = u - v;
}

template <class T, int N>
vec<T, N>& operator*=(vec<T, N>& u, vec<T, N> const& v)
{
    return u = u * v;
}

template <class T, int N>
vec<T, N>& operator/=(vec<T, N>& u, vec<T, N> const& v)
{
    return u = u / v;
}

template <class T, int N>
vec<T, N>& operator+=(vec<T, N>& u, Id<T> s)
{
    return u = u + s;
}

template <class T, int N>
vec<T, N>& operator-=(vec<T, N>& u, Id<T> s)
{
    return u = u - s;
}

template <class T, int N>
vec<T, N>& operator*=(vec<T, N>& u, Id<T> s)
{
    return u = u * s;
}

template <class T, int N>
vec<T, N>& operator/=(vec<T, N>& u, Id<T> s)
{
    return u = u / s;
}

//--------------------------------------------------------------------------------------------------
// Component-wise operations
//

template <class T, int N>
vec<T, N> abs(vec<T, N> const& u)
{
    vec<T, N> r;

    for (int n = 0; n < N; ++n)
    {
        r[n] = abs(u[n]);
    }

    return r;
}

//--------------------------------------------------------------------------------------------------
// Geometric functions
//

// Returns the dot product of the normalized vecs u and v.
template <class T, int N>
T normalized_dot(vec<T, N> const& u, vec<T, N> const& v)
{
    return dot(u, v) * rsqrt(dot(u, u) * dot(v, v));
}

// Returns the squared length of the given vec
template <class T, int N>
T length_squared(vec<T, N> const& u)
{
    return dot(u, u);
}

// Returns the length of the given vec
template <class T, int N>
T length(vec<T, N> const& u)
{
    return sqrt(length_squared(u));
}

// Returns the inverse of the length of the given vec
template <class T, int N>
T rlength(vec<T, N> const& u)
{
    return rsqrt(length_squared(u));
}

// Computes the distance between the two vecs
template <class T, int N>
T distance(vec<T, N> const& u, vec<T, N> const& v)
{
    return length(u - v);
}

// Returns a normalized version of the given vec
template <class T, int N>
vec<T, N> normalize(vec<T, N> const& u)
{
    return u * rlength(u);
}

// Returns the angle between u and v
// NOTE: Assumes u and v are normalized!
template <class T, int N>
T angle(vec<T, N> const& u, vec<T, N> const& v)
{
//  return acos(dot(u, v));
    return acos(clamp(dot(u, v), static_cast<T>(-1), static_cast<T>(1)));
}

//--------------------------------------------------------------------------------------------------
// Relational functions
//

template <class B, class T, int N>
vec<T, N> select(vec<B, N> const& condition, vec<T, N> const& true_, vec<T, N> const& false_)
{
    vec<T, N> r;

    for (int n = 0; n < N; ++n)
        r[n] = select(condition[n], true_[n], false_[n]);

    return r;
}

template <class B, int N>
bool any(vec<B, N> const& mask)
{
    for (int n = 0; n < N; ++n)
        if (any(mask[n]))
            return true;

    return false;
}

template <class B, int N>
bool all(vec<B, N> const& mask)
{
    for (int n = 0; n < N; ++n)
        if (!all(mask[n]))
            return false;

    return true;
}

#define LIB_MATH_VECCMP(F)                                                                          \
    template <class T, int N>                                                                       \
    auto F(vec<T, N> const& lhs, vec<T, N> const& rhs) -> vec< decltype(F(lhs[0], rhs[0])), N >     \
    {                                                                                               \
        vec< decltype(F(lhs[0], rhs[0])), N > r;                                                    \
        for (int n = 0; n < N; ++n)                                                                 \
            r[n] = F(lhs[n], rhs[n]);                                                               \
        return r;                                                                                   \
    }                                                                                               \
    /**/

LIB_MATH_VECCMP(cmpeq)
LIB_MATH_VECCMP(cmpne)
LIB_MATH_VECCMP(cmplt)
LIB_MATH_VECCMP(cmple)
LIB_MATH_VECCMP(cmpgt)
LIB_MATH_VECCMP(cmpge)

#undef LIB_MATH_VECCMP

} // namespace math
