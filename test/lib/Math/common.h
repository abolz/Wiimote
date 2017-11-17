// This file is distributed under the MIT license.
// See the LICENSE file for details.

#pragma once

#define LIB_MATH_COMMON_H 1

#include "lib/Math/compiler.h"
#include "lib/Math/config.h"

#include <cassert>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility> // pair, swap

#ifdef min
#pragma message("NOTE: macro min is incompatible with C++.  #undefing min")
#undef min
#endif

#ifdef max
#pragma message("NOTE: macro max is incompatible with C++.  #undefing max")
#undef max
#endif

namespace math
{

//==================================================================================================
//
//==================================================================================================

//--------------------------------------------------------------------------------------------------
// Type traits
//

template <class T> struct identity_type { using type = T; };

template <class T>
using Id = typename identity_type<T>::type;

template <class T>
using Scalar_type = typename T::scalar_type;

template <class T>
using Vector_type = typename T::vector_type;

template <int Bits> struct sized_int_type;
template <> struct sized_int_type< 8> { using type = int8_t;  };
template <> struct sized_int_type<16> { using type = int16_t; };
template <> struct sized_int_type<32> { using type = int32_t; };
template <> struct sized_int_type<64> { using type = int64_t; };

#if 1

// WORKAROUND:
// msvc ice when using an alias template with more than one parameter...

template <int Bits>
using Sized_int = typename sized_int_type<Bits>::type;

template <int Bits>
using Sized_uint = typename std::make_unsigned<Sized_int<Bits>>::type;

template <int I1, int I2>
struct Max_value { static const int value = I1 <= I2 ? I2 : I1; };

#define MATH_SIZED_UINT_T(T, MIN) \
    Sized_uint<Max_value<8 * sizeof(T), MIN>::value>

#else

template <int Bits, int Min = 8>
using Sized_int = typename sized_int_type<((Bits <= Min) ? Min : Bits)>::type;

template <int Bits, int Min = 8>
using Sized_uint = typename std::make_unsigned<Sized_int<Bits, Min>>::type;

template <class Int, int Min = 8>
using Sized_int_t = Sized_int<CHAR_BIT * sizeof(Int), Min>;

template <class Int, int Min = 8>
using Sized_uint_t = Sized_uint<CHAR_BIT * sizeof(Int), Min>;

#endif

//--------------------------------------------------------------------------------------------------
// Declarations
//

template <class T, int N/*Columns*/>
struct vec;

struct quat;

struct mat4;

struct mat3;

template <class T, int N/*Dim*/>
struct bbox;

using vec2b     = vec<bool, 2>;
using vec3b     = vec<bool, 3>;
using vec4b     = vec<bool, 4>;
using vec2uc    = vec<unsigned char, 2>;
using vec3uc    = vec<unsigned char, 3>;
using vec4uc    = vec<unsigned char, 4>;
using vec2i     = vec<int, 2>;
using vec3i     = vec<int, 3>;
using vec4i     = vec<int, 4>;
using vec2u     = vec<unsigned, 2>;
using vec3u     = vec<unsigned, 3>;
using vec4u     = vec<unsigned, 4>;
using vec2      = vec<float, 2>;
using vec3      = vec<float, 3>;
using vec4      = vec<float, 4>;
using vec2d     = vec<double, 2>;
using vec3d     = vec<double, 3>;
using vec4d     = vec<double, 4>;

using rect      = bbox<float, 2>;
using aabb      = bbox<float, 3>;

//--------------------------------------------------------------------------------------------------
// Type traits
//

template <class T> struct aligned_base {};

#if !LIB_MATH_NO_ALIGNED_VECTOR_TYPES

#if CXX_MSVC
#pragma warning(push)
#pragma warning(disable : 4324) // structure was padded due to __declspec(align()))
#endif

template <> struct CXX_ALIGN_AS( 2) aligned_base< vec<unsigned char, 2> > {};
template <> struct CXX_ALIGN_AS( 4) aligned_base< vec<unsigned char, 4> > {};
template <> struct CXX_ALIGN_AS( 8) aligned_base< vec<int,           2> > {};
template <> struct CXX_ALIGN_AS(16) aligned_base< vec<int,           4> > {};
template <> struct CXX_ALIGN_AS( 8) aligned_base< vec<unsigned,      2> > {};
template <> struct CXX_ALIGN_AS(16) aligned_base< vec<unsigned,      4> > {};
template <> struct CXX_ALIGN_AS( 8) aligned_base< vec<float,         2> > {};
template <> struct CXX_ALIGN_AS(16) aligned_base< vec<float,         4> > {};
template <> struct CXX_ALIGN_AS(16) aligned_base< vec<double,        2> > {};
template <> struct CXX_ALIGN_AS(32) aligned_base< vec<double,        4> > {};
template <> struct CXX_ALIGN_AS(16) aligned_base< quat                  > {};
template <> struct CXX_ALIGN_AS(64) aligned_base< mat4                  > {};

#if CXX_MSVC
#pragma warning(pop)
#endif

#endif // !LIB_MATH_NO_ALIGNED_VECTOR_TYPES

//==================================================================================================
//
//==================================================================================================

//--------------------------------------------------------------------------------------------------
// Constants
//

namespace constants
{

#define LIB_MATH_CONST(NAME, VALUE) \
    const struct NAME##_t { \
        template <class T> explicit operator T() const { return T(VALUE); } \
    } \
    NAME

LIB_MATH_CONST(degress_to_radians,   1.74532925199432957692369076849e-02);
LIB_MATH_CONST(radians_to_degrees,   5.72957795130823208767981548141e+01);
LIB_MATH_CONST(zero,                 0.0);
LIB_MATH_CONST(half,                 0.5);
LIB_MATH_CONST(one,                  1.0);
LIB_MATH_CONST(pi,                   3.14159265358979323846264338328e+00);
LIB_MATH_CONST(inv_pi,               3.18309886183790671537767526745e-01);
LIB_MATH_CONST(pi_half,              1.57079632679489661923132169164e+00);
LIB_MATH_CONST(two_pi,               6.28318530717958647692528676656e+00);
LIB_MATH_CONST(sqrt_pi,              1.77245385090551602729816748334e+00);
LIB_MATH_CONST(sqrt_two_pi,          2.50662827463100050241576528481e+00);
LIB_MATH_CONST(inv_sqrt_two_pi,      3.98942280401432677939946059934e-01);
LIB_MATH_CONST(ln_2,                 6.93147180559945309417232121458e-01);
LIB_MATH_CONST(ln_10,                2.30258509299404568401799145468e+00);
LIB_MATH_CONST(sqrt_two,             1.41421356237309504880168872421e+00);
LIB_MATH_CONST(inv_sqrt_two,         7.07106781186547524400844362105e-01);
LIB_MATH_CONST(min,                  std::numeric_limits<T>::lowest());
LIB_MATH_CONST(max,                  std::numeric_limits<T>::max());
LIB_MATH_CONST(neg_inf,             -std::numeric_limits<T>::infinity());
LIB_MATH_CONST(pos_inf,              std::numeric_limits<T>::infinity());
LIB_MATH_CONST(nan,                  std::numeric_limits<T>::quiet_NaN());
LIB_MATH_CONST(epsilon,              std::numeric_limits<T>::epsilon());

#undef LIB_MATH_CONST

} // namespace constants

//--------------------------------------------------------------------------------------------------
// Import required math functions from the standard library.
// Implement missing...
//

using std::abs;
using std::acos;
using std::asin;
using std::atan2;
using std::atan;
using std::ceil;
using std::copysign;
using std::cos;
using std::exp;
using std::exp2;
using std::floor;
using std::log;
using std::log2;
using std::lrint;
using std::lround;
using std::pow;
using std::rint;
using std::round;
using std::sin;
using std::sqrt;
using std::tan;
using std::trunc;

inline float frac(float x)
{
    return x - ::floorf(x);
}

inline double frac(double x)
{
    return x - ::floor(x);
}

// Map fmin/fmax to min/max

inline float min(float x, float y)
{
    return ::fminf(x, y);
}

inline double min(double x, double y)
{
    return ::fmin(x, y);
}

inline float max(float x, float y)
{
    return ::fmaxf(x, y);
}

inline double max(double x, double y)
{
    return ::fmax(x, y);
}

template <class T>
inline T min(T const& x, T const& y)
{
    return y < x ? y : x;
}

template <class T>
inline T max(T const& x, T const& y)
{
    return y < x ? x : y;
}

template <class T, class... Ts>
inline T min(T const& x, T const& y, Ts const&... z)
{
    return min(min(x, y), z...);
}

template <class T, class... Ts>
inline T max(T const& x, T const& y, Ts const&... z)
{
    return max(max(x, y), z...);
}

template <class T>
inline T clamp(T const& x, T const& a, T const& b)
{
    return max(a, min(x, b));
}

template <class T>
inline T saturate(T const& x)
{
    return max(T(0.0), min(x, T(1.0)));
}

template <class T>
inline T fast_rcp(T const& x)
{
    return T(1.0) / x;
}

template <class T>
inline T rcp(T const& x)
{
    return T(1.0) / x;
}

template <class T>
inline T fast_rsqrt(T const& x)
{
    return T(1.0) / sqrt(x);
}

template <class T>
inline T rsqrt(T const& x)
{
    return T(1.0) / sqrt(x);
}

template <class T, class S>
inline T lerp(T const& a, T const& b, S const& t)
{
    return (S(1.0) - t) * a + t * b;
}

template <class T, class S>
inline T bilerp(T const& a1, T const& b1, T const& a2, T const& b2, S const& t1, S const& t2)
{
    return lerp( lerp(a1, b1, t1), lerp(a2, b2, t1), t2 );
}

template <class T>
inline T linearstep(T const& a, T const& b, T const& t)
{
    return saturate((t - a) / (b - a));
}

template <class T>
inline T smoothstep(T const& a, T const& b, T const& t)
{
    auto y = linearstep(a, b, t);

    return y * y * (T(3.0) - T(2.0) * y);
}

// Perlin's C2 continous variation on smoothstep()
//template <class T>
//inline T smootherstep(T const& a, T const& b, T const& t)
//{
//    auto y = linearstep(a, b, t);
//
//    return y * y * y * (y * (y * T(6.0) - T(15.0)) + T(10.0));
//}

// Integer square root. Returns floor(sqrt(N))
template <class Int>
inline Int isqrt(Int n)
{
    if (n <= 0)
        return 0;

    Int m = n;
    Int k = 1;
    do
    {
        m = (m + k) / 2;
        k = n / m;
    }
    while (m > k);

    return m;
}

template <class Int>
inline Int gcd(Int a, Int b)
{
    while (b != 0)
    {
        Int t = a % b;
        a = b;
        b = t;
    }

    return a;
}

//--------------------------------------------------------------------------------------------------
// Some additional useful methods
//

template <class T>
inline T to_radians(T const& x)
{
    return x * T(constants::degress_to_radians);
}

template <class T>
inline T to_degrees(T const& x)
{
    return x * T(constants::radians_to_degrees);
}

template <class T>
inline T det2(T const& m00, T const& m01, T const& m10, T const& m11)
{
    return m00 * m11 - m10 * m01;
}

//inline Int wrap_clamp(Int x, Int len)
//{
//    return clamp(x, 0, len - 1);
//}

//inline Int wrap_repeat(Int x, Int len)
//{
//    if (x >= 0)
//        return x % len;
//
//    return ((x + 1) % len) + (len - 1);
//}

//inline Int wrap_mirrored_repeat(Int x, Int len)
//{
//    Int A = 2*len - 2;
//
//    if (x < 0)
//        x = -x;
//
//    if (x >= A)
//        x = x - A * (x / A);
//
//    if (x >= len)
//        x = A - x;
//
//    return x;
//}

//--------------------------------------------------------------------------------------------------
// Relational functions
//

template <class T>
T select(bool condition, T const& true_, T const& false_)
{
    return condition ? true_ : false_;
}

inline bool any(bool mask)
{
    return mask;
}

inline bool all(bool mask)
{
    return mask;
}

template <class T> bool cmpeq(T const& lhs, T const& rhs) { return lhs == rhs; }
template <class T> bool cmpne(T const& lhs, T const& rhs) { return lhs != rhs; }
template <class T> bool cmplt(T const& lhs, T const& rhs) { return lhs <  rhs; }
template <class T> bool cmple(T const& lhs, T const& rhs) { return lhs <= rhs; }
template <class T> bool cmpgt(T const& lhs, T const& rhs) { return lhs >  rhs; }
template <class T> bool cmpge(T const& lhs, T const& rhs) { return lhs >= rhs; }

} // namespace math
