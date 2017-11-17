// This file is distributed under the MIT license.
// See the LICENSE file for details.

#pragma once

#ifndef LIB_COMPILER_H
#define LIB_COMPILER_H 1

// NOTE:
//
// No pragma once here! This file is intended to be easily copyable to different
// libraries. Always use LIB_COMPILER_H as the include guard!

#if defined(__GNUC__)
#   define CXX_GCC           (__GNUC__ * 100 + __GNUC_MINOR__)
#elif defined(__clang__)
#   define CXX_CLANG         (__clang_major__ * 100 + __clang_minor__)
#elif defined(_MSC_VER)
#   define CXX_MSVC          (_MSC_VER)
#endif

#if CXX_GCC || CXX_CLANG
#   define CXX_GCCLIKE 1
#endif

#if defined(__CUDACC__)
#   define CXX_CUDA 1
#   if defined(__CUDA_ARCH__)
#       define CXX_CUDA_DEVICE 1
#   endif
#endif

#if CXX_MSVC
#   define CXX_ALIGN_AS(N) __declspec(align(N))
#else
#   define CXX_ALIGN_AS(N) __attribute__ ((__aligned__(N))) // alignas(N)
#endif

#if CXX_MSVC
#   define CXX_NO_INLINE __declspec(noinline)
#else
#   define CXX_NO_INLINE __attribute__ ((noinline))
#endif

#if CXX_MSVC
// 4201: nonstandard extension used : nameless struct/union
#   define CXX_EXTENSION __pragma(warning(suppress : 4201))
#else
#   define CXX_EXTENSION __extension__
#endif

#endif
