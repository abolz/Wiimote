// This file is distributed under the MIT license.
// See the LICENSE file for details.

#pragma once

#define LIB_MATH_BBOX_H 1

#include "lib/Math/mat.h"

#include <array>
#include <limits>

namespace math
{

//--------------------------------------------------------------------------------------------------
// bbox
// Axis-Aligned Bounding Box
//

template <class T, int N>
struct bbox
{
    using scalar_type = T;
    using vector_type = vec<T, N>;

    vector_type min;
    vector_type max;

    // Default constructor
    bbox() = default;

    // Construct from minimum and maximum edge points
    bbox(vector_type const& min_, vector_type const& max_)
        : min(min_)
        , max(max_)
    {
    }

    // Construct from a single point
    explicit bbox(vector_type const& p)
        : min(p)
        , max(p)
    {
    }

    // Invalidates the box edges (sets min > max)
    void invalidate()
    {
        min = vector_type(std::numeric_limits<scalar_type>::max());
        max = vector_type(std::numeric_limits<scalar_type>::lowest());
    }

    // Sets the bounding box to hold a single point
    void clear(vector_type const& point)
    {
        min = point;
        max = point;
    }

    // Computes the center of the bounding box
    vector_type center() const
    {
        return scalar_type(0.5) * (max + min);
    }

    // Computes the size of the bounding box
    vector_type size() const
    {
        return max - min;
    }

    // Computes the volume of the bounding box
    scalar_type volume() const
    {
        return hmul(size());
    }

    // Computes the area of the bounding box
    scalar_type area() const
    {
        auto s = size();
        return 2.0f * (s.x * s.y + s.y * s.z + s.z * s.x);
    }
};

//--------------------------------------------------------------------------------------------------
// Geometric functions
//

// Computes the intersection of two bounding boxes
template <class T, int N>
bbox<T, N> intersect(bbox<T, N> const& lhs, bbox<T, N> const& rhs)
{
    return { max(lhs.min, rhs.min), min(lhs.max, rhs.max) };
}

// Computes the union of two bounding boxes
template <class T, int N>
bbox<T, N> combine(bbox<T, N> const& lhs, bbox<T, N> const& rhs)
{
    return { min(lhs.min, rhs.min), max(lhs.max, rhs.max) };
}

// Computes the union of a bounding box and a point
template <class T, int N>
bbox<T, N> combine(bbox<T, N> const& lhs, Vector_type<bbox<T, N>> const& rhs)
{
    return { min(lhs.min, rhs), max(lhs.max, rhs) };
}

// Computes the union of a bounding box and a point
template <class T, int N>
bbox<T, N> combine(Vector_type<bbox<T, N>> const& lhs, bbox<T, N> const& rhs)
{
    return combine(rhs, lhs);
}

// Computes the vertices of a bounding box
template <class T>
std::array<vec<T, 2>, 4> compute_vertices(bbox<T, 2> const& box)
{
    using V = vec<T, 2>;

    auto minx = box.min[0];
    auto miny = box.min[1];
    auto maxx = box.max[0];
    auto maxy = box.max[1];

    //
    // 0 = min
    // 2 = max
    //              3 ---- 2
    //     y        |      |
    //     |        |      |
    //     +-- x    0 ---- 1
    //
    return { V(minx, miny),
             V(maxx, miny),
             V(maxx, maxy),
             V(minx, maxy) };
}

// Computes the vertices of a bounding box
template <class T>
std::array<vec<T, 3>, 8> compute_vertices(bbox<T, 3> const& box)
{
    using V = vec<T, 3>;

    auto minx = box.min[0];
    auto miny = box.min[1];
    auto minz = box.min[2];
    auto maxx = box.max[0];
    auto maxy = box.max[1];
    auto maxz = box.max[2];

    //
    // 0 = min
    // 6 = max
    //                 3 ---- 2
    //     y          /|    / |
    //     |        7 ---- 6  |
    //     +-- x    |  0 --|- 1
    //    /         | /    | /
    //   z          4 ---- 5
    //
    return { V(minx, miny, minz),
             V(maxx, miny, minz),
             V(maxx, maxy, minz),
             V(minx, maxy, minz),
             V(minx, miny, maxz),
             V(maxx, miny, maxz),
             V(maxx, maxy, maxz),
             V(minx, maxy, maxz) };
}

// Transform a bounding box by an affine transformation (only the upper 3x4 matrix is used)
inline aabb xform(mat4 const& m, aabb const& box)
{
    aabb result(m(3).xyz());

    for (size_t r = 0; r < 3; ++r)
    {
        for (size_t c = 0; c < 3; ++c)
        {
            auto x = m(r,c) * box.min[c];
            auto y = m(r,c) * box.max[c];

            result.min[r] += min(x, y);
            result.max[r] += max(x, y);
        }
    }

    return result;
}

//--------------------------------------------------------------------------------------------------
// Operators
//

template <class T, int N>
bbox<T, N> operator+(bbox<T, N> const& box, Vector_type<bbox<T, N>> const& d)
{
    return { box.min + d, box.max + d };
}

template <class T, int N>
bbox<T, N> operator-(bbox<T, N> const& box, Vector_type<bbox<T, N>> const& d)
{
    return { box.min - d, box.max - d };
}

template <class T, int N>
bbox<T, N> operator*(bbox<T, N> const& box, Vector_type<bbox<T, N>> const& d)
{
    return { box.min * d, box.max * d };
}

template <class T, int N>
bbox<T, N> operator/(bbox<T, N> const& box, Vector_type<bbox<T, N>> const& d)
{
    return { box.min / d, box.max / d };
}

template <class T, int N>
bbox<T, N> operator+(Vector_type<bbox<T, N>> const& d, bbox<T, N> const& box)
{
    return { d + box.min, d + box.max };
}

template <class T, int N>
bbox<T, N> operator-(Vector_type<bbox<T, N>> const& d, bbox<T, N> const& box)
{
    return { d - box.min, d - box.max };
}

template <class T, int N>
bbox<T, N> operator*(Vector_type<bbox<T, N>> const& d, bbox<T, N> const& box)
{
    return { d * box.min, d * box.max };
}

template <class T, int N>
bbox<T, N>& operator+=(bbox<T, N>& r, Vector_type<bbox<T, N>> const& d)
{
    return r = r + d;
}

template <class T, int N>
bbox<T, N>& operator-=(bbox<T, N>& r, Vector_type<bbox<T, N>> const& d)
{
    return r = r - d;
}

template <class T, int N>
bbox<T, N>& operator*=(bbox<T, N>& r, Vector_type<bbox<T, N>> const& s)
{
    return r = r * s;
}

template <class T, int N>
bbox<T, N>& operator/=(bbox<T, N>& r, Vector_type<bbox<T, N>> const& s)
{
    return r = r / s;
}

} // namespace math
