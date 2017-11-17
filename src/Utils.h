// This file is distributed under the MIT license.
// See the LICENSE file for details.

#pragma once

#include <cassert>
#include <cstdint>
#include <cstring>

namespace wii
{

template <class T, class U>
inline T BitCast(U const& u)
{
    T t;
    std::memcpy(&t, &u, sizeof(T));
    return t;
}

// Mask out the bits which are 0 in prev and 1 in curr
inline unsigned RecentlySet(unsigned prev, unsigned curr)
{
    return curr & ~(prev & curr);
}

// Mask out the bits which are 1 in prev and 0 in curr
inline unsigned RecentlyCleared(unsigned prev, unsigned curr)
{
    return prev & ~(prev & curr);
}

inline uint8_t B0(unsigned n)
{
    return static_cast<uint8_t>((n >>  0) & 0xFF);
}

inline uint8_t B1(unsigned n)
{
    return static_cast<uint8_t>((n >>  8) & 0xFF);
}

inline uint8_t B2(unsigned n)
{
    return static_cast<uint8_t>((n >> 16) & 0xFF);
}

inline uint8_t B3(unsigned n)
{
    return static_cast<uint8_t>((n >> 24) & 0xFF);
}

inline unsigned Read8(uint8_t const* p)
{
    return p[0];
}

inline unsigned Read16(uint8_t const* p)
{
    return p[0] << 8 | p[1];
}

inline unsigned Read32(uint8_t const* p)
{
    return p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3];
}

inline void Write8(uint8_t* p, unsigned n)
{
    p[0] = B0(n);
}

inline void Write16(uint8_t* p, unsigned n)
{
    p[0] = B1(n);
    p[1] = B0(n);
}

inline void Write32(uint8_t* p, unsigned n)
{
    p[0] = B3(n);
    p[1] = B2(n);
    p[2] = B1(n);
    p[3] = B0(n);
}

} // namespace wii
