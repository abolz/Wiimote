// This file is distributed under the MIT license.
// See the LICENSE file for details.

#pragma once

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------

#define WII_LOG_0           0
#define WII_LOG_1           1

#define WII_LOG_STATUS      1
#define WII_LOG_READ        0
#define WII_LOG_WRITE       1
#define WII_LOG_INIT        1
#define WII_LOG_MP          1 // motion-plus
#define WII_LOG_IO          1

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------

namespace wii { namespace details {

// Prevent:
// C4127: conditional expression is constant
inline bool false_() { return false; }

}}

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------

#define WII_CONCAT_(X, Y) \
    X ## Y

#define WII_CONCAT(X, Y) \
    WII_CONCAT_(X, Y)

#define WII_INVOKE(X, ...) \
    X(__VA_ARGS__)

#define WII_DLOG_0(...) \
    do { } while (::wii::details::false_())

#define WII_DLOG_1(...) \
    do { printf(__VA_ARGS__); } while (::wii::details::false_())

#define WII_LOG(X, ...) \
    WII_INVOKE(WII_CONCAT(WII_DLOG_, WII_LOG_##X), __VA_ARGS__)
