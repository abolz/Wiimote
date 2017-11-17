// This file is distributed under the MIT license.
// See the LICENSE file for details.

#pragma once

#define LIB_MATH_UTIL_FILTER_H 1

#include "lib/Math/vec.h"

namespace math
{

//--------------------------------------------------------------------------------------------------
// LowPassFilter
//

class LowPassFilter
{
    // The current value
    vec3 value_;
    // Filter constant
    float RC_;

public:
    explicit LowPassFilter(float RC = 0.2f)
        : value_(0.0f)
        , RC_(RC)
    {
    }

    // Returns the current value
    vec3 const& getValue() const
    {
        return value_;
    }

    // Returns the filter constant
    float getFilterConstant() const
    {
        return RC_;
    }

    // Reset the filter
    void restart(vec3 const& init = vec3(0.0f))
    {
        value_ = init;
    }

    // Sets the filter constant
    void setFilterConstant(float RC)
    {
        RC_ = RC;
    }

    // Adds a sample.
    // Returns the new value.
    vec3 const& add(vec3 const& sample, float dt)
    {
        auto alpha = dt / (dt + RC_);

        value_ = lerp(value_, sample, alpha);

        return value_;
    }
};

//--------------------------------------------------------------------------------------------------
// AdaptiveLowPassFilter
//

class AdaptiveLowPassFilter
{
    // The current value
    vec3 value_;
    // Filter constant
    float RC_;
    // Filter configuration
    float minStep_;
    // Filter configuration
    float noiseAttenuation_;

public:
    explicit AdaptiveLowPassFilter(float RC = 0.2f, float minStep = 0.02f, float noiseAttenuation = 3.0f)
        : value_(0.0f)
        , RC_(RC)
        , minStep_(minStep)
        , noiseAttenuation_(noiseAttenuation)
    {
    }

    // Returns the current value
    vec3 const& getValue() const
    {
        return value_;
    }

    // Returns the filter constant
    float getFilterConstant() const
    {
        return RC_;
    }

    // Reset the filter
    void restart(vec3 const& init = vec3(0.0f))
    {
        value_ = init;
    }

    // Sets the filter constant
    void setFilterConstant(float RC)
    {
        RC_ = RC;
    }

    // Adds a sample.
    // Returns the new value.
    vec3 const& add(vec3 const& sample, float dt)
    {
        float alpha = dt / (dt + RC_);

        float d = saturate( abs(length(value_) - length(sample)) / minStep_ - 1.0f );

        alpha = (1.0f - d) * alpha / noiseAttenuation_ + d * alpha;

        value_ = lerp(value_, sample, alpha);

        return value_;
    }
};

} // namespace math
