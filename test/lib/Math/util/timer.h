// This file is distributed under the MIT license.
// See the LICENSE file for details.

#pragma once

#define LIB_MATH_UTIL_TIMER_H 1

#include <cmath>
#include <chrono>

namespace math
{

//--------------------------------------------------------------------------------------------------
// Timer
//

class Timer
{
    using clock_type    = std::chrono::high_resolution_clock;
    using duration      = clock_type::duration;
    using time_point    = clock_type::time_point;

    // Last time reset() was called
    time_point start_;
    // Last time lap() was called
    time_point last_lap_;

public:
    // Constructor.
    Timer()
        : start_(clock_type::now())
        , last_lap_(start_)
    {
    }

    // Resets the timer
    void reset()
    {
        start_ = clock_type::now();
        last_lap_ = start_;
    }

    // Returns the elapsed time in seconds
    double elapsed() const
    {
        return ToSeconds(clock_type::now() - start_);
    }

    // Returns the lap time in seconds
    double lap()
    {
        time_point now = clock_type::now();

        duration d = now - last_lap_;

        last_lap_ = now;

        return ToSeconds(d);
    }

private:
    static double ToSeconds(duration d) {
        return std::chrono::duration<double>(d).count();
    }
};

//--------------------------------------------------------------------------------------------------
// FrameCounter
//

class FrameCounter
{
    using clock_type    = std::chrono::high_resolution_clock;
    using duration      = clock_type::duration;
    using time_point    = clock_type::time_point;

    // Last time numFrames_ was 0
    time_point start_;
    // Last time update() was called
    time_point last_;
    // Number of elapsed frames
    unsigned numFrames_;
    // Current fps estimate
    double fps_;
    // Update interval
    double updateInterval_;

public:
    // Constructor.
    FrameCounter(double updateInterval = 0.5)
        : start_(clock_type::now())
        , last_(start_)
        , numFrames_(0)
        , fps_(0.0)
        , updateInterval_(updateInterval)
    {
    }

    // Register a frame
    // Returns the frame-delta, ie. the time in seconds since the last call to update()
    double update()
    {
        // Increase frame counter
        numFrames_++;

        time_point now = clock_type::now();

        double elapsed = ToSeconds(now - last_);

        last_ = now;

        double dt = ToSeconds(now - start_);

        if (dt >= updateInterval_ /* && numFrames_ >= 5 */)
        {
            fps_ = numFrames_ / dt;
            numFrames_ = 0;
            start_ = now;
        }

        return elapsed;
    }

    // Returns current fps
    double getFPS() const
    {
        return fps_;
    }

private:
    static double ToSeconds(duration d) {
        return std::chrono::duration<double>(d).count();
    }
};

//--------------------------------------------------------------------------------------------------
// Timesteps
//

class Timesteps
{
    using clock_type    = std::chrono::high_resolution_clock;
    using duration      = clock_type::duration;
    using time_point    = clock_type::time_point;

    // Frame delta (in seconds)
    double delta_;
    // Accumulated frame delta (in seconds)
    double acc_;
    // Last update time
    time_point last_;

public:
    explicit Timesteps(double delta = 1.0/60.0)
        : delta_(delta)
        , acc_(0.0)
        , last_(clock_type::now())
    {
    }

    // Restart the timer
    void restart()
    {
        acc_ = 0.0;
        last_ = clock_type::now();
    }

    // Returns the frame delta
    double delta() const {
        return delta_;
    }

    // Returns the accumulated frame delta
    double acc() const {
        return acc_;
    }

    // Accumulate frame delta
    void update()
    {
        time_point now = clock_type::now();

        // Accumulate frame delta
        double s = acc_ + ToSeconds(now - last_);

        if (s >= delta_)
        {
            acc_ = s;
            last_ = now;
        }
    }

    // Returns whether a single timestep is available
    bool empty()
    {
        return acc_ < delta_;
    }

    // Consume some timesteps
    int consume()
    {
        if (acc_ >= delta_)
        {
            double n = 0;
            acc_ = delta_ * std::modf(acc_ / delta_, &n);

            return static_cast<int>(n);
        }

        return 0;
    }

private:
    static double ToSeconds(duration d) {
        return std::chrono::duration<double>(d).count();
    }
};

} // namespace math
