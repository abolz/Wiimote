// This file is distributed under the MIT license.
// See the LICENSE file for details.

#pragma once

#include "lib/Math/math.h"
#include "lib/Math/util/filter.h"

class Track
{
public:
    struct State
    {
        // Current wiimote position (relative to something)
        math::vec3 position;
        // Current velocity
        math::vec3 velocity;
        // Current acceleration
        math::vec3 acceleration;
        // Current angular velocity (normalized)
        math::vec3 omega;
        // Orientation estimate based on motion-plus
        math::quat rotGyro;
        // Orientation estimate based on accelerometer (= rotX * rotZ)
        math::quat rotAccel;
        // Rotation around the x-axis (based on accelerometer readings)
        math::quat rotX;
        // Rotation around the z-axis (based on accelerometer readings)
        math::quat rotZ;
    };

private:
    // Current state
    State state_;
    // Low-pass filter for gyro bias
    math::LowPassFilter biasGyroFilter;
    // Gyro bias
    math::vec3 biasGyro;
    // Low-pass filter for raw accelerometer values
    math::AdaptiveLowPassFilter accelFilter;

public:
    Track();

    // Returns the current tracking state
    State const& state() const {
        return state_;
    }

    // Reset.
    void reset();

    // Reset gyro estimate based on accelerometer estimate
    void home();

    // Calibrate
    void calibrateAccel(math::vec3 const& a, float dt);

    // Calibrate
    void calibrateGyro(math::vec3 const& w, float dt);

    // Update bias
    void init();

    // Handle wiimote accelerometer values
    void handleAccel(math::vec3 a, float dt);

    // Handle motion-plus gyro values
    // Returns false while calibrating, true otherwise
    bool handleGyros(math::vec3 w, math::vec3b fast, float dt);
};
