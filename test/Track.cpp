// This file is distributed under the MIT license.
// See the LICENSE file for details.

#include "Track.h"

using namespace math;

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------

template <class T>
static T map(T const& v)
{
    return T(-v.x, v.z, v.y);
}

static vec3b map(vec3b const& v)
{
    return vec3b(v.x, v.z, v.y);
}

static vec3 NormalizeGyros(vec3 const& w, vec3b const& fast, vec3 const& bias)
{
    // slow mode: 20 units = 1 deg/sec
    const vec3 ScaleSlow(1.0f/20.0f);
    // fast mode: 20 units = 4.54 deg/sec
//    const vec3 ScaleFast(2.27f/0.5f * ScaleSlow);
    const vec3 ScaleFast(4.54f/20.0f);

    // Minimum and maximum raw gyro values in slow and fast mode, resp.
    //
    // TODO:
    // Motion+ calibration data... ?!?!?!?!
    //
//    const vec3 MinSlow(  560.0f,   560.0f,   560.0f);
//    const vec3 MaxSlow(15807.0f, 15808.0f, 15808.0f);
//    const vec3 MinFast( 6438.0f,  6504.0f,  6532.0f);
//    const vec3 MaxFast( 9774.0f,  9749.0f,  9881.0f);
    const vec3 MinSlow(  560.0f);
    const vec3 MaxSlow(15800.0f);
    const vec3 MinFast( 6500.0f);
    const vec3 MaxFast( 9780.0f);

    const vec3 Alpha = (MaxSlow - MinSlow) / (MaxFast - MinFast);

//    vec3 BFast = (bias + 0.5f * (Alpha * (MinFast + MaxFast) - (MinSlow + MaxSlow))) / Alpha;
    vec3 BFast = MinFast + (bias - MinSlow) / Alpha;

    vec3 B = select(fast, BFast, bias);
    vec3 S = select(fast, ScaleFast, ScaleSlow);

    vec3 v = S * (w - B);

//    printf("GYRO: [%d %d %d] % 9.4f % 9.4f % 9.4f\n", (int)fast.x, (int)fast.y, (int)fast.z, v.x, v.y, v.z);
//    printf("GYRO: [%d %d %d] % 9.4f % 9.4f % 9.4f\n", (int)fast.x, (int)fast.y, (int)fast.z, w.x, w.y, w.z);

    return v;
}

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------

Track::Track()
{
    reset();
}

void Track::reset()
{
    state_.position     = vec3(0.0f);
    state_.velocity     = vec3(0.0f);
    state_.acceleration = vec3(0.0f);
    state_.omega        = vec3(0.0f);
    state_.rotGyro      = quat::identity();
    state_.rotAccel     = quat::identity();
    state_.rotX         = quat::identity();
    state_.rotZ         = quat::identity();

    // Reset gyroscope filter
    biasGyroFilter.setFilterConstant(0.5f);
    biasGyroFilter.restart();

    // Reset accelerometer filter
    accelFilter.setFilterConstant(0.05f);
    accelFilter.restart();
}

void Track::home()
{
    state_.rotGyro = state_.rotAccel;
}

void Track::calibrateAccel(vec3 const& a, float dt)
{
    // Filter accelerometer values
    accelFilter.add(a, dt);
}

void Track::calibrateGyro(vec3 const& w, float dt)
{
    // Filter gyroscope values
    biasGyroFilter.add(w, dt);
}

void Track::init()
{
    // Save (current) gyroscope bias
    biasGyro = biasGyroFilter.getValue();

    printf("BIAS: %f %f %f\n", biasGyro.x, biasGyro.y, biasGyro.z);
}

void Track::handleAccel(vec3 a, float dt)
{
    // Filter accelerometer data
    a = accelFilter.add(a, dt);

    // Normalize the accelerometer measurement
    a = normalize(a);

    // Map to OpenGL coordinate system
    a = map(a);

    // Estimate rotation around the x-axis
    auto Ax = atan2(-a.z, a.y);

    // Estimate rotation around the z-axis
    auto Az = atan2( a.x, sqrt(a.y * a.y + a.z * a.z));

    // Compute rotation
    auto R = quat::rotation(vec3(1,0,0), Ax)
           * quat::rotation(vec3(0,0,1), Az);

    state_.rotAccel = normalize(R);
}

#define USE_RUNGE_KUTTA 3 // 3 4
#define USE_QUERGS 1

bool Track::handleGyros(vec3 w, vec3b fast, float dt)
{
    if (dt < std::numeric_limits<float>::epsilon())
        return false;

    static const float Beta = sqrt(3.0f/4.0f) * to_radians(2.0f);

    // Normalize angular rotation data
    w = NormalizeGyros(w, fast, biasGyro);

    // Convert degrees to radians
    w.x = to_radians(w.x);
    w.y = to_radians(w.y);
    w.z = to_radians(w.z);

    // Map to OpenGL coordinate system
    w = map(w);

#if USE_RUNGE_KUTTA
    // Store last angular acceleration.
    // Used in the integration method below.
    auto w_last = state_.omega;
#endif

    // Store normalized angular acceleration
    state_.omega = w;

    // Get accelerometer data
    auto a = accelFilter.getValue();

    // Normalize the accelerometer measurement
    a = normalize(a);

    // Map to OpenGL coordinate system
    a = map(a);

    // Current orientation estimate
    quat q = state_.rotGyro;

    // Compute the objective function and Jacobian
    vec3 f;

    // f = -a + conj(q) * g * q where g = (0,1,0)
    f.x = -a.x + 2.0f * (q.w * q.z + q.x * q.y);
    f.y = -a.y - 2.0f * (q.x * q.x + q.z * q.z) + 1.0f;
    f.z = -a.z + 2.0f * (q.y * q.z - q.w * q.x);

    // Compute the gradient (matrix multiplication)
    quat G;

    // grad = J^T * f
    G.w = 2.0f * q.z * f.x                    - 2.0f * q.x * f.z;
    G.x = 2.0f * q.y * f.x - 4.0f * q.x * f.y - 2.0f * q.w * f.z;
    G.y = 2.0f * q.x * f.x                    + 2.0f * q.z * f.z;
    G.z = 2.0f * q.w * f.x - 4.0f * q.z * f.y + 2.0f * q.y * f.z;

    // Normalize the gradient
    G = normalize(G);

    // Integrate angular velocity
#if USE_RUNGE_KUTTA

    auto f_i = [&](float c_i, quat const& q)
    {
        auto w_i = (1.0f - c_i) * w_last + c_i * w;
        return 0.5f * q * quat(w_i);
    };

#if USE_RUNGE_KUTTA == 3

    auto k1 = f_i( 0.0f, q );
    auto k2 = f_i( 0.5f, q + (0.5f * dt) * k1 );
    auto k3 = f_i( 1.0f, q - (1.0f * dt) * k1 + (2.0f * dt) * k2 );

    auto dq = 1.0f/6.0f * k1 + 2.0f/3.0f * k2 + 1.0f/6.0f * k3;

#elif USE_RUNGE_KUTTA == 4

    auto k1 = f_i( 0.0f, q );
    auto k2 = f_i( 0.5f, q + (0.5f * dt) * k1 );
    auto k3 = f_i( 0.5f, q + (0.5f * dt) * k2 );
    auto k4 = f_i( 1.0f, q + (1.0f * dt) * k3 );

    auto dq = 1.0f/6.0f * k1 + 1.0f/3.0f * k2 + 1.0f/3.0f * k3 + 1.0f/6.0f * k4;

#else
#error invalid Runge-Kutta order
#endif

#else

    auto dq = 0.5f * q * quat(w);

#endif

    // Estimate orientation
#if USE_QUERGS
    q += quergs(dt * (dq - Beta * G));
#else
    q += dt * (dq - Beta * G);
#endif

    // Normalize quaternion
    state_.rotGyro = normalize(q);

    return true;
}
