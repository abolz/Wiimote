// This file is distributed under the MIT license.
// See the LICENSE file for details.

#include "Data.h"
#include "Log.h"
#include "Utils.h"

#include <algorithm>
#include <vector>
#include <fstream>
#include <utility>

using namespace wii;

#define WII_LOG_MP 1
#define WII_LOG_MP_STATISTICS 0

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------

#if WII_LOG_MP_STATISTICS
struct MotionPlusStatistics
{
    MotionPlusData mpPrev;

    Point3i slowMin;
    Point3i slowMax;
    Point3i fastMin;
    Point3i fastMax;

    MotionPlusStatistics()
    {
        mpPrev.time = -1.0;

        slowMin.x = slowMin.y = slowMin.z =  99999;
        slowMax.x = slowMax.y = slowMax.z = -99999;
        fastMin.x = fastMin.y = fastMin.z = -99999;
        fastMax.x = fastMax.y = fastMax.z =  99999;
    }

    void updateFast(int& fastMin, int& fastMax, int prev, int curr, bool currFast)
    {
        if (currFast)
        {
            // slow -> fast
            // prev in slow mode, curr in fast-mode

            if (curr < 8192)
                fastMin = std::max(fastMin, curr);
            else
                fastMax = std::min(fastMax, curr);
        }
        else
        {
            // fast -> slow
            // prev in fast-mode, curr in slow-mode

            if (prev < 8192)
                fastMin = std::max(fastMin, prev);
            else
                fastMax = std::min(fastMax, prev);
        }
    }

    void updateFast(MotionPlusData& prev, MotionPlusData& curr)
    {
        if (prev.fast.x != curr.fast.x)
            updateFast(fastMin.x, fastMax.x, prev.raw.x, curr.raw.x, curr.fast.x != 0);

        if (prev.fast.y != curr.fast.y)
            updateFast(fastMin.y, fastMax.y, prev.raw.y, curr.raw.y, curr.fast.y != 0);

        if (prev.fast.z != curr.fast.z)
            updateFast(fastMin.z, fastMax.z, prev.raw.z, curr.raw.z, curr.fast.z != 0);
    }

    void update(MotionPlusData& mp)
    {
        if (mpPrev.time < 0)
        {
            mpPrev = mp;
            return;
        }

        // Update min and max values in slow-mode
        if (!mp.fast.x)
        {
            slowMin.x = std::min(slowMin.x, mp.raw.x);
            slowMax.x = std::max(slowMax.x, mp.raw.x);
        }
        if (!mp.fast.y)
        {
            slowMin.y = std::min(slowMin.y, mp.raw.y);
            slowMax.y = std::max(slowMax.y, mp.raw.y);
        }
        if (!mp.fast.z)
        {
            slowMin.z = std::min(slowMin.z, mp.raw.z);
            slowMax.z = std::max(slowMax.z, mp.raw.z);
        }

        // Update fast values when switching between slow and fast mode
        updateFast(mpPrev, mp);

        // Store last state
        mpPrev = mp;
    }
};
#endif

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------

namespace
{

bool NormalizeAccel(AccelData& accel)
{
    if (accel.cal.valid)
    {
        accel.normalized.x = (float)(accel.raw.x - accel.cal.zero.x) / (float)(accel.cal.g.x - accel.cal.zero.x);
        accel.normalized.y = (float)(accel.raw.y - accel.cal.zero.y) / (float)(accel.cal.g.y - accel.cal.zero.y);
        accel.normalized.z = (float)(accel.raw.z - accel.cal.zero.z) / (float)(accel.cal.g.z - accel.cal.zero.z);
    }
    else
    {
        accel.normalized.x = 0.0f;
        accel.normalized.y = 0.0f;
        accel.normalized.z = 1.0f;
    }

    return accel.cal.valid;
}

bool NormalizeStick(JoystickData& stick)
{
    if (stick.cal.valid)
    {
        stick.normalized.x = 2.0f * (float)(stick.raw.x - stick.cal.center.x) / (float)(stick.cal.max.x - stick.cal.min.x);
        stick.normalized.y = 2.0f * (float)(stick.raw.y - stick.cal.center.y) / (float)(stick.cal.max.y - stick.cal.min.y);
    }
    else
    {
        stick.normalized.x = 2.0f * (stick.raw.x - 127.0f) / 255.0f;
        stick.normalized.y = 2.0f * (stick.raw.y - 127.0f) / 255.0f;
    }

    return stick.cal.valid;
}

bool NormalizeMotionPlus(MotionPlusData& mp)
{
    Point3i B;
    Point3f S;

    if (mp.cal.valid)
    {
        B.x = mp.fast.x ? mp.cal.biasFast.x : mp.cal.biasSlow.x;
        B.y = mp.fast.y ? mp.cal.biasFast.y : mp.cal.biasSlow.y;
        B.z = mp.fast.z ? mp.cal.biasFast.z : mp.cal.biasSlow.z;

        S.x = mp.fast.x ? mp.cal.scaleFast.x : mp.cal.scaleSlow.x;
        S.y = mp.fast.y ? mp.cal.scaleFast.y : mp.cal.scaleSlow.y;
        S.z = mp.fast.z ? mp.cal.scaleFast.z : mp.cal.scaleSlow.z;
    }
    else
    {
        float scaleSlow = 0.05f;
        float scaleFast = scaleSlow * 4.54f;

        B.x = 8063;
        B.y = 8063;
        B.z = 8063;

        S.x = mp.fast.x ? scaleFast : scaleSlow;
        S.y = mp.fast.y ? scaleFast : scaleSlow;
        S.z = mp.fast.z ? scaleFast : scaleSlow;
    }

    mp.normalized.x = (mp.raw.x - B.x) * S.x;
    mp.normalized.y = (mp.raw.y - B.y) * S.y;
    mp.normalized.z = (mp.raw.z - B.z) * S.z;

    return mp.cal.valid;
}

} // namespace

//--------------------------------------------------------------------------------------------------
// Common
//--------------------------------------------------------------------------------------------------

bool wii::ParseAccelCalibrationData(AccelData::CalibrationData& cal, uint8_t const* buf)
{
    //
    // The four bytes starting at 0x0016 and 0x0020 store the calibrated zero offsets for the
    // accelerometer (high 8 bits of X,Y,Z in the first three bytes, low 2 bits packed in the
    // fourth byte as --XXYYZZ). Apparently, the four bytes at 0x001A and 0x24 store the force
    // of gravity on those axes. The function of other data bytes is not known, and most of them
    // differ between Wii Remotes
    //

    cal.zero.x  = (buf[0] << 2) | ((buf[3] & 0x30) >> 4);
    cal.zero.y  = (buf[1] << 2) | ((buf[3] & 0x0C) >> 2);
    cal.zero.z  = (buf[2] << 2) | ((buf[3] & 0x03) >> 0);
    cal.g.x     = (buf[4] << 2) | ((buf[7] & 0x30) >> 4);
    cal.g.y     = (buf[5] << 2) | ((buf[7] & 0x0C) >> 2);
    cal.g.z     = (buf[6] << 2) | ((buf[7] & 0x03) >> 0);

    cal.valid   = cal.zero.x != cal.g.x &&
                  cal.zero.y != cal.g.y &&
                  cal.zero.z != cal.g.z;

    return cal.valid;
}

bool wii::ParseStickCalibrationData(JoystickData::CalibrationData& cal, uint8_t const* buf)
{
    cal.max.x       = buf[0];
    cal.min.x       = buf[1];
    cal.center.x    = buf[2];
    cal.max.y       = buf[3];
    cal.min.y       = buf[4];
    cal.center.y    = buf[5];
    cal.valid       = cal.min.x < cal.center.x && cal.center.x < cal.max.x &&
                      cal.min.y < cal.center.y && cal.center.y < cal.max.y;

#if 0
    if (!cal.valid)
    {
        cal.max.x       = 255;
        cal.min.x       = 0;
        cal.center.x    = 127;
        cal.max.y       = 255;
        cal.min.y       = 0;
        cal.center.y    = 127;
        cal.valid       = true;
    }
#endif

    return cal.valid;
}

//--------------------------------------------------------------------------------------------------
// Wiimote
//--------------------------------------------------------------------------------------------------

bool wii::ParseButtons(State& state, uint8_t const* buf)
{
    unsigned buttons = state.buttons;

    state.buttons = (buf[0] | (buf[1] << 8)) & State::ButtonMask;

    state.buttonsPressed = RecentlySet(buttons, state.buttons);
    state.buttonsReleased = RecentlyCleared(buttons, state.buttons);

    state.data |= State::Buttons;

    return true;
}

bool wii::ParseAccel(State& state, uint8_t const* buf)
{
    AccelData& acc = state.accel;

    acc.raw.x = (buf[2] << 2) | ((buf[0] & 0x60) >> 5);
    acc.raw.y = (buf[3] << 2) | ((buf[1] & 0x20) >> 4);
    acc.raw.z = (buf[4] << 2) | ((buf[1] & 0x40) >> 5);

    NormalizeAccel(acc);

    state.data |= State::Accel;

    return true;
}

bool wii::ParseIR(State& state, uint8_t const* buf)
{
    IRData& ir = state.ir;

    switch (ir.mode)
    {
    case IRData::Mode::Off:
        //
        // This function should not be called if the IR sensor is disabled
        //
        return false;

    case IRData::Mode::Basic:
        //
        // In Basic Mode, the IR Camera returns 10 bytes of data corresponding to the X and
        // Y locations of each of the four dots. Each location is encoded in 10 bits and has
        // a range of 0-1023 for the X dimension, and 0-767 for the Y dimension. Each pair
        // of dots is packed into 5 bytes, and two of these are transmitted for a total of
        // 4 dots and 10 bytes.
        //

        ir.dots[0].raw.x = buf[0] | ((buf[2] & 0x30) << 4);
        ir.dots[0].raw.y = buf[1] | ((buf[2] & 0xC0) << 2);
        ir.dots[1].raw.x = buf[3] | ((buf[2] & 0x03) << 8);
        ir.dots[1].raw.y = buf[4] | ((buf[2] & 0x0C) << 6);
        ir.dots[2].raw.x = buf[5] | ((buf[5] & 0x30) << 4);
        ir.dots[2].raw.y = buf[6] | ((buf[5] & 0xC0) << 2);
        ir.dots[3].raw.x = buf[8] | ((buf[5] & 0x03) << 8);
        ir.dots[3].raw.y = buf[9] | ((buf[5] & 0x0C) << 6);

        ir.dots[0].size = 0;
        ir.dots[1].size = 0;
        ir.dots[2].size = 0;
        ir.dots[3].size = 0;
        break;

    case IRData::Mode::Extended:
        //
        // In Extended Mode, the IR Camera returns the same data as it does in Basic Mode,
        // plus a rough size value for each object. The data is returned as 12 bytes, three
        // bytes per object. Size has a range of 0-15.
        //

        ir.dots[0].raw.x = buf[ 0] | ((buf[ 2] & 0x30) << 4);
        ir.dots[0].raw.y = buf[ 1] | ((buf[ 2] & 0xC0) << 2);
        ir.dots[1].raw.x = buf[ 3] | ((buf[ 5] & 0x30) << 4);
        ir.dots[1].raw.y = buf[ 4] | ((buf[ 5] & 0xC0) << 2);
        ir.dots[2].raw.x = buf[ 6] | ((buf[ 8] & 0x30) << 4);
        ir.dots[2].raw.y = buf[ 7] | ((buf[ 8] & 0xC0) << 2);
        ir.dots[3].raw.x = buf[ 9] | ((buf[11] & 0x30) << 4);
        ir.dots[3].raw.y = buf[10] | ((buf[11] & 0xC0) << 2);

        ir.dots[0].size = buf[ 2] & 0x0F;
        ir.dots[1].size = buf[ 5] & 0x0F;
        ir.dots[2].size = buf[ 8] & 0x0F;
        ir.dots[3].size = buf[11] & 0x0F;
        break;

    default:
        return false;
    }

    // Normalize IR data
    for (auto& dot : ir.dots)
    {
        // Compute normalized position
        dot.normalized.x = dot.raw.x / 1023.0f;
        dot.normalized.y = dot.raw.y /  767.0f;

        // And check whether this IR dot is visible
        dot.visible = dot.raw.x != 0x3FF && dot.raw.y != 0x3FF;
    }

    state.data |= State::IR;

    return true;
}

bool wii::ParseCalibrationData(State& state, uint8_t const* buf, unsigned /*len*/, unsigned /*error*/)
{
    //
    // TODO:
    // Use backup data on failure
    //

    ParseAccelCalibrationData(state.accel.cal, buf);

    return true;
}

//--------------------------------------------------------------------------------------------------
// Extensions
//--------------------------------------------------------------------------------------------------

bool wii::ParseNunchuk(State& state, uint8_t const* buf, bool passthrough)
{
    NunchukData& nc = state.extension.nunchuk;

    unsigned buttons = nc.buttons;

    nc.time = state.time;

    nc.stick.raw.x = buf[0];
    nc.stick.raw.y = buf[1];

    if (passthrough)
    {
        nc.accel.raw.x = ((buf[2]       ) << 2) | ((buf[5] & 0x10) >> 4);
        nc.accel.raw.y = ((buf[3]       ) << 2) | ((buf[5] & 0x20) >> 5);
        nc.accel.raw.z = ((buf[4] & 0xFE) << 2) | ((buf[5] & 0xC0) >> 5);

        nc.buttons = ((~buf[5]) >> 2) & 0x03;
    }
    else
    {
        nc.accel.raw.x = (buf[2] << 2) | ((buf[5] & 0x0C) >> 2);
        nc.accel.raw.y = (buf[3] << 2) | ((buf[5] & 0x30) >> 4);
        nc.accel.raw.z = (buf[4] << 2) | ((buf[5] & 0xC0) >> 6);

        nc.buttons = (~buf[5]) & 0x03;
    }

    NormalizeAccel(nc.accel);
    NormalizeStick(nc.stick);

    nc.buttonsPressed = RecentlySet(buttons, nc.buttons);
    nc.buttonsReleased = RecentlyCleared(buttons, nc.buttons);

    state.data |= State::Nunchuk;

    return true;
}

bool wii::ParseClassicController(State& state, uint8_t const* buf, bool passthrough)
{
    ClassicControllerData& cc = state.extension.classic;

    unsigned buttons = cc.buttons;

    cc.time = state.time;

    if (passthrough)
    {
        cc.buttons = ((~buf[4] & 0xFE) << 0) | ((~buf[5] & 0xFC) << 8) | ((~buf[0] & 0x01) << 8) | ((~buf[1] & 0x01) << 9);

        cc.stickL.raw.x = (buf[0] & 0x3E);
        cc.stickL.raw.y = (buf[1] & 0x3E);
    }
    else
    {
        cc.buttons = ((~buf[4] & 0xFE) << 0) | ((~buf[5] & 0xFF) << 8);

        cc.stickL.raw.x = (buf[0] & 0x3F);
        cc.stickL.raw.y = (buf[1] & 0x3F);
    }

    cc.stickR.raw.x = ((buf[2] & 0x80) >> 7) | ((buf[1] & 0xC0) >> 5) | ((buf[0] & 0xC0) >> 3);
    cc.stickR.raw.y = ((buf[2] & 0x1F) << 0);

    //
    // Scale joystick values to full range
    //
    cc.stickL.raw.x <<= 2;
    cc.stickL.raw.y <<= 2;
    cc.stickR.raw.x <<= 3;
    cc.stickR.raw.y <<= 3;

    //
    // Normalize:
    //
    NormalizeStick(cc.stickL);
    NormalizeStick(cc.stickR);

    cc.buttonsPressed = RecentlySet(buttons, cc.buttons);
    cc.buttonsReleased = RecentlyCleared(buttons, cc.buttons);

    state.data |= State::ClassicController;

    return 0;
}

bool wii::ParseMotionPlus(State& state, uint8_t const* buf)
{
    MotionPlusData& mp = state.extension.motionPlus;

    mp.time = state.time;

    mp.raw.x = buf[2] | ((buf[5] & 0xFC) << 6);
    mp.raw.y = buf[1] | ((buf[4] & 0xFC) << 6);
    mp.raw.z = buf[0] | ((buf[3] & 0xFC) << 6);

    mp.fast.x = (buf[3] & 0x01) == 0;
    mp.fast.y = (buf[4] & 0x02) == 0;
    mp.fast.z = (buf[3] & 0x02) == 0;

    mp.ext = (buf[4] & 0x01) != 0;

    NormalizeMotionPlus(mp);

    state.data |= State::MotionPlus;

#if WII_LOG_MP_STATISTICS
    static MotionPlusStatistics stat;

    stat.update(mp);

    WII_LOG(MP,
        "MP: slow: min: % 9d % 9d % 9d max: % 9d % 9d % 9d\n"
        "MP: fast: min: % 9d % 9d % 9d max: % 9d % 9d % 9d\n"
        ,
        stat.slowMin.x, stat.slowMin.y, stat.slowMin.z, stat.slowMax.x, stat.slowMax.y, stat.slowMax.z
        ,
        stat.fastMin.x, stat.fastMin.y, stat.fastMin.z, stat.fastMax.x, stat.fastMax.y, stat.fastMax.z
        );
#endif

    return true;
}

bool wii::ParseExtension(State& state, uint8_t const* buf)
{
    if (state.extension.type == 0)
        return true;

    unsigned motionPlus = state.extension.type &  Extension::MotionPlus;
    unsigned other      = state.extension.type & ~Extension::MotionPlus;

    bool passthrough = motionPlus && other;

    if ((motionPlus && !other) || (passthrough && (buf[5] & 0x02)))
    {
        // This report contains motion-plus data
        return ParseMotionPlus(state, buf);
    }
    else
    {
        // This report contains extension data
        switch (other)
        {
        case Extension::Nunchuk:
            return ParseNunchuk(state, buf, passthrough);
        case Extension::ClassicController:
            return ParseClassicController(state, buf, passthrough);
        }
    }

    return false;
}

bool wii::ParseNunchukCalibrationData(State& state, uint8_t const* buf, unsigned /*len*/, unsigned /*error*/)
{
    //
    // TODO:
    // Use backup data on failure
    //

    ParseAccelCalibrationData(state.extension.nunchuk.accel.cal, buf + 0);
    ParseStickCalibrationData(state.extension.nunchuk.stick.cal, buf + 8);

    return true;
}

bool wii::ParseClassicControllerCalibrationData(State& state, uint8_t const* buf, unsigned /*len*/, unsigned /*error*/)
{
    //
    // TODO:
    // Use backup data on failure
    //

    ParseStickCalibrationData(state.extension.classic.stickL.cal, buf + 0);
    ParseStickCalibrationData(state.extension.classic.stickR.cal, buf + 6);

    return 0;
}

bool wii::ParseExtensionCalibrationData(State& state, uint8_t const* buf, unsigned len, unsigned error)
{
    // Get the extension type
    unsigned ext = state.extension.type & ~Extension::MotionPlus;

    switch (ext)
    {
    case Extension::Nunchuk:
        return ParseNunchukCalibrationData(state, buf, len, error);
    case Extension::ClassicController:
        return ParseClassicControllerCalibrationData(state, buf, len, error);
    case 0:
        return true;
    }

    return false;
}

bool wii::ParseMotionPlusCalibrationData(State& state, uint8_t const* buf, unsigned /*len*/, unsigned /*error*/)
{
    MotionPlusData::CalibrationData& cal = state.extension.motionPlus.cal;

    cal.biasSlow.x = (buf[2] << 8 | buf[3]) / 4;
    cal.biasSlow.y = (buf[4] << 8 | buf[5]) / 4;
    cal.biasSlow.z = (buf[0] << 8 | buf[1]) / 4;

#if 0
    cal.biasFast.x = 8192;
    cal.biasFast.y = 8192;
    cal.biasFast.z = 8192;
#else
    cal.biasFast.x = static_cast<int>(cal.biasSlow.x * 4.54f/4.4f + 0.5f);
    cal.biasFast.y = static_cast<int>(cal.biasSlow.z * 4.54f/4.4f + 0.5f);
    cal.biasFast.z = static_cast<int>(cal.biasSlow.y * 4.54f/4.4f + 0.5f);
#endif

    // Slow mode: convert units into deg/s
    cal.scaleSlow.x = 0.05f;
    cal.scaleSlow.y = 0.05f;
    cal.scaleSlow.z = 0.05f;

    // Same for fast mode.
    cal.scaleFast.x = cal.scaleSlow.x * 4.54f;
    cal.scaleFast.y = cal.scaleSlow.y * 4.54f;
    cal.scaleFast.z = cal.scaleSlow.z * 4.54f;

    cal.valid = true;

#if 0
    for (unsigned n = 0; n < len; ++n)
    {
        if (n % 16 == 0)
            printf("\n%08x: ", n);
        printf("%02x ", buf[n]);
    }
    printf("\n");
#endif

    return true;
}
