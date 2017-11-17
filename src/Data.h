// This file is distributed under the MIT license.
// See the LICENSE file for details.

#pragma once

#include "Wiimote/Wiimote.h"

namespace wii
{

//--------------------------------------------------------------------------------------------------
// Common
//--------------------------------------------------------------------------------------------------

bool ParseAccelCalibrationData(AccelData::CalibrationData& cal, uint8_t const* buf);

bool ParseStickCalibrationData(JoystickData::CalibrationData& cal, uint8_t const* buf);

//--------------------------------------------------------------------------------------------------
// Wiimote
//--------------------------------------------------------------------------------------------------

bool ParseButtons(State& state, uint8_t const* buf);

bool ParseAccel(State& state, uint8_t const* buf);

bool ParseIR(State& state, uint8_t const* buf);

bool ParseCalibrationData(State& state, uint8_t const* buf, unsigned len, unsigned error);

//--------------------------------------------------------------------------------------------------
// Extensions
//--------------------------------------------------------------------------------------------------

bool ParseNunchuk(State& state, uint8_t const* buf, bool passthrough = false);

bool ParseClassicController(State& state, uint8_t const* buf, bool passthrough = false);

bool ParseMotionPlus(State& state, uint8_t const* buf);

bool ParseExtension(State& state, uint8_t const* buf);

bool ParseNunchukCalibrationData(State& state, uint8_t const* buf, unsigned len, unsigned error);

bool ParseClassicControllerCalibrationData(State& state, uint8_t const* buf, unsigned len, unsigned error);

bool ParseExtensionCalibrationData(State& state, uint8_t const* buf, unsigned len, unsigned error);

bool ParseMotionPlusCalibrationData(State& state, uint8_t const* buf, unsigned len, unsigned error);

} // namespace wii
