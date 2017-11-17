// This file is distributed under the MIT license.
// See the LICENSE file for details.

#pragma once

#include <memory>

#ifdef WIIMOTE_EXPORTS
#define WIIAPI __declspec(dllexport)
#else
#define WIIAPI __declspec(dllimport)
#endif

namespace wii
{

struct Point2i
{
    int x;
    int y;
};

struct Point2f
{
    float x;
    float y;
};

struct Point3i
{
    int x;
    int y;
    int z;
};

struct Point3f
{
    float x;
    float y;
    float z;
};

//
// NOTE:
//
// The accelerometer and gyroscope (motion-plus) are relative to the Wiimote's coordinate
// system (http://wiibrew.org/wiki/Wiimote#Accelerometer)
//
// Wiimote coordinate system:
// Wiimote pointing along the negative y-axis.
//
//
//      z                +---+
//      |                | + |
//      |           x ---|   |
//      +----- y         | : |
//     /                 +---+
//    /                    |
//   x                     y
//
//

// Accelerometer information used by Wiimote and Nunchuk
struct AccelData
{
    struct CalibrationData
    {
        // Zero point of accelerometer
        Point3i zero;
        // Gravity at rest of accelerometer
        Point3i g;
        // Whether calibration data is valid
        bool valid;
    };

    // Raw accelerometer data
    Point3i raw;
    // Normalized accelerometer data in units where g=1
    Point3f normalized;
    // Calibration data
    CalibrationData cal;
};

// Joystick information used by Wii Remote and Nunchuk
struct JoystickData
{
    struct CalibrationData
    {
        // Maximum values
        Point2i max;
        // Minimum values
        Point2i min;
        // Center position
        Point2i center;
        // Whether calibration data is valid
        bool valid;
    };

    // Raw joystick values
    Point2i raw;
    // Normalized joystick values in [-1,1]x[-1,1]
    Point2f normalized;
    // Calibration data
    CalibrationData cal;
};

struct IRData
{
    enum Mode {
        Off         = 0,
        Basic       = 0x01,
        Extended    = 0x03,
    };

    enum Sensitivity {
        Level1,
        Level2,
        Level3,
        Level4,
        Level5,
    };

    struct Dot
    {
        // Raw position in [0,1023]x[0,767]
        Point2i raw;
        // Normalized position in [0,1]x[0,1]
        Point2f normalized;
        // Rough size estimate in [0,15]
        // Only valid if current IR mode is Extended or Full
        unsigned size;
        // Whether the IR dot is visible
        bool visible;
    };

    // IR mode; implicitly set by SetReportMode
    Mode mode;
    // IR sensor sensitivitystruct
    Sensitivity sensitivity;
    // IR dots
    Dot dots[4];
};

struct NunchukData
{
    enum Button {
        Z = 0x01,
        C = 0x02,
    };

    // Timing
    // NOTE: Might be different from Wiimote's time if in pass-through mode
    double time;
    // Accelerometer data
    AccelData accel;
    // Joystick data
    JoystickData stick;
    // Buttons
    unsigned buttons;
    // Recently pressed
    unsigned buttonsPressed;
    // Recently released
    unsigned buttonsReleased;
};

struct ClassicControllerData
{
    enum Button : unsigned {
        R       = 0x0002,
        Plus    = 0x0004,
        Home    = 0x0008,
        Minus   = 0x0010,
        L       = 0x0020,
        Down    = 0x0040,
        Right   = 0x0080,
        Up      = 0x0100, // = (buf[0] & 0x01) << 8 in pass-through
        Left    = 0x0200, // = (buf[1] & 0x01) << 9 in pass-through
        ZR      = 0x0400,
        X       = 0x0800,
        A       = 0x1000,
        Y       = 0x2000,
        B       = 0x4000,
        ZL      = 0x8000,
    };

    // Timing
    // NOTE: Might be different from Wiimote's time if in pass-through mode
    double time;
    // Currently pressed buttons
    unsigned buttons;
    // Recently pressed buttons
    unsigned buttonsPressed;
    // Recently released buttons
    unsigned buttonsReleased;
    // Left joystick
    JoystickData stickL;
    // Right joystick
    JoystickData stickR;
};

struct MotionPlusData
{
    struct CalibrationData // XXX
    {
        // Gyro bias slow mode
        Point3i biasSlow;
        // Gyro bias fast mode
        Point3i biasFast;
        // Scaling factors slow mode
        Point3f scaleSlow;
        // Scaling factors fast mode
        Point3f scaleFast;
        // Whether calibration data is valid
        bool valid;
    };

    // Timing
    // NOTE: Might be different from Wiimote's m_state.time if in pass-through mode
    double time;
    // Raw agular rate values
    Point3i raw;
    // Whether the raw values are in fast or slow units resp.
    Point3i fast;
    // Normalized angular rate value: (rate + bias) deg/sec
    Point3f normalized;
    // Whether an extension is connected to the motion-plus
    bool ext;
    // Internal status.
    unsigned status;
    // Calibration data
    CalibrationData cal;
};

struct Extension
{
    enum Type : unsigned {
        Nunchuk             = 0x0001,
        ClassicController   = 0x0002,
        MotionPlus          = 0x1000,
    };

    // Extensions type
    unsigned type;
    // Motion-Plus data
    MotionPlusData motionPlus;
    // Extensions data
    union
    {
        // Timing
        // NOTE: Might be different from Wiimote's time if in pass-through mode
        double time;
        // Nunchuk data
        NunchukData nunchuk;
        // Classic controller data
        ClassicControllerData classic;
    };
};

struct State
{
    enum Data : unsigned {
        Buttons             = 0x0001,
        Accel               = 0x0002,
        IR                  = 0x0004,
        Nunchuk             = 0x0008,
        ClassicController   = 0x0010,
        MotionPlus          = 0x1000,
    };

    enum LED : unsigned {
        LED1    = 0x10,
        LED2    = 0x20,
        LED3    = 0x40,
        LED4    = 0x80,
        LEDMask = 0xF0,
    };

    enum Button : unsigned {
        Left        = 0x0001,
        Right       = 0x0002,
        Down        = 0x0004,
        Up          = 0x0008,
        Plus        = 0x0010,
        Two         = 0x0100,
        One         = 0x0200,
        B           = 0x0400,
        A           = 0x0800,
        Minus       = 0x1000,
        Home        = 0x8000,
        ButtonMask  = 0x9F1F,
    };

    // Determines what kind of data is valid (see 'enum Data')
    unsigned data;
    // The current state's time in seconds
    double time;
    // Raw battery status (~180 for full, <60 for low)
    unsigned battery;
    // Whether the battery is nearly empty
    bool batteryLow;
    // Whether an extension is plugged in
    bool extPresent;
    // Whther the speaker is enabled
    bool speakerEnabled;
    // Whether the IR camera is enabled
    bool irEnabled;
    // Whether rumble is on
    bool rumble;
    // LED state (see 'enum LED')
    unsigned leds;
    // Button state (see 'enum Button')
    unsigned buttons;
    // Recently pressed buttons
    unsigned buttonsPressed;
    // Recently released buttons
    unsigned buttonsReleased;
    // Accelerometer data
    AccelData accel;
    // IR camera status
    IRData ir;
    // Extensions status
    Extension extension;
};

class Wiimote
{
    struct Impl;
    std::unique_ptr<Impl> impl;

public:
    enum class ReportMode {
        Undefined           = 0,
        Buttons             = 0x30,
        ButtonsAccel        = 0x31,
        ButtonsExt          = 0x32,
        ButtonsAccelIR      = 0x33,
        ButtonsAccelExt     = 0x35,
        ButtonsIRExt        = 0x36,
        ButtonsAccelIRExt   = 0x37,
    };

public:
    // Constructor
    WIIAPI Wiimote();

    // Destructor
    WIIAPI ~Wiimote();

    // Connect to first found Wiimote
    WIIAPI bool Connect();

    // Set the report mode
    // IR sensitivity level is set to Level3
    WIIAPI bool SetReportMode(ReportMode mode, bool continuous = true);

    // Set the report mode
    WIIAPI bool SetReportMode(ReportMode mode, IRData::Sensitivity sensitivity, bool continuous = true);

    // Set the LEDs
    WIIAPI bool SetLEDs(unsigned leds);

    // Set the LEDs
    WIIAPI bool SetLEDs(bool led1, bool led2, bool led3, bool led4);

    // Set rumble
    WIIAPI bool SetRumble(bool enable);

    // Poll data from this wiimote
    WIIAPI bool Poll();

    // Properly shutdown this Wiimote
    WIIAPI bool Shutdown();

    // Disconnect this Wiimote
    WIIAPI bool Disconnect();

    // Enable motion-plus -- if any
    WIIAPI bool CheckForMotionPlus();

    // Disable motion-plus -- if any
    WIIAPI bool DisableMotionPlus();

    // Get current wiimote state
    WIIAPI State const& GetState() const;
};

} // namespace wii
