// This file is distributed under the MIT license.
// See the LICENSE file for details.

#pragma once

#include "Wiimote/Wiimote.h"

#include <cassert>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

#define WII_REPORT_LENGTH               22

#define WII_VENDOR_ID                   0x057E // Nintendo
#define WII_PRODUCT_ID                  0x0306 // Nintendo RVL-CNT-01
#define WII_PRODUCT_ID_2                0x0330 // Nintendo RVL-CNT-01-TR

#define WII_OUTPUT_UNKNOWN              0x10 // Rumble report?!
#define WII_OUTPUT_LEDS                 0x11
#define WII_OUTPUT_REPORT_MODE          0x12
#define WII_OUTPUT_ENABLE_IR_1          0x13
#define WII_OUTPUT_ENABLE_SPEAKER       0x14
#define WII_OUTPUT_STATUS               0x15
#define WII_OUTPUT_WRITE_MEMORY         0x16
#define WII_OUTPUT_READ_MEMORY          0x17
#define WII_OUTPUT_SPEAKER_DATA         0x18
#define WII_OUTPUT_MUTE_SPEAKER         0x19
#define WII_OUTPUT_ENABLE_IR_2          0x1A

// Internal Wiimote status
#define WII_STATUS_UNKNOWN              0
#define WII_STATUS_CONNECTED            1
#define WII_STATUS_STARTUP              2
#define WII_STATUS_READY                3
#define WII_STATUS_SHUTDOWN             4
#define WII_STATUS_SHUTDOWN_COMPLETE    5
#define WII_STATUS_DISCONNECTED         6
#define WII_STATUS_ERROR                7

// Internal motion-plus status
#define WII_STATUS_MP_UNKNOWN           0
#define WII_STATUS_MP_STARTUP           1
#define WII_STATUS_MP_NOT_PRESENT       2
#define WII_STATUS_MP_ACTIVE            3
#define WII_STATUS_MP_SHUTDOWN          4
#define WII_STATUS_MP_INACTIVE          5
#define WII_STATUS_MP_NO_LONGER_ACTIVE  6

namespace wii
{

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------

using ReadHandler = std::function<bool (uint8_t const* buf, unsigned len, unsigned error)>;

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------

// Identifies a read/write/status request
struct Request
{
    enum class Type {
        Status,
        Read,
        Write,
    };

    Request(Type type, unsigned address, unsigned size, uint8_t const* data = nullptr)
        : type(type)
        , buffer(size)
        , address(address)
        , error(0)
        , done(0)
        , pending(size)
        , sent(false)
        , handler()
    {
        if (data)
        {
            assert(size != 0);
            std::memcpy(&buffer[0], data, size);
        }
    }

    // Type of the request
    Type type;
    // Buffer
    std::vector<uint8_t> buffer;
    // Address
    unsigned address;
    // Last error code
    unsigned error;
    // Bytes already read/written
    unsigned done;
    // Bytes waiting to be read/written
    unsigned pending;
    // Whether the output report for this request has been written.
    bool sent;
    // The callback
    ReadHandler handler;
};

using Requests = std::deque<Request>;

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------

struct Wiimote::Impl
{
    // The current state of the wiimote and expansions
    State state;
    // Current report mode
    ReportMode reportMode;
    // Whether the Wiimote should operate in continuous mode, ie. send reports even
    // if the data hasn't changed.
    bool continous;
    // List of pending read/write requests
    Requests requests;
    // Internal status
    unsigned status;
    // Current motion-plus status
    unsigned motionPlusStatus;
#ifdef _WIN32
    // Wiimote device handle
    HANDLE device;
    // Overlapped data structure for asynchronuous reads
    OVERLAPPED overlapped;
#endif

public:
    //--------------------------------------------------------------------------
    // Common
    //

    // Constructor
    Impl();

    // Destructor
    ~Impl();

    // Write a report to the wiimote
    bool SendReport(uint8_t type, uint8_t const* data, unsigned size);

    // Write a report to the wiimote
    bool SendReport(uint8_t type, uint8_t data);

    // Write a report to the wiimote
    bool SendStatusReport();

    // Write a report to the wiimote
    bool SendReadReport(unsigned address, unsigned size);

    // Write a report to the wiimote
    bool SendWriteReport(unsigned address, unsigned size, uint8_t const* data);

    // Pushes a new request onto the queue
    void PushRequest(Request::Type type, unsigned address = 0, unsigned size = 0, uint8_t const* buffer = 0);

    // Pops a request from the queue; frees the requests resources
    void PopRequest();

    // Checks for pending requests and writes a report to the wiimote -- if any
    bool SendNextRequest();

    // Sets the report mode
    bool SetReportMode(ReportMode mode, IRData::Sensitivity sensitivity, bool continous_);

    // Poll a report from the Wiimote
    bool Poll();

    // Initialize the shutdown process
    bool Shutdown();

    // Request a status report
    void RequestStatusReport();

    // Read data from the wiimote
    void ReadData(unsigned address, unsigned size);

    // Read data from the wiimote then invoke the handler
    void ReadData(unsigned address, unsigned size, ReadHandler handler);

    // Write data to the wiimote
    void WriteData(unsigned address, uint8_t const* data, unsigned size);

    // Write a single byte to the wiimote
    void WriteData(unsigned address, uint8_t data);

    // Parse an input report
    bool ProcessReport(uint8_t const* buf);

    // Parse a status report
    bool ProcessStatusReport(uint8_t const* buf);

    // Parse a data report
    bool ProcessDataReport(uint8_t const* buf);

    // Parse an acknowledge report
    bool ProcessAcknowledgeReport(uint8_t const* buf);

    // Parse the extension identifier
    bool ParseExtensionIdentifier(uint8_t const* buf, unsigned len, unsigned error);

    // Parse the motion-plus identifier
    bool ParseMotionPlusIdentifier(uint8_t const* buf, unsigned len, unsigned error);

    // Enable IR camera
    bool EnableIR(IRData::Mode mode, IRData::Sensitivity sensitivity);

    // Disable IR camera
    bool DisableIR();

    // Enable motion-plus -- if any
    bool EnableMotionPlus();

    // Disable motion-plus -- if any
    bool DisableMotionPlus();

    //--------------------------------------------------------------------------
    //

    // Read Wiimote calibration data
    void ReadCalibrationData();

    // Read extension calibration data
    void ReadExtensionCalibrationData();

    // Read motion-plus calibration data
    void ReadMotionPlusCalibrationData();

    // Read the extension identifier
    void ReadExtensionIdentifier();

    // Read the motion-plus identifier
    void ReadMotionPlusIdentifier();

    // Initialize the extension
    void InitExtension();

    // Initialize the motion-plus
    void InitMotionPlus();

    //--------------------------------------------------------------------------
    //

    // Enable/Disable leds
    bool SetLEDs(unsigned leds);

    // Enable/Disable rumble
    bool SetRumble(bool enable);

    //--------------------------------------------------------------------------
    // OS-specifc
    // Implemented in Wiimote-?.inl
    //

    // Initialize
    void Init();

    // Clean-up
    void Finish();

    // Disconnect this Wiimote
    void Disconnect();

    // Read a report from the wiimote
    bool GetInputReport(uint8_t* report);

    // Write a report to the wiimote
    bool SetOutputReport(uint8_t const* report, unsigned len);

    // Returns current time
    double Time();

#ifdef _WIN32
    // Open a device handle for the specified device and check if it's a wiimote
    // Returns INVALID_HANDLE_VALUE on failure
    static HANDLE OpenDeviceHandle(LPCTSTR devicePath);
#endif

    // Connect all Wiimotes
    static bool Connect(Wiimote* wiimotes, unsigned& count);
};

} // namespace wii
