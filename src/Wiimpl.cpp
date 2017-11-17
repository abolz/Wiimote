// This file is distributed under the MIT license.
// See the LICENSE file for details.

#include "Wiimpl.h"
#include "Data.h"
#include "Log.h"
#include "Utils.h"

using namespace wii;

//--------------------------------------------------------------------------------------------------
// Common
//--------------------------------------------------------------------------------------------------

Wiimote::Impl::Impl()
    : state()
    , reportMode(ReportMode::Undefined)
    , continous(true)
    , requests()
    , status(WII_STATUS_UNKNOWN)
    , device(INVALID_HANDLE_VALUE)
    , overlapped()
{
    // Clear the state!
    memset(&state, 0, sizeof(state));

    Init();
}

Wiimote::Impl::~Impl()
{
    assert( (status == WII_STATUS_UNKNOWN || status == WII_STATUS_DISCONNECTED || status == WII_STATUS_ERROR)
        && "Wiimote not properly disconnected" );

    Finish();
}

bool Wiimote::Impl::SendReport(uint8_t type, uint8_t const* data, unsigned size)
{
    assert(data);
    assert(0 < size && size < WII_REPORT_LENGTH);

    uint8_t report[WII_REPORT_LENGTH] = { 0 };

    memcpy(report + 1, data, size);

    report[0]  = type;
    report[1] |= state.rumble ? 0x01 : 0x00; // Remember to set rumble bit

    return SetOutputReport(report, size + 1);
}

bool Wiimote::Impl::SendReport(uint8_t type, uint8_t data)
{
    return SendReport(type, &data, 1);
}

bool Wiimote::Impl::SendStatusReport()
{
    return SendReport(WII_OUTPUT_STATUS, 0x00);
}

bool Wiimote::Impl::SendReadReport(unsigned address, unsigned size)
{
    uint8_t buf[6];

    buf[0] = B3(address);
    buf[1] = B2(address);
    buf[2] = B1(address);
    buf[3] = B0(address);
    buf[4] = B1(size);
    buf[5] = B0(size);

    return SendReport(WII_OUTPUT_READ_MEMORY, buf, 6);
}

bool Wiimote::Impl::SendWriteReport(unsigned address, unsigned size, uint8_t const* data)
{
    assert(data);
    assert(0 < size && size <= 16); // Maximum length is 16 bytes at once!

    uint8_t buf[21] = { 0 };

    buf[0] = B3(address);
    buf[1] = B2(address);
    buf[2] = B1(address);
    buf[3] = B0(address);
    buf[4] = B0(size);

    memcpy(buf + 5, data, size);

    return SendReport(WII_OUTPUT_WRITE_MEMORY, buf, 21);
}

void Wiimote::Impl::PushRequest(Request::Type type, unsigned address, unsigned size, uint8_t const* buffer)
{
    // Create request and add it to the queue
    requests.emplace_back(type, address, size, buffer);
}

void Wiimote::Impl::PopRequest()
{
    assert(!requests.empty());

    // and remove from queue
    requests.pop_front();
}

bool Wiimote::Impl::SendNextRequest()
{
    if (requests.empty() || requests.front().sent)
        return true;

    Request& req = requests.front();

    req.sent = true;

    switch (req.type)
    {
    case Request::Type::Status:
        return SendStatusReport();

    case Request::Type::Read:
        WII_LOG(READ, "ReadData: 0x%08X\n", req.address);
        return SendReadReport(req.address, req.pending);

    case Request::Type::Write:
        WII_LOG(WRITE, "WriteData: %08x\n", req.address);
        return SendWriteReport(req.address, req.pending, req.buffer.data());
    }

    return false;
}

bool Wiimote::Impl::SetReportMode(ReportMode mode, IRData::Sensitivity sensitivity, bool continous_)
{
    reportMode = mode;
    continous = continous_;

    switch (mode)
    {
    case ReportMode::ButtonsAccelIR:        // 12 IR bytes (extended)
        EnableIR(IRData::Mode::Extended, sensitivity);
        break;
    case ReportMode::ButtonsIRExt:          // 10 IR bytes (basic)
    case ReportMode::ButtonsAccelIRExt:     // 10 IR bytes (basic)
        EnableIR(IRData::Mode::Basic, sensitivity);
        break;
    default:
        DisableIR();
        break;
    }

    unsigned char buf[2] = { 0 };

    buf[0] = continous ? 0x04 : 0x00;
    buf[1] = static_cast<unsigned char>(mode);

    return SendReport(WII_OUTPUT_REPORT_MODE, buf, 2);
}

bool Wiimote::Impl::Poll()
{
    assert(reportMode != ReportMode::Undefined);

    if (status == WII_STATUS_ERROR || status == WII_STATUS_SHUTDOWN_COMPLETE)
        return false;

    // Send the next request
    SendNextRequest();

    unsigned char report[WII_REPORT_LENGTH] = { 0 };

    // Read a report from the wiimote
    if (!GetInputReport(report))
    {
        WII_LOG(STATUS, "Connection lost\n");

        status = WII_STATUS_ERROR; // Connection lost   .
        return false;
    }

    // Process the report
    if (!ProcessReport(report))
    {
#if 0
        status = WII_STATUS_ERROR;
        return false;
#endif
    }

    // Process any startup or shutdown requests
    if (status != WII_STATUS_READY)
    {
        if (status == WII_STATUS_CONNECTED)
        {
            // Read the calibration data and a status report.
            // Reading the status report initializes all extensions currently plugged in

            WII_LOG(INIT, "CONNECTED.\n");

            // Read calibration data
            ReadCalibrationData();

            // Request a status report
            PushRequest(Request::Type::Status);

            status = WII_STATUS_STARTUP;
        }
        else if (status == WII_STATUS_STARTUP)
        {
            // Currently processing calibration data and the first status report
            // to sync data structures with the wiimote

            WII_LOG(INIT, "Startup...\n");

            if (requests.empty())
            {
                // If there are no more pending requests the wiimote
                // is considered ready for use.

                WII_LOG(INIT, "READY.\n");

                // Now read the motion-plus identifier
                // This will fail if there is no motion-plus or if the motion-plus is already enabled,
                // otherwise this will enable the motion-plus.
                ReadMotionPlusIdentifier();

                status = WII_STATUS_READY;
            }
        }
        else if (status == WII_STATUS_SHUTDOWN)
        {
            // During shutdown, the motion-plus and rumble will be disabled.
            // If no more requests are pending it's safe to disconnect.

            WII_LOG(INIT, "Shutdown...\n");

            if (requests.empty())
            {
                WII_LOG(INIT, "Shutdown complete.\n");

                status = WII_STATUS_SHUTDOWN_COMPLETE;
            }
        }
    }

    return true;
}

bool Wiimote::Impl::Shutdown()
{
    if (status == WII_STATUS_SHUTDOWN || status == WII_STATUS_SHUTDOWN_COMPLETE)
        return true;
    if (status == WII_STATUS_UNKNOWN)
        return true;
    if (status == WII_STATUS_STARTUP)
        return true;
    if (status == WII_STATUS_DISCONNECTED)
        return true;
    if (status == WII_STATUS_ERROR)
        return true;

    // Disable motion-plus -- if any
    DisableMotionPlus();
    // Reset LEDs
    SetLEDs(0);
    // Disable Rumble!
    SetRumble(false);

    status = WII_STATUS_SHUTDOWN;

    return true;
}

void Wiimote::Impl::RequestStatusReport()
{
    PushRequest(Request::Type::Status);
}

void Wiimote::Impl::ReadData(unsigned address, unsigned size)
{
    PushRequest(Request::Type::Read, address, size);
}

void Wiimote::Impl::ReadData(unsigned address, unsigned size, ReadHandler handler)
{
    Request req(Request::Type::Read, address, size);

    req.handler = handler;

    requests.push_back(req);
}

void Wiimote::Impl::WriteData(unsigned address, uint8_t const* data, unsigned size)
{
    assert(0 < size && size <= 16);

    PushRequest(Request::Type::Write, address, size, data);
}

void Wiimote::Impl::WriteData(unsigned address, uint8_t data)
{
    WriteData(address, &data, 1);
}

bool Wiimote::Impl::ProcessReport(uint8_t const* buf /*[22]*/)
{
    state.data = 0; // Mark everything as invalid
    state.time = Time();

    switch (buf[0])
    {
    case 0x20: // Status
        ParseButtons(state, buf + 1);
        ProcessStatusReport(buf + 3);
        break;

    case 0x21: // Read Memory Data
        ParseButtons(state, buf + 1);
        ProcessDataReport(buf + 3);
        break;

    case 0x22: // Acknowledge output report, return function result
        ParseButtons(state, buf + 1);
        ProcessAcknowledgeReport(buf + 3);
        break;

    case 0x30: // Buttons
        ParseButtons(state, buf + 1);
        break;

    case 0x31: // ButtonsAccel
        ParseButtons(state, buf + 1);
        ParseAccel(state, buf + 1);
        break;

    case 0x32: // ButtonsExt
        ParseButtons(state, buf + 1);
        ParseExtension(state, buf + 3);
        break;

    case 0x33: // ButtonsAccelIR (12 IR bytes)
        ParseButtons(state, buf + 1);
        ParseAccel(state, buf + 1);
        ParseIR(state, buf + 6);
        break;

    case 0x35: // ButtonsAccelExt
        ParseButtons(state, buf + 1);
        ParseAccel(state, buf + 1);
        ParseExtension(state, buf + 6);
        break;

    case 0x36: // ButtonsIRExt (10 IR bytes)
        ParseButtons(state, buf + 1);
        ParseIR(state, buf + 3);
        ParseExtension(state, buf + 13);
        break;

    case 0x37: // ButtonsAccelIRExt (10 IR bytes)
        ParseButtons(state, buf + 1);
        ParseAccel(state, buf + 1);
        ParseIR(state, buf + 6);
        ParseExtension(state, buf + 16);
        break;

    default:
        assert(0); // Not implemented
        break;
    }

    return 0;
}

bool Wiimote::Impl::ProcessStatusReport(uint8_t const* buf)
{
    assert(reportMode != ReportMode::Undefined);

    WII_LOG(STATUS, "Status report:\n");

    //
    // Bit  Mask    Meaning
    //
    // 0    0x01    Battery is nearly empty
    // 1    0x02    An Extension Controller is connected
    // 2    0x04    Speaker enabled
    // 3    0x08    IR camera enabled
    // 4    0x10    LED 1
    // 5    0x20    LED 2
    // 6    0x40    LED 3
    // 7    0x80    LED 4
    //

    bool batteryLow     = (buf[0] & 0x01) != 0;
    bool extPresent     = (buf[0] & 0x02) != 0;
    bool speakerEnabled = (buf[0] & 0x04) != 0;
    bool irEnabled      = (buf[0] & 0x08) != 0;
    unsigned leds       = (buf[0] & 0xF0);
    unsigned battery    = (buf[3]);

    WII_LOG(STATUS, "  Extensions     : %08x\n", state.extension.type);
    WII_LOG(STATUS, "  Battery low    : %s\n", batteryLow ? "yes" : "no");
    WII_LOG(STATUS, "  Battery status : %u\n", battery);
    WII_LOG(STATUS, "  Extension      : %s\n", extPresent ? "yes" : "no");
    WII_LOG(STATUS, "  LEDs           : %u %u %u %u\n", !!(leds & 0x10), !!(leds & 0x20), !!(leds & 0x40), !!(leds & 0x80));
    WII_LOG(STATUS, "  Speaker        : %s\n", speakerEnabled ? "on" : "off");
    WII_LOG(STATUS, "  IR camera      : %s\n", irEnabled ? "on" : "off");

    //
    // This report is sent either on request (in response to report 0x15), or in response
    // to an expansion being plugged in or unplugged (or synced if wireless).
    //

    if (!requests.empty())
    {
        Request& req = requests.front();

        if (req.type == Request::Type::Status && req.sent)
        {
            WII_LOG(STATUS, "Status report removed from queue.\n");

            PopRequest();
        }
    }

    if (extPresent != state.extPresent)
    {
        if (extPresent)
        {
            //
            // The new way to initialize the extension is by writing 0x55 to 0x(4)A400F0,
            // then writing 0x00 to 0x(4)A400FB. It works on all extensions, and makes the
            // extension type bytes unencrypted.
            //

            if (state.extension.motionPlus.status != WII_STATUS_MP_STARTUP)
            {
                WriteData(0x04A400F0, 0x55);
                WriteData(0x04A400FB, 0x00);
            }

            //
            // Once initialized, the last six bytes of the register block identify the connected
            // Extension Controller. A six-byte read of register 0xa400fa will return these bytes.
            // The Extension Controller must have been initialized prior to this. There are two ways
            // of initializing the extension...
            //

            ReadExtensionIdentifier();
        }
        else
        {
            if (state.extension.motionPlus.status != WII_STATUS_MP_STARTUP)
            {
                state.extension.type = 0;
            }
        }
    }

    //
    // Update state
    //

    state.battery           = battery;
    state.batteryLow        = batteryLow;
    state.extPresent        = extPresent;
    state.speakerEnabled    = speakerEnabled;
    state.irEnabled         = irEnabled;
    state.leds              = leds;

    //
    // If this status report is received though not requested, the application MUST
    // send report 0x12 to change the data reporting mode, otherwise no further data
    // reports will be received
    //

    SetReportMode(reportMode, state.ir.sensitivity, continous);

    return true;
}

bool Wiimote::Impl::ProcessDataReport(uint8_t const* buf)
{
    assert(!requests.empty());

    // buf = SE AA AA DD DD DD DD DD DD DD DD DD DD DD DD DD DD DD DD

    unsigned count      = 1 + (buf[0] >> 4);
    unsigned error      = buf[0] & 0x0F;
    unsigned address    = buf[1] << 8 | buf[2];

    static_cast<void>(address); // unused in release builds...

    Request& req = requests.front();

    assert(req.type == Request::Type::Read);
    assert(count <= req.pending);
    assert(address == ((req.address + req.done) & 0xFFFF));

    req.error = error;

    //
    // Copy transferred bytes into buffer
    //

    if (req.error == 0)
    {
        // Copy the bytes from the report into the buffer
        std::memcpy(req.buffer.data() + req.done, buf + 3, count);

        req.done += count;
        req.pending -= count;
    }

    //
    // If an error ocurred, discard the read request
    // If there are no more bytes to read, process the read
    //

    if (req.error != 0 || req.pending == 0)
    {
        assert(req.handler);

        // Handle this read request
        req.handler(req.buffer.data(), static_cast<unsigned>(req.buffer.size()), req.error);

        // Remove the request from the queue
        PopRequest();
    }

    return true;
}

bool Wiimote::Impl::ProcessAcknowledgeReport(uint8_t const* buf)
{
#if 1
    assert(!requests.empty());

    // buf = RR EE

    unsigned reg    = buf[0];
    unsigned error  = buf[1];

    Request& req = requests.front();

    req.error = error;

    WII_LOG(WRITE, "Ack: reg: %02x error: %02x (address: %08x)\n", reg, error, req.address);

    if (reg == WII_OUTPUT_WRITE_MEMORY && req.type == Request::Type::Write)
    {
        PopRequest();
    }
#endif

    return true;
}

bool Wiimote::Impl::ParseExtensionIdentifier(uint8_t const* buf, unsigned /*len*/, unsigned /*error*/)
{
    unsigned motionPlus = state.extension.type &  Extension::MotionPlus;
    unsigned other      = state.extension.type & ~Extension::MotionPlus;

    WII_LOG(STATUS, "Extension identifier read...\n");
    WII_LOG(STATUS, "  motion-plus: %08x\n", motionPlus);
    WII_LOG(STATUS, "  other      : %08x\n", other);

    // Clear motion-plus and extension states
    std::memset(&state.extension, 0, sizeof(state.extension));

    unsigned Id0 = Read16(buf + 0);
    unsigned Id1 = Read32(buf + 2);

    switch (Id1)
    {
    case 0xA4200000:
        state.extension.type = Extension::Nunchuk;
        break;
    case 0xA4200101:
        state.extension.type = Extension::ClassicController;
        break;
    case 0xA4200405:
        state.extension.type = Extension::MotionPlus;
        break;
    case 0xA4200505:
        state.extension.type = Extension::MotionPlus | Extension::Nunchuk;
        break;
    case 0xA4200705:
        state.extension.type = Extension::MotionPlus | Extension::ClassicController;
        break;
    default:
        state.extension.type = 0;
        break;
    }

    motionPlus = state.extension.type &  Extension::MotionPlus;
    other      = state.extension.type & ~Extension::MotionPlus;

    WII_LOG(STATUS, "  Extension detected: %08x [ID: %04x %08x]\n", state.extension.type, Id0, Id1);

    if (motionPlus)
    {
        WII_LOG(STATUS, "  Motion-plus activated.\n");

        state.extension.motionPlus.status = WII_STATUS_MP_ACTIVE;
    }
    else
    {
        WII_LOG(STATUS, "  Motion-plus deactivated.\n");

        state.extension.motionPlus.status = WII_STATUS_MP_INACTIVE;
    }

    if (motionPlus)
    {
        WII_LOG(STATUS, "  Read motion-plus calibration data...\n");

        ReadMotionPlusCalibrationData();
    }

    if (other)
    {
        WII_LOG(STATUS, "  Read extension calibration data...\n");

        // Extension connected.
        // Read extension calibration data.
        ReadExtensionCalibrationData();
    }

    return true;
}

bool Wiimote::Impl::ParseMotionPlusIdentifier(uint8_t const* buf, unsigned /*len*/, unsigned error)
{
    assert(state.extension.motionPlus.status == WII_STATUS_MP_STARTUP);

    WII_LOG(MP, "Motion Plus identifier read.\n");

    if (error)
    {
        WII_LOG(MP, "  Motion Plus not present\n");

        //
        // Reading the two bytes at 0x04A600FE fails if no MotionPlus is present
        // or if the MotionPlus is already enabled
        //

        state.extension.motionPlus.status = WII_STATUS_MP_NOT_PRESENT;

        return true;
    }

    unsigned ID = Read16(buf);

    if (ID == 0x0005)
    {
        WII_LOG(MP, "  Inactive Wii Motion Plus\n");

        //
        // The extension is initialised by writing 0x55 to 0x(4)a600f0. But there is no
        // need to write 00 to 0x(4)a500fb, since Wii games don't do that.
        //
        // Writing 0x04 to 0x(4)A600FE activates the MotionPlus as the "active" extension.
        // This does 3 things (with no additional initialization):
        //
        //  1.  A status report (0x20) will automatically be sent indicating that a normal
        //      extension has been plugged in, if and only if there was no extension plugged
        //      into the MotionPlus pass-through extension port.
        //  2.  The standard extension identifier at 0x(4)A400FA now reads 00 00 A4 20 04 05
        //  3.  Extension reports now contain MotionPlus data.
        //

        EnableMotionPlus();

        return true;
    }

    if (ID == 0x0405 || ID == 0x0505 || ID == 0x0705)
    {
        if (ID == 0x0405)
            WII_LOG(MP, "  No-longer active Wii Motion Plus\n");
        if (ID == 0x0505)
            WII_LOG(MP, "  No-longer nunchuk-passthrough Wii Motion Plus\n");
        if (ID == 0x0705)
            WII_LOG(MP, "  No-longer classic-passthrough Wii Motion Plus\n");

        //
        // Writing 0x55 to 0x(4)A400F0 deactivates the MotionPlus, and activates the Nunchuk
        // or other extension plugged into the back of the Motion Plus. It does these things
        // (with no additional initialization):
        //
        //	1.	A status report (0x20) will always be sent indicating that a normal extension
        //		has been unplugged.
        //	2.	The Motion Plus extension identifier at 0x(4)A600FA now reads 00 00 A6 20 04 05
        //	3.	Another status report (0x20) will always be sent indicating that a normal
        //		extension has been plugged in.
        //	4.	The standard extension block at 0x(4)A40000 now reads from the Nunchuk (or
        //		other extension)
        //	5.	Extension reports no longer contain MotionPlus data
        //

        state.extension.motionPlus.status = WII_STATUS_MP_NO_LONGER_ACTIVE;

        DisableMotionPlus();

        return true;
    }

    WII_LOG(MP, "  Unknown id: %08x\n", ID);

    return false;
}

bool Wiimote::Impl::EnableIR(IRData::Mode mode, IRData::Sensitivity sensitivity)
{
    struct IRBlock {
        uint8_t block1[9];
        uint8_t block2[2];
    };

    static const IRBlock blocks[] = {
        {{ 0x02,0x00,0x00,0x71,0x01,0x00,0x64,0x00,0xFE }, { 0xFD,0x05 }}, // Level 1
        {{ 0x02,0x00,0x00,0x71,0x01,0x00,0x96,0x00,0xB4 }, { 0xB3,0x04 }}, // Level 2
        {{ 0x02,0x00,0x00,0x71,0x01,0x00,0xAA,0x00,0x64 }, { 0x63,0x03 }}, // Level 3
        {{ 0x02,0x00,0x00,0x71,0x01,0x00,0xC8,0x00,0x36 }, { 0x35,0x03 }}, // Level 4
        {{ 0x02,0x00,0x00,0x71,0x01,0x00,0x72,0x00,0x20 }, { 0x1F,0x03 }}, // Level 5
    };

    // Is it really required?
    if (state.ir.mode == mode && state.ir.sensitivity == sensitivity)
        return true;

    state.ir.mode = mode;
    state.ir.sensitivity = sensitivity;

    //
    // The following procedure should be followed to turn on the IR Camera:
    //
    // 1. Enable IR Camera (Send 0x04 to Output Report 0x13)
    // 2. Enable IR Camera 2 (Send 0x04 to Output Report 0x1a)
    // 3. Write 0x08 to register 0xb00030
    // 4. Write Sensitivity Block 1 to registers at 0xb00000
    // 5. Write Sensitivity Block 2 to registers at 0xb0001a
    // 6. Write Mode Number to register 0xb00033
    // 7. Write 0x08 to register 0xb00030 (again)
    //

    SendReport(WII_OUTPUT_ENABLE_IR_1, 0x04);
    SendReport(WII_OUTPUT_ENABLE_IR_2, 0x04);

    auto&& block = blocks[static_cast<int>(sensitivity)];

    WriteData(0x04B00030, 0x08);
    WriteData(0x04B00000, block.block1, sizeof(block.block1));
    WriteData(0x04B0001A, block.block2, sizeof(block.block2));
    WriteData(0x04B00033, static_cast<uint8_t>(mode));
    WriteData(0x04B00030, 0x08);

    return true;
}

bool Wiimote::Impl::DisableIR()
{
    if (state.ir.mode == IRData::Mode::Off)
        return true;

    state.ir.mode = IRData::Mode::Off;

    SendReport(WII_OUTPUT_ENABLE_IR_1, 0x00);
    SendReport(WII_OUTPUT_ENABLE_IR_2, 0x00);

    return true;
}

bool Wiimote::Impl::EnableMotionPlus()
{
    WII_LOG(MP, "Enable motion-plus...\n");

    //
    // The extension is initialised by writing 0x55 to 0x(4)a600f0. But there is no
    // need to write 00 to 0x(4)a500fb, since Wii games don't do that.
    //
    // Writing 0x04 to 0x(4)A600FE activates the MotionPlus as the "active" extension.
    // This does 3 things (with no additional initialization):
    //
    //  1.  A status report (0x20) will automatically be sent indicating that a normal
    //      extension has been plugged in, if and only if there was no extension plugged
    //      into the MotionPlus pass-through extension port.
    //  2.  The standard extension identifier at 0x(4)A400FA now reads 00 00 A4 20 04 05
    //  3.  Extension reports now contain MotionPlus data.
    //

#if 0
    using namespace std::placeholders;
    ReadData(0x04A60000, 0x100, std::bind(&ParseMotionPlusCalibrationData, std::ref(state), _1, _2, _3));
#endif

    WriteData(0x04A600F0, 0x55);

#if 0
    using namespace std::placeholders;
    ReadData(0x04A60000, 0x100, std::bind(&ParseMotionPlusCalibrationData, std::ref(state), _1, _2, _3));
#endif

    switch (state.extension.type)
    {
    case Extension::Nunchuk:
        WriteData(0x04A600FE, 0x05);
        break;
    case Extension::ClassicController:
        WriteData(0x04A600FE, 0x07);
        break;
    default:
        WriteData(0x04A600FE, 0x04);
        break;
    }

#if 0
    using namespace std::placeholders;
    ReadData(0x04A60000, 0x100, std::bind(&ParseMotionPlusCalibrationData, std::ref(state), _1, _2, _3));
#endif

    // A status report is not always sent.
    // Explicitly request one.
    PushRequest(Request::Type::Status);

    return true;
}

bool Wiimote::Impl::DisableMotionPlus()
{
    WII_LOG(MP, "Disable motion-plus...\n");

    //
    // Writing 0x55 to 0x(4)A400F0 deactivates the MotionPlus, and activates the Nunchuk
    // or other extension plugged into the back of the Motion Plus. It does these things
    // (with no additional initialization):
    //
    //	1.	A status report (0x20) will always be sent indicating that a normal extension
    //		has been unplugged.
    //	2.	The Motion Plus extension identifier at 0x(4)A600FA now reads 00 00 A6 20 04 05
    //	3.	Another status report (0x20) will always be sent indicating that a normal
    //		extension has been plugged in.
    //	4.	The standard extension block at 0x(4)A40000 now reads from the Nunchuk (or
    //		other extension)
    //	5.	Extension reports no longer contain MotionPlus data
    //

    WriteData(0x04A400F0, 0x55);
    WriteData(0x04A600FE, 0x00);

    return true;
}

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------

void Wiimote::Impl::ReadCalibrationData()
{
    using namespace std::placeholders;

    ReadData(0x00000016, 8, std::bind(&ParseCalibrationData, std::ref(state), _1, _2, _3));
}

void Wiimote::Impl::ReadExtensionCalibrationData()
{
    using namespace std::placeholders;

    ReadData(0x04A40020, 16, std::bind(&ParseExtensionCalibrationData, std::ref(state), _1, _2, _3));
}

void Wiimote::Impl::ReadMotionPlusCalibrationData()
{
    using namespace std::placeholders;

#if 0
    ReadData(0x04A60000, 0x100, std::bind(&ParseMotionPlusCalibrationData, std::ref(state), _1, _2, _3));
#else
//    ReadData(0x04A40020, 32, std::bind(&ParseMotionPlusCalibrationData, std::ref(state), _1, _2, _3));
    ReadData(0x04A40000, 0x100, std::bind(&ParseMotionPlusCalibrationData, std::ref(state), _1, _2, _3));
#endif
}

void Wiimote::Impl::ReadExtensionIdentifier()
{
    using namespace std::placeholders;

    ReadData(0x04A400FA, 6, std::bind(&Impl::ParseExtensionIdentifier, this, _1, _2, _3));
}

void Wiimote::Impl::ReadMotionPlusIdentifier()
{
    using namespace std::placeholders;

    WII_LOG(MP, "Reading motion-plus identifier...\n");

    //
    // The Wii Motion Plus is first identified by the 6 bytes: 00 00 A6 20 00 05
    // at register address 0x(4)a600fa (instead of 0x(4)a400fa like a regular extension).
    // Games attempt to detect the Wii Motion Plus by trying to read the two-byte expansion
    // identifier at 0xA600FE (they try up to 3 times, then wait 8 seconds, then check again).
    // If a Wii Motion Plus is not present, or it has already been activated, then the
    // attempt to read those bytes will fail with error 7.
    //

    state.extension.motionPlus.status = WII_STATUS_MP_STARTUP;

    ReadData(0x04A600FE, 2, std::bind(&Impl::ParseMotionPlusIdentifier, this, _1, _2, _3));
}

void Wiimote::Impl::InitExtension()
{
    //
    // The new way to initialize the extension is by writing 0x55 to 0x(4)A400F0,
    // then writing 0x00 to 0x(4)A400FB. It works on all extensions, and makes the
    // extension type bytes unencrypted.
    //
    WriteData(0x04A400F0, 0x55);
    WriteData(0x04A400FB, 0x00);
}

void Wiimote::Impl::InitMotionPlus()
{
    //
    // The extension is initialised by writing 0x55 to 0x(4)a600f0. But there is no
    // need to write 00 to 0x(4)a500fb, since Wii games don't do that.
    //
    WriteData(0x04A600F0, 0x55);
#if 0
    WriteData(0x04A600FB, 0x00);
#endif
}

//--------------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------------

bool Wiimote::Impl::SetLEDs(unsigned leds)
{
    state.leds = leds & 0xF0;

    return SendReport(WII_OUTPUT_LEDS, static_cast<unsigned char>(state.leds));
}

bool Wiimote::Impl::SetRumble(bool enable)
{
    state.rumble = enable;

    return SetLEDs(state.leds); // LED report also handles rumble
}

//--------------------------------------------------------------------------------------------------
// OS-specific implementation
//--------------------------------------------------------------------------------------------------

#ifdef _WIN32
#include "Windows/Wiimpl.inl"
#else
#include "Unix/Wiimpl.inl"
#endif
