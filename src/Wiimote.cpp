// This file is distributed under the MIT license.
// See the LICENSE file for details.

#include "Wiimote/Wiimote.h"

#include "Wiimpl.h"

using namespace wii;

Wiimote::Wiimote()
    : impl(new Impl)
{
}

Wiimote::~Wiimote()
{
}

bool Wiimote::Connect()
{
    unsigned count = 1;

    return impl->Connect(this, count) && count == 1;
}

bool Wiimote::SetReportMode(ReportMode mode, bool continous)
{
    return SetReportMode(mode, IRData::Sensitivity::Level3, continous);
}

bool Wiimote::SetReportMode(ReportMode mode, IRData::Sensitivity sensitivity, bool continous)
{
    return impl->SetReportMode(mode, sensitivity, continous);
}

bool Wiimote::SetLEDs(unsigned leds)
{
    return impl->SetLEDs(leds & 0xF0);
}

bool Wiimote::SetLEDs(bool led1, bool led2, bool led3, bool led4)
{
    unsigned leds = 0;

    if (led1)
        leds |= State::LED1;
    if (led2)
        leds |= State::LED2;
    if (led3)
        leds |= State::LED3;
    if (led4)
        leds |= State::LED4;

    return SetLEDs(leds);
}

bool Wiimote::SetRumble(bool enable)
{
    return impl->SetRumble(enable);
}

bool Wiimote::Poll()
{
    return impl->Poll();
}

bool Wiimote::Shutdown()
{
    return impl->Shutdown();
}

bool Wiimote::Disconnect()
{
    impl->Disconnect();
    return true;
}

bool Wiimote::CheckForMotionPlus()
{
    impl->ReadMotionPlusIdentifier();
    return true;
}

bool Wiimote::DisableMotionPlus()
{
    impl->DisableMotionPlus();
    return true;
}

State const& Wiimote::GetState() const
{
    return impl->state;
}
