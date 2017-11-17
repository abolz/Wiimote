// This file is distributed under the MIT license.
// See the LICENSE file for details.

#include <cstdio>

#include <mmsystem.h>

extern "C"
{
#include <hidsdi.h>
#include <setupapi.h>
}

#ifdef __MINGW32__
extern "C" BOOLEAN __stdcall HidD_SetOutputReport(HANDLE HidDeviceObject, PVOID ReportBuffer, ULONG ReportBufferLength);
#endif

void Wiimote::Impl::Init()
{
    overlapped.Internal     = 0;
    overlapped.InternalHigh = 0;
    overlapped.Offset       = 0;
    overlapped.OffsetHigh   = 0;
    overlapped.Pointer      = 0;
    overlapped.hEvent       = CreateEvent(0, TRUE, FALSE, TEXT(""));

    if (overlapped.hEvent == NULL)
    {
    }

    timeBeginPeriod(1);
}

void Wiimote::Impl::Finish()
{
    timeEndPeriod(1);

    CloseHandle(overlapped.hEvent);
}

void Wiimote::Impl::Disconnect()
{
    if (device != INVALID_HANDLE_VALUE)
    {
        CloseHandle(device);

        device = INVALID_HANDLE_VALUE;
    }

    status = WII_STATUS_DISCONNECTED;
}

#if 0
#define WII_RESET_EVENT() do { ResetEvent(overlapped.hEvent); } while (::wii::details::false_())
#else
#define WII_RESET_EVENT() do { } while (::wii::details::false_())
#endif

bool Wiimote::Impl::GetInputReport(uint8_t* report)
{
    assert( device != INVALID_HANDLE_VALUE );

    //
    // NOTE:
    //
    // This does actually NOT perform an async read
    // The overlapped read is only used for performance reasons and this routine waits
    // for the read to complete.
    // If the read does not finish within TIMEOUT ms, it's assumed the read failed
    //

    if (ReadFile(device, report, WII_REPORT_LENGTH, 0, &overlapped))
    {
        WII_RESET_EVENT();
        return true;
    }

    //
    // Error occured or read incomplete.
    // Figure out
    //

    if (GetLastError() != ERROR_IO_PENDING)
    {
        WII_RESET_EVENT();
        return false;
    }

    //
    // Incomplete read.
    // Wait for read operation to finish
    //

    DWORD waitResult = WaitForSingleObject(overlapped.hEvent, continous ? 1000 : INFINITE);
    DWORD transferred = 0;

    switch (waitResult)
    {
    case WAIT_OBJECT_0:
        if (GetOverlappedResult(device, &overlapped, &transferred, FALSE))
        {
            assert(transferred == WII_REPORT_LENGTH);

            WII_RESET_EVENT();
            return true;
        }
        break;

    case WAIT_TIMEOUT:
        //
        // Wait timed-out
        // Connection lost!
        //
        break;

    case WAIT_FAILED:
    default:
        //
        // Wait failed
        // This should never happen as it indicates a severe Windows internal error
        //
        break;
    }

    WII_LOG(IO, "Read failed.\n");

    //
    // Cancel read request
    //
    CancelIo(device);

    WII_RESET_EVENT();

    return false;
}

bool Wiimote::Impl::SetOutputReport(uint8_t const* report, unsigned len)
{
    assert( device != INVALID_HANDLE_VALUE );

    for (unsigned n = 0; n < 10; ++n)
    {
        if (HidD_SetOutputReport(device, (PVOID)report, len))
        {
            return true;
        }
    }

    WII_LOG(IO, "Write failed.\n");

    return false;
}

double Wiimote::Impl::Time()
{
    return timeGetTime() / 1000.0;
}

HANDLE Wiimote::Impl::OpenDeviceHandle(LPCTSTR devicePath)
{
    //
    // Open a read/write handle to our device using the DevicePath returned
    //

    HANDLE handle = CreateFile( devicePath,
                                GENERIC_READ | GENERIC_WRITE,
                                FILE_SHARE_READ | FILE_SHARE_WRITE,
                                NULL,
                                OPEN_EXISTING,
                                FILE_FLAG_OVERLAPPED,
                                NULL );

    if (handle != INVALID_HANDLE_VALUE)
    {
        //
        // Get the attributes of the current device
        // If the vendor and product IDs match up it's a Wiimote
        //

        HIDD_ATTRIBUTES attrib;

        memset(&attrib, 0, sizeof(attrib));

        attrib.Size = sizeof(attrib);

        if (HidD_GetAttributes(handle, &attrib))
        {
            bool vendorOK = attrib.VendorID == WII_VENDOR_ID;
            bool productOK = attrib.ProductID == WII_PRODUCT_ID || attrib.ProductID == WII_PRODUCT_ID_2;

            if (vendorOK && productOK)
            {
                return handle;
            }
        }

        CloseHandle(handle);
    }

    return INVALID_HANDLE_VALUE;
}

bool Wiimote::Impl::Connect(Wiimote* wiimotes, unsigned& count)
{
    // Parameter validation
    assert( wiimotes && count > 0 );

    GUID guid;

    HidD_GetHidGuid(&guid);

    //
    // Get a handle to all devices that are part of the HID class
    //

    HANDLE hDevInfo = SetupDiGetClassDevs(&guid, 0, 0, DIGCF_DEVICEINTERFACE | DIGCF_PRESENT);

    if (hDevInfo == INVALID_HANDLE_VALUE)
    {
        return false;
    }

    SP_DEVICE_INTERFACE_DATA diData;

    memset(&diData, 0, sizeof(diData));

    diData.cbSize = sizeof(diData);

    //
    // Enumerate all devices
    //

    unsigned connected = 0;

    for (DWORD index = 0; connected < count; index++)
    {
        if (!SetupDiEnumDeviceInterfaces(hDevInfo, 0, &guid, index, &diData))
        {
            break;
        }

        //
        // Get the required size of the DeviceInterfaceDetailData buffer
        // This size includes the size of the fixed part of the structure plus the number of bytes
        // required for the variable-length device path string
        //

        DWORD cbSize = 0;

        SetupDiGetDeviceInterfaceDetail(hDevInfo, &diData, 0, 0, &cbSize, 0);

        if (cbSize == 0)
        {
            break;
        }

        // Allocate storage for the buffer
        std::vector<BYTE> pdiDetailBuffer(cbSize);

        PSP_DEVICE_INTERFACE_DETAIL_DATA pdiDetail = (PSP_DEVICE_INTERFACE_DETAIL_DATA)&pdiDetailBuffer[0];

        pdiDetail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);
        pdiDetail->DevicePath[0] = 0;

        if (SetupDiGetDeviceInterfaceDetail(hDevInfo, &diData, pdiDetail, cbSize, &cbSize, 0))
        {
            HANDLE handle = OpenDeviceHandle(pdiDetail->DevicePath);

            if (handle != INVALID_HANDLE_VALUE)
            {
                wiimotes[connected].impl->device = handle;
                wiimotes[connected].impl->status = WII_STATUS_CONNECTED;

                connected++;
            }
        }
    }

    SetupDiDestroyDeviceInfoList(hDevInfo);

    count = connected;

    return true;
}
