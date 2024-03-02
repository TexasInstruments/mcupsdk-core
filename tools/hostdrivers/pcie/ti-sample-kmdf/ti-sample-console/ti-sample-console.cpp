/*
 *  Copyright (C) 2023-2024 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <format>

#include <Windows.h>

#include <Public.h>

int main()
{
    HANDLE hDevice;
    BOOL   bRc;
    _TISAMPLEKMDF_TEST_DMA *DmaInputBuffer;
    _TISAMPLEKMDF_TEST_DMA *DmaOutputBuffer;
    TISAMPLEKMDF_TEST_MSI MsiOutputBuffer;
    TISAMPLEKMDF_TEST_BARS BarOutputBuffer;
    DWORD  bytesReturned;

    std::cout << "--------------------------------------------------------------------------------" << std::endl;
    std::cout << "Starting PCIe RC KMDF test application" << std::endl;
    std::cout << "--------------------------------------------------------------------------------" << std::endl;

    std::cout << "Opening windows kernel mode driver \\\\.\\SampleTI" << std::endl;

    /* open the file handle to the kernel mode driver */
    hDevice = CreateFile(L"\\\\.\\SampleTI",
        GENERIC_READ | GENERIC_WRITE,    // requested access
        0,                             // share mode
        NULL,                          // security attributes
        OPEN_EXISTING,                 // create disposition
        0,                             // flags
        NULL);                         // template file

    if (hDevice == INVALID_HANDLE_VALUE)
    {
        std::cout << "Failed to open device" << std::endl;
        return -1;
    }

    std::cout << "--------------------------------------------------------------------------------" << std::endl;
    std::cout << "Start COPY test" << std::endl;

    /* the DMA test requires a larger buffer and is thus allocated on the heap */
    DmaInputBuffer  = (_TISAMPLEKMDF_TEST_DMA *) malloc(sizeof(_TISAMPLEKMDF_TEST_DMA));
    DmaOutputBuffer = (_TISAMPLEKMDF_TEST_DMA *) malloc(sizeof(_TISAMPLEKMDF_TEST_DMA));

    if (!DmaInputBuffer || !DmaOutputBuffer)
    {
        std::cout << "Failed to allocate buffers for device I/O" << std::endl;
        return -1;
    }

    memset(DmaOutputBuffer->result, 0, sizeof(DmaOutputBuffer->result));
    memset(DmaInputBuffer->result, 0, sizeof(DmaInputBuffer->result));
    DmaInputBuffer->pattern = 0xcafef00d;

    /* call the kernel mode driver to perform the DMA test:
     * - kernel mode driver fills the EP's BAR0 with 'pattern'
     * - kernel mode driver triggers a copy operation in the EP
     * - EP copies data from BAR0 to the DMA buffer within the kernel mode driver
     * - EP signals completion via MSI
     * - kernel mode driver copies data back to user space
     */
    bRc = DeviceIoControl(hDevice,
        (DWORD)IOCTL_TISAMPLEKMDF_TEST_DMA,
        DmaInputBuffer,
        sizeof (*DmaInputBuffer),
        DmaOutputBuffer,
        sizeof(*DmaOutputBuffer),
        &bytesReturned,
        NULL
    );

    if (bRc == TRUE)
    {
        std::cout << "IOCTL_TISAMPLEKMDF_TEST_DMA returned data, verifying..." << std::endl;

        for (int i = 0; i < sizeof(DmaOutputBuffer->result) / sizeof(ULONG); i++)
        {
            /* if the test was successful our buffer should be filled with 'pattern' */
            if (DmaOutputBuffer->result[i] != DmaInputBuffer->pattern)
                std::cout << "Receieved data in DMA buffer at offset " <<
                std::format("{:08x}", i) <<
                " doesn't match pattern: " << std::format("{:08x}", DmaOutputBuffer->result[i]) <<
                " != " << std::format("{:08x}", DmaInputBuffer->pattern) << std::endl;
        }

        std::cout << "COPY test passed" << std::endl;
    }
    else
    { 
        std::cout << "IOCTL_TISAMPLEKMDF_TEST_DMA returned no data - ERROR" << std::endl;
    }

    free(DmaInputBuffer);
    free(DmaOutputBuffer);

    std::cout << "--------------------------------------------------------------------------------" << std::endl;
    std::cout << "Start MSI test" << std::endl;

    memset(&MsiOutputBuffer, 0, sizeof(MsiOutputBuffer));

    /* call the kernel mode driver to perform the MSI test:
     * - kernel mode driver triggers MSI test in the EP
     * - EP triggers every MSI allocated to the device (multi message enable)
     * - kernel mode driver waits for all MSIs assigned to the device
     *   to be triggered
     * - bitfield of MSIs receieved is returned to user space
     */
    bRc = DeviceIoControl(hDevice,
        (DWORD)IOCTL_TISAMPLEKMDF_TEST_MSI,
        NULL,
        0,
        &MsiOutputBuffer,
        sizeof(MsiOutputBuffer),
        &bytesReturned,
        NULL
    );

    if (bRc == TRUE)
    {
        std::cout << "IOCTL_TISAMPLEKMDF_TEST_MSI returned, result: "
            << std::format("{:08x}", MsiOutputBuffer.result) << std::endl;
        std::cout << "MSI test passed" << std::endl;
    }
    else
    {
        std::cout << "IOCTL_TISAMPLEKMDF_TEST_MSI returned no data - ERROR" << std::endl;
    }

    std::cout << "--------------------------------------------------------------------------------" << std::endl;
    std::cout << "Start Bar1/2 test" << std::endl;

    memset(&BarOutputBuffer, 0, sizeof(BarOutputBuffer));

    /* call the kernel mode driver to perform the BARs test:
     * - kernel mode fills BAR1 with increasing pattern (0, 1, 2, ...)
     * - kernel mode fills BAR2 with binary inverse of pattern (ffffffff, fffffffe, ...)
     * - kernel mode driver triggers BAR test in the EP
     * - EP verifies contents of BAR1 and BAR2 (known pattern)
     * - EP signals completion via MSI
     * - kernel mode returns '1' in case of success
     */
    bRc = DeviceIoControl(hDevice,
        (DWORD)IOCTL_TISAMPLEKMDF_TEST_BARS,
        NULL,
        0,
        &BarOutputBuffer,
        sizeof(BarOutputBuffer),
        &bytesReturned,
        NULL
    );

    if (bRc == TRUE)
    {
        std::cout << "IOCTL_TISAMPLEKMDF_TEST_BARS returned, result: "
            << std::format("{:08x}", BarOutputBuffer.result) << std::endl;
        std::cout << "BAR test passed" << std::endl;
    }
    else
    {
        std::cout << "IOCTL_TISAMPLEKMDF_TEST_BARS returned no data - ERROR" << std::endl;
    }

    std::cout << "--------------------------------------------------------------------------------" << std::endl;
    std::cout << "Closing windows kernel mode driver" << std::endl;

    CloseHandle(hDevice);

    std::cout << "KMDF test application done" << std::endl;
    std::cout << "--------------------------------------------------------------------------------" << std::endl;

    return 0;
}
