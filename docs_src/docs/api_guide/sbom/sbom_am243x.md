# Software Bill of Materials {#SBOM}

<table>
    <tr>
        <th>Component type</th>
        <th>Component Name</th>
        <th>Production or Reference</th>
        <th>Process compliance (FOSS, BQ, ASPICE, FSQ, CSQ)</th>
        <th>Certification </th>
        <th>Distribution</th>
        <th>Comments</th>
    </tr>
    <tr>
        <td>Firmware</td>
        <td>
            DMSC Firmware
            <em>
            <br/>${SDK_INSTALL_PATH}/source/drivers/sciclient/soc/am64x_am243x/*bin
            </em>
        </td>
        <td>Production</td>
        <td>BQ</td>
        <td>NA</td>
        <td>TI.com</td>
        <td>Required for DM and security TISCI services</td>
    </tr>
    <tr>
        <td>Libraries/ Drivers</td>
        <td>
            Drivers (I2C, SPI, UART, CPSW, etc)
            <em>
            <br/>${SDK_INSTALL_PATH}/source/drivers
            <br/>${SDK_INSTALL_PATH}/source/networking/enet/core
            <br/>${SDK_INSTALL_PATH}/source/networking/enet/icss_*
            <br/>${SDK_INSTALL_PATH}/source/pru_io
            <br/>${SDK_INSTALL_PATH}/source/security/security_common/drivers
            </em>
        </td>
        <td>Production</td>
        <td>BQ</td>
        <td>NA</td>
        <td>TI.com</td>
        <td>Drivers for various IPs in the SOC.</td>
    </tr>
    <tr>
        <td>Libraries/ Drivers</td>
        <td>
            Board Drivers
            <em>
            <br/>${SDK_INSTALL_PATH}/source/board
            </em>
        </td>
        <td>Reference</td>
        <td>Demo quality</td>
        <td>NA</td>
        <td>TI.com</td>
        <td>Board peripheral drivers like ETHPHY, EEPROM etc</td>
    </tr>
    <tr>
        <td>Examples</td>
        <td>
            Driver, Kernel Examples                       
            <em>
            <br/>${SDK_INSTALL_PATH}/examples
            <br/>${SDK_INSTALL_PATH}/source/networking/enet/core/examples
            </em>
        </td>
        <td>Reference</td>
        <td>Demo quality</td>
        <td>NA</td>
        <td>TI.com</td>
        <td>Application examples to show how to use the driver APIs</td>
    </tr>
    <tr>
        <td>Examples</td>
        <td>
            Secondary bootloader (SBL)
            <em>
            <br/>${SDK_INSTALL_PATH}/examples/drivers/boot
            </em>
        </td>
        <td>Reference</td>
        <td>Demo quality</td>
        <td>NA</td>
        <td>TI.com</td>
        <td>Secondary bootloader application examples</td>
    </tr>
    <tr>
        <td>Libraries/ Drivers</td>
        <td>
            Safety diagnostics library (SDL)
            <em>
            <br/>${SDK_INSTALL_PATH}/source/sdl
            </em>
        </td>
        <td>Production</td>
        <td>FSQ</td>
        <td>TUV Certification</td>
        <td>TI.com</td>
        <td>Interfaces to safety mechanisms and features in the library</td>
    </tr>
    <tr>
        <td>Libraries</td>
        <td>
            CMSIS
            <em>
            <br/>${SDK_INSTALL_PATH}/source/cmsis            
            </em>
        </td>
        <td>Reference</td>
        <td>FOSS†</td>
        <td>NA</td>
        <td>TI.com</td>
        <td>Common Microcontroller Software Interface Standard</td>
    </tr>
    <tr>
        <td>Libraries</td>
        <td>
            Math Library
            <em>
            <br/>${SDK_INSTALL_PATH}/source/mathlib            
            </em>
        </td>
        <td>Reference</td>
        <td>FOSS†</td>
        <td>NA</td>
        <td>TI.com</td>
        <td>Library for math functions</td>
    </tr>
    <tr>
        <td>Libraries</td>
        <td>
            Dhrystone Benchmark
            <em>
            <br/>${SDK_INSTALL_PATH}/source/dhrystone_benchmark
            </em>
        </td>
        <td>Reference</td>
        <td>FOSS†</td>
        <td>NA</td>
        <td>TI.com</td>
        <td>Dhrystone CPU benchmarking for A53 core</td>
    </tr>
    <tr>
        <td>OS</td>
        <td>
            FreeRTOS
            <em>
            <br/>${SDK_INSTALL_PATH}/source/kernel/freertos/FreeRTOS-Kernel
            </em>
        </td>
        <td>Reference</td>
        <td>FOSS†</td>
        <td>NA</td>
        <td>TI.com</td>
        <td>Reference OS for ARM R5F</td>
    </tr>
    <tr>
        <td>OS Abstraction</td>
        <td>
            Driver Porting Layer (DPL)
            <em>
            <br/>${SDK_INSTALL_PATH}/source/kernel/freertos/dpl
            <br/>${SDK_INSTALL_PATH}/source/kernel/nortos/dpl
            </em>
        </td>
        <td>Production</td>
        <td>BQ</td>
        <td>NA</td>
        <td>TI.com</td>
        <td>Abstraction layer between FreeRTOS and NoRTOS</td>
    </tr>
    <tr>
        <td>Stack/ Middleware</td>
        <td>
            LWIP
            <em>
            <br/>${SDK_INSTALL_PATH}/source/networking/lwip
            </em>
        </td>
        <td>Reference</td>
        <td>FOSS†</td>
        <td>NA</td>
        <td>TI.com</td>
        <td>Light weight open source TCP/IP Stack</td>
    </tr>
    <tr>
        <td>Stack/ Middleware</td>
        <td>
            MbedTLS
            <em>
            <br/>${SDK_INSTALL_PATH}/source/networking/mbedtls_library
            </em>            
        </td>
        <td>Reference</td>
        <td>FOSS†</td>
        <td>NA</td>
        <td>TI.com</td>
        <td>SSL/TLS and DTLS protocols</td>
    </tr>
    <tr>
        <td>Stack/ Middleware</td>
        <td>
            TSN
            <em>
            <br/>${SDK_INSTALL_PATH}/source/networking/tsn
            </em>
        </td>
        <td>Production</td>
        <td>BQ</td>
        <td>NA</td>
        <td>TI.com</td>
        <td>Ethernet Time Sensitive Networking Stack</td>
    </tr>
    <tr>
        <td>Stack/ Middleware</td>
        <td>
            FreeRTOS FAT FS
            <em>
            <br/>${SDK_INSTALL_PATH}/source/fs/freertos_fat
            </em>
        </td>
        <td>Reference</td>
        <td>FOSS†</td>
        <td>NA</td>
        <td>TI.com</td>
        <td>Open source FAT file system</td>
    </tr>
    <tr>
        <td>Stack/ Middleware</td>
        <td>
            Little FS            
            <em>
            <br/>${SDK_INSTALL_PATH}/source/fs/littlefs
            </em>
        </td>
        <td>Reference</td>
        <td>FOSS†</td>
        <td>NA</td>
        <td>TI.com</td>
        <td>Open source flash file system</td>
    </tr>    
    <tr>
        <td>Tools</td>
        <td>
            SDK Tools
            <em>
            <br/>${SDK_INSTALL_PATH}/tools
            <br/>${SDK_INSTALL_PATH}/source/networking/enet/core/tools
            <br/>${SDK_INSTALL_PATH}/source/security/security_common/tools
            </em>
        </td>
        <td>Reference</td>
        <td>Demo quality</td>
        <td>NA</td>
        <td>TI.com</td>
        <td>Tools for Appimage creation, Flashing, signing, etc.</td>
    </tr>
    <tr>
        <td>Examples</td>
        <td>OTP Key Writer</td>
        <td>Reference</td>
        <td>BQ</td>
        <td>NA</td>
        <td>mysecuresw</td>
        <td>Reference tool to program the keys</td>
    </tr>
    <tr>
        <td>Libraries/ Drivers</td>
        <td>ARM Clang RTS library</td>
        <td>Production</td>
        <td>FOSS, BQ</td>
        <td>NA</td>
        <td>TI.com</td>
        <td>Complier libraries for ARM cores</td>
    </tr>
    <tr>
        <td>Tools</td>
        <td>ARM CLANG compiler</td>
        <td>Production</td>
        <td>BQ</td>
        <td>QKIT</td>
        <td>TI.com</td>
        <td>Compiler tool for ARM R5F and M4F</td>
    </tr>
    <tr>
        <td>Tools</td>
        <td>SYSCONFIG</td>
        <td>Reference</td>
        <td>BQ</td>
        <td>NA</td>
        <td>Ti.com</td>
        <td>Tool to configure SOC pinmux and driver parameters</td>
    </tr>
</table>

† To be used for production based on customer judgement
