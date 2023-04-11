/*!
* \file ecSlvApiDef_SoE.h
*
* \brief
* SoE defines.
*
* \author
* KUNBUS GmbH
*
* \date
* 2022-08-19
*
* \copyright
* Copyright (c) 2022, KUNBUS GmbH<br /><br />
* All rights reserved.<br />
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:<br />
* <ol>
* <li>Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.</li>
* <li>Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.</li>
* <li>Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.</li>
* </ol>
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#if !(defined __ECSLVAPIDEF_SOE_H__)
#define __ECSLVAPIDEF_SOE_H__		1



#if (defined __cplusplus)
extern "C" {
#endif


/*-----------------------------------------------------------------------------------------
------
------    Defines and Types
------
-----------------------------------------------------------------------------------------*/

/**
* \addtogroup SoE SoE Service and Flags
* @{
*/
/*---------------------------------------------
-    SOE services
-----------------------------------------------*/
#define    ECAT_SOE_OPCODE_RRQ      0x0001    /**< \brief SoE Read Request*/
#define    ECAT_SOE_OPCODE_RRS      0x0002    /**< \brief SoE Read Response*/
#define    ECAT_SOE_OPCODE_WRQ      0x0003    /**< \brief SoE Write Request*/
#define    ECAT_SOE_OPCODE_WRS      0x0004    /**< \brief SoE Write Response*/
#define    ECAT_SOE_OPCODE_NFC      0x0005    /**< \brief SoE Notification Request*/
#define    ECAT_SOE_OPCODE_EMCY     0x0006    /**< \brief SoE Emergency*/

/*---------------------------------------------
-    SOE flags
-----------------------------------------------*/
#define    SOEFLAGS_OPCODE          0x0007    /**< \brief SoE Flags*/
                                               /**<
                                               * 0 = unused<br>
                                               * 1 = readReq<br>
                                               * 2 = readRes<br>
                                               * 3 = writeReq<br>
                                               * 4 = writeRes<br>
                                               * 5 = notification (command changed notification)*/
#define    SOEFLAGS_INCOMPLETE      0x0008    /**< \brief more follows*/
#define    SOEFLAGS_ERROR           0x0010    /**< \brief an error word follows*/
#define    SOEFLAGS_DRIVENO         0x00E0    /**< \brief drive number*/

#define    SOEFLAGS_DATASTATE       0x0100    /**< \brief Data state follows or requested*/
#define    SOEFLAGS_NAME            0x0200    /**< \brief Name follows or requested*/
#define    SOEFLAGS_ATTRIBUTE       0x0400    /**< \brief Attribute follows or requested*/
#define    SOEFLAGS_UNIT            0x0800    /**< \brief Unit follows or requested*/
#define    SOEFLAGS_MIN             0x1000    /**< \brief Min value follows or requested*/
#define    SOEFLAGS_MAX             0x2000    /**< \brief Max value follows or requested*/
#define    SOEFLAGS_VALUE           0x4000    /**< \brief Value follows or requested*/
#define    SOEFLAGS_DEFAULT         0x8000    /**< \brief Default value*/
/** @}*/


#if (defined __cplusplus)
}
#endif

#endif /* __ECSLVAPIDEF_SOE_H__ */
