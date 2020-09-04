/*!
* \file ecSlvApiDef_AoE.h
*
* \brief
* AoE defines.
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

#if !(defined __ECSLVAPIDEF_AOE_H__)
#define __ECSLVAPIDEF_AOE_H__		1


#if (defined __cplusplus)
extern "C" {
#endif


/*-----------------------------------------------------------------------------------------
------
------    Defines and Types
------
-----------------------------------------------------------------------------------------*/


/*---------------------------------------------
-    ADS Service Ids
-----------------------------------------------*/
#define     ADSSRVID_READ                       0x02                    /**< \brief ADS read service*/
#define     ADSSRVID_WRITE                      0x03                    /**< \brief ADS write service*/
#define     ADSSRVID_READWRITE                  0x09                    /**< \brief ADS read/write service*/
#define     ROUTERCMD_FRAGMENT                  0x902                   /**< \brief Fragmented ADS service*/

/*---------------------------------------------
-    ADS error codes
-----------------------------------------------*/
#define     ERR_ADSERRS                         0x700                   /**< \brief ADS Device Error*/

#define     ERR_NOERROR                         0x00                    /**< \brief No ADS Error*/
#define     ERR_TARGETPORTNOTFOUND              0x06                    /**< \brief Target port not found*/
#define     ERR_TARGETMACHINENOTFOUND           0x07                    /**< \brief Target machine not found*/

#define     ADSERR_DEVICE_ERROR                 (0x00+(ERR_ADSERRS))    /**< \brief Error class < device error >*/
#define     ADSERR_DEVICE_SRVNOTSUPP            (0x01+(ERR_ADSERRS))    /**< \brief Service is not supported by server*/
#define     ADSERR_DEVICE_INVALIDGRP            (0x02+(ERR_ADSERRS))    /**< \brief Invalid indexGroup*/
#define     ADSERR_DEVICE_INVALIDOFFSET         (0x03+(ERR_ADSERRS))    /**< \brief Invalid indexOffset*/
#define     ADSERR_DEVICE_INVALIDACCESS         (0x04+(ERR_ADSERRS))    /**< \brief Reading/writing not permitted*/
#define     ADSERR_DEVICE_INVALIDSIZE           (0x05+(ERR_ADSERRS))    /**< \brief Parameter size not correct*/
#define     ADSERR_DEVICE_INVALIDDATA           (0x06+(ERR_ADSERRS))    /**< \brief Invalid parameter value(s)*/
#define     ADSERR_DEVICE_NOTREADY              (0x07+(ERR_ADSERRS))    /**< \brief Device is not in a ready state*/
#define     ADSERR_DEVICE_BUSY                  (0x08+(ERR_ADSERRS))    /**< \brief Device is busy*/
#define     ADSERR_DEVICE_NOMEMORY              (0x0A+(ERR_ADSERRS))    /**< \brief Out of memory*/
#define     ADSERR_DEVICE_INVALIDPARM           (0x0B+(ERR_ADSERRS))    /**< \brief Invalid parameter value(s)*/
#define     ADSERR_DEVICE_NOTFOUND              (0x0C+(ERR_ADSERRS))    /**< \brief Not found (files, ...)*/
#define     ADSERR_DEVICE_SYNTAX                (0x0D+(ERR_ADSERRS))    /**< \brief Syntax error in comand or file*/
#define     ADSERR_DEVICE_INCOMPATIBLE          (0x0E+(ERR_ADSERRS))    /**< \brief Objects do not match*/
#define     ADSERR_DEVICE_EXISTS                (0x0F+(ERR_ADSERRS))    /**< \brief Object already exists*/
#define     ADSERR_DEVICE_INVALIDSTATE          (0x12+(ERR_ADSERRS))    /**< \brief Server is in invalid state*/
#define     ADSERR_DEVICE_NOTINIT               (0x18+(ERR_ADSERRS))    /**< \brief Device not initialized*/
#define     ADSERR_DEVICE_TIMEOUT               (0x19+(ERR_ADSERRS))    /**< \brief Device has a timeout*/


/*---------------------------------------------
-    ADS index groups
-----------------------------------------------*/
#define     ADSIGRP_CANOPEN_SDO                 0xF302                  /**< \brief SDO access*/


/*! \brief AoE OBD access
* \details
* index = HIWORD(iOffs)
* subIndex = LOBYTE(LOWORD(iOffs))
*/

#define     ADSIGRP_CANOPEN_SDO_INFO_LIST       0xF3FC                  /**< \brief ListType = HIWORD(iOffs)*/
#define     ADSIGRP_CANOPEN_SDO_INFO_OBJ        0xF3FD                  /**< \brief index = HIWORD(iOffs)*/
#define     ADSIGRP_CANOPEN_SDO_INFO_ENTRY      0xF3FE                  /**< \brief index = HIWORD(iOffs), subIndex = LOBYTE(LOWORD(iOffs))*/


/*---------------------------------------------
-    ADS index offsets
-----------------------------------------------*/
#define     ADSIOFFS_ECAT_COE_COMPLETEACCESS    0x00000100              /**< \brief SDO complete access mask(Group 0xF302)*/



#if (defined __cplusplus)
}
#endif

#endif /* __ECSLVAPIDEF_AOE_H__ */
