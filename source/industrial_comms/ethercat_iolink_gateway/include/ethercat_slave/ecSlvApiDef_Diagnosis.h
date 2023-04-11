/*!
* \file ecSlvApiDef_Diagnosis.h
*
* \brief
* Diagnosis history defines.
*
* \author
* KUNBUS GmbH
*
* \date
* 2022-12-13
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

#if !(defined __ECSLVAPIDEF_DIAGNOSIS_H__)
#define __ECSLVAPIDEF_DIAGNOSIS_H__		1

#include <stdint.h>

#if (defined __cplusplus)
extern "C" {
#endif

/*-----------------------------------------------------------------------------------------
------
------    Defines and Types
------
-----------------------------------------------------------------------------------------*/

/**
* \addtogroup DIAG types and flags
* @{
*/

/*---------------------------------------------
-    DIAG types
-----------------------------------------------*/
/**
* \brief This structure handles parameter information
*/
typedef struct EC_API_SLV_SDIAG_parameter
{
    uint16_t    defType;    /**< \brief paramter type (DEFTYPE_UNSIGNED8, DEFTYPE_UNSIGNED16...) */
    uint8_t     msgType;    /**< \brief message type (DIAG_MSG_PARAM_TYPE_DATA, DIAG_MSG_PARAM_TYPE_B_ARRY...) */
    uint32_t    length;     /**< \brief param length */
    uint8_t*    pParam;     /**< \brief pointer to the corresponding parameter memory*/
} EC_API_SLV_SDIAG_parameter_t;


/*---------------------------------------------
-    DIAG flags
-----------------------------------------------*/

#define DIAG_MSG_TYPE_INFO           0x0 /**< \brief Diagnosis message type info*/
#define DIAG_MSG_TYPE_WARNING        0x1 /**< \brief Diagnosis message type warning*/
#define DIAG_MSG_TYPE_ERROR          0x2 /**< \brief Diagnosis message type error*/

#define DIAG_MSG_PARAM_TYPE_DATA    0x0 /**< \brief Diagnosis parameter type data*/
#define DIAG_MSG_PARAM_TYPE_B_ARRY  0x1 /**< \brief Diagnosis parameter type Byte Array*/
#define DIAG_MSG_PARAM_TYPE_ASCII   0x2 /**< \brief Diagnosis parameter type ASCII*/
#define DIAG_MSG_PARAM_TYPE_UNICODE 0x3 /**< \brief Diagnosis parameter type UNICODE*/

/*Defined Diagnosis Codes*/
#define DIAG_CODE_EMCY(Code)        (((uint32_t)(Code))<< 16 | (uint16_t)(0xE800))  /**< \brief Macro to create emergency code*/
#define DIAG_APPL_INFO              0xE002                                          /**< \brief Diag Code for application information*/
#define DIAG_MDP_DEVICE             0xE803                                          /**< \brief Device related messages in a modular device*/

/*Text IDs*/
#define DIAG_TEXT_ID_STATE_CHANGE_INFO           0x01 /**< \brief State change Request From: %d To:%d*/
#define DIAG_TEXT_ID_DEC_NUMBER                  0x02 /**< \brief %d*/
#define DIAG_TEXT_ID_CALC_BUS_CYCLE_FAILED       0x0F /**< \brief Calculate bus cycle time failed (Local timer too slow)*/
#define DIAG_TEXT_ID_STATE_SM_ADDR               0x11 /**< \brief Sync Manager %d invalid address (%d)*/
#define DIAG_TEXT_ID_STATE_SM_SIZE               0x12 /**< \brief Sync Manager %d invalid size (%d)*/
#define DIAG_TEXT_ID_STATE_SM_SETTING            0x13 /**< \brief Sync Manager %d invalid settings (%d)*/
#define DIAG_TEXT_ID_INVALID_REG_981             0x20 /**< \brief DC activation register is invalid*/
#define DIAG_TEXT_ID_INVALID_SYNC_TYPE           0x21 /**< \brief Configured SyncType (0x1C32.1 or 0x1C33.1) not supported. Check DC registers and supported SyncTypes (0x1C32.4 and 0x1C33.4)*/

/** @}*/

#if (defined __cplusplus)
}
#endif

#endif /* __ECSLVAPIDEF_DIAGNOSIS_H__ */
