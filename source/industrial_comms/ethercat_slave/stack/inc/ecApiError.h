/*!
* \file ecApiError.h
*
* \brief
* EtherCAT Error codes.
*
* \author
* KUNBUS GmbH
*
* \date
* 2021-05-19
*
* \copyright
* Copyright (c) 2021, KUNBUS GmbH<br /><br />
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

#if !(defined __ECAPIERROR_H__)
#define __ECAPIERROR_H__		1

#if (defined __cplusplus)
extern "C" {
#endif


//+=============================================================================================
//|     Konstanten / constants
//+=============================================================================================

// ERROR-CLASS Sitara EtherCAT DTK  0x3801 0000

#define EC_ERROR_CLASS(x) (0x38010000|((x)&0xffff))

//+=============================================================================================
//|     Typen / types
//+=============================================================================================

/*! This enum represent each API Error
 *  \ingroup EC_API_SLV_ERROR_CODES
 */
typedef enum EC_API_EError
{
    EC_API_eERR_NONE                 = 0,                        /*!< No Error */
    EC_API_eERR_CRITICAL             = EC_ERROR_CLASS(0x0001),
    EC_API_eERR_EMPTY                = EC_ERROR_CLASS(0x0002),
    EC_API_eERR_FULL                 = EC_ERROR_CLASS(0x0003),
    EC_API_eERR_MEM                  = EC_ERROR_CLASS(0x0004),
    EC_API_eERR_BUSY                 = EC_ERROR_CLASS(0x0005),
    EC_API_eERR_ABORT                = EC_ERROR_CLASS(0x0006),
    EC_API_eERR_INVALID              = EC_ERROR_CLASS(0x0007),
    EC_API_eERR_STOPPED              = EC_ERROR_CLASS(0x0008),
    EC_API_eERR_NOTFOUND             = EC_ERROR_CLASS(0x0009),
    EC_API_eERR_TIMEOUT              = EC_ERROR_CLASS(0x000A),
    EC_API_eERR_SKIP                 = EC_ERROR_CLASS(0x000B),
    EC_API_eERR_WAIT                 = EC_ERROR_CLASS(0x000C),
    EC_API_eERR_NULLPTR              = EC_ERROR_CLASS(0x000D),
    EC_API_eERR_ALLOC_FAILED         = EC_ERROR_CLASS(0x000E),
    EC_API_eERR_OUT_OF_BOUND         = EC_ERROR_CLASS(0x000F),
    EC_API_eERR_STR_OVERFLOW         = EC_ERROR_CLASS(0x0010),
    EC_API_eERR_PDO_OVERFLOW         = EC_ERROR_CLASS(0x0011),
    EC_API_eERR_NOT_INITIALIZED      = EC_ERROR_CLASS(0x0012),
    EC_API_eERR_MALLOC               = EC_ERROR_CLASS(0x0013),
    EC_API_eERR_UNALIGNED            = EC_ERROR_CLASS(0x0014),
    EC_API_eERR_TX_PROC_IMG_OVERFLOW = EC_ERROR_CLASS(0x0015),
    EC_API_eERR_RX_PROC_IMG_OVERFLOW = EC_ERROR_CLASS(0x0016),
    EC_API_eERR_COE_OBJ_DESC_OVERFLOW= EC_ERROR_CLASS(0x0017),
    EC_API_eERR_MAP_AREA_OVERFLOW    = EC_ERROR_CLASS(0x0018),
    EC_API_eERR_PDO_ENTRY_OVERFLOW   = EC_ERROR_CLASS(0x0019),
    EC_API_eERR_VARIABLE_OVERFLOW    = EC_ERROR_CLASS(0x001A),
    EC_API_eERR_EMERGENCY_QUEUE_FULL = EC_ERROR_CLASS(0x001B),
    EC_API_eERR_ERR_NO_CYCLIC_BUFFER = EC_ERROR_CLASS(0x001C),
    EC_API_eERR_READ_ONLY            = EC_ERROR_CLASS(0x001D),
    EC_API_eERR_OBJECT_DICT_CREATE   = EC_ERROR_CLASS(0x001E),
    EC_API_eERR_VENDOR_ID_NOT_SET    = EC_ERROR_CLASS(0x001F),
    EC_API_eERR_PRODUCT_CODE_NOT_SET = EC_ERROR_CLASS(0x0020),
    EC_API_eERR_PRODUCT_NAME_NOT_SET = EC_ERROR_CLASS(0x0021),
    EC_API_eERR_HW_VERSION_NOT_SET   = EC_ERROR_CLASS(0x0022),
    EC_API_eERR_SW_VERSION_NOT_SET   = EC_ERROR_CLASS(0x0023),
    EC_API_eERR_COE_OBJ_OVERFLOW     = EC_ERROR_CLASS(0x0024),
    EC_API_eERR_MAP_ADDR_INV         = EC_ERROR_CLASS(0x0025),
    EC_API_eERR_MISSING_PROC_DATA    = EC_ERROR_CLASS(0x0026),
    EC_API_eERR_NO_INPUT_PROC_IMAGE  = EC_ERROR_CLASS(0x0027),
    EC_API_eERR_NO_OUTPUT_PROC_IMAGE = EC_ERROR_CLASS(0x0028),
    EC_API_eERR_FW_ERR_OPEN_FAILED   = EC_ERROR_CLASS(0x0029),
    EC_API_eERR_FW_ERR_MEM_OVERFLOW  = EC_ERROR_CLASS(0x002A),
    EC_API_eERR_NO_INPUT_PROC_MEMORY = EC_ERROR_CLASS(0x002B),
    EC_API_eERR_NO_OUTPUT_PROC_MEMORY= EC_ERROR_CLASS(0x002C),
    EC_API_eERR_INVALID_SYNCMAN_IDX  = EC_ERROR_CLASS(0x002D),
    EC_API_eERR_NOT_SUPPORTED        = EC_ERROR_CLASS(0x002E),
    EC_API_eERR_XML_OPEN_FAILED      = EC_ERROR_CLASS(0x002F),
    EC_API_eERR_XML_HEX_PARSE_FAILED = EC_ERROR_CLASS(0x0030),
    EC_API_eERR_XML_PARSE_INV_STATE  = EC_ERROR_CLASS(0x0031),
    EC_API_eERR_XML_PARSE_INV_VALUE  = EC_ERROR_CLASS(0x0032),
    EC_API_eERR_XML_CREATE_PDO       = EC_ERROR_CLASS(0x0033),
    EC_API_eERR_XML_PARSE_PDO_IDX_VAL= EC_ERROR_CLASS(0x0034),
    EC_API_eERR_XML_CREATE_PDO_ENTRY = EC_ERROR_CLASS(0x0035),
    EC_API_eERR_COE_OBJ_INDEX_EXISTS = EC_ERROR_CLASS(0x0036),
    EC_API_eERR_PDO_MAPINDEX_EXISTS  = EC_ERROR_CLASS(0x0037),
    EC_API_eERR_EVALUATION_MODE      = EC_ERROR_CLASS(0x0038),
    EC_API_eERR_GROUP_TYPE_NOT_SET   = EC_ERROR_CLASS(0x0039),
    EC_API_eERR_DEPRECATED           = EC_ERROR_CLASS(0x0039),
} EC_API_EError_t;

#if (defined __cplusplus)
}
#endif

#endif /* __ECAPIERROR_H__ */
