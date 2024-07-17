/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

#ifndef __ELF_U_PARSER_H_
#define __ELF_U_PARSER_H_

/* tielfup or TI ELF Micro Parser */

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#define E_IDENT 16

/* Segment types */
#define PT_NULL       (0)
#define PT_LOAD       (1)
#define PT_DYNAMIC    (2)
#define PT_INTERP     (3)
#define PT_NOTE       (4)
#define PT_SHLIB      (5)
#define PT_PHDR       (6)

#define ELFCLASS_IDX              (4U)
#define ELFCLASS_32               (1U)
#define ELFCLASS_64               (2U)

#define ELF_NOTE_NAMESZ_SIZE      (4U)
#define ELF_NOTE_DESCSZ_SIZE      (4U)
#define ELF_NOTE_TYPE_SIZE        (4U)

#define ELF_HEADER_32_SIZE        (52U)
#define ELF_HEADER_64_SIZE        (64U)
#define ELF_P_HEADER_32_SIZE      (32U)
#define ELF_P_HEADER_64_SIZE      (56U)

#define ELF_HEADER_MAX_SIZE       (ELF_HEADER_64_SIZE)
#define ELF_MAX_SEGMENTS          (64U)
#define ELF_P_HEADER_MAX_SIZE     (ELF_P_HEADER_64_SIZE)
#define ELF_NOTE_SEGMENT_MAX_SIZE (512U)


#define ELFUP_PARSER_STATE_INIT     (0U)
#define ELFUP_PARSER_STATE_IDEN0    (1U)
#define ELFUP_PARSER_STATE_IDEN1    (2U)
#define ELFUP_PARSER_STATE_IDEN2    (3U)
#define ELFUP_PARSER_STATE_IDEN3    (4U)
#define ELFUP_PARSER_STATE_PHT      (5U)
#define ELFUP_PARSER_STATE_END      (6U)
#define ELFUP_PARSER_STATE_ERROR    (7U)

typedef struct ELFUP_ELFH32_s
{
    uint8_t  e_ident[E_IDENT];
    uint16_t e_type;
    uint16_t e_machine;
    uint32_t e_version;
    uint32_t e_entry;
    uint32_t e_phoff;
    uint32_t e_shoff;
    uint32_t e_flags;
    uint16_t e_ehsize;
    uint16_t e_phentsize;
    uint16_t e_phnum;
    uint16_t e_shentsize;
    uint16_t e_shnum;
    uint16_t e_shstrndx;

} ELFUP_ELFH32; /* 52 bytes */

typedef struct ELFUP_ELFPH32_s
{
	uint32_t type;
    uint32_t offset;
    uint32_t vaddr;
    uint32_t paddr;
    uint32_t filesz;
    uint32_t memsz;
    uint32_t flags;
    uint32_t align;

} ELFUP_ELFPH32; /* 32 bytes */

typedef struct ELFUP_ELFNote_s
{
    uint32_t namesz;
    uint32_t descsz;
    uint32_t type;
} ELFUP_ELFNote;

typedef struct ELFUP_EntryPoint32_s
{
    uint32_t coreId;
    uint32_t entryPoint;

} ELFUP_EntryPoint32;

typedef union ELFUP_ELFH32_u_t
{
    ELFUP_ELFH32 ELFH;
    uint8_t buff[sizeof(ELFUP_ELFH32)];
} ELFUP_ELFH;

typedef union ELFUP_ELFPH32_u_t
{
    ELFUP_ELFPH32 ELFPH;
    uint8_t buff[sizeof(ELFUP_ELFPH32)];
} ELFUP_ELFPH;


typedef struct ELFUP_State_s_t
{
    uint32_t stateNext;
    uint32_t stateNow;
    uint32_t statePrev;
    uint32_t elfFileStartOffset;
    uint32_t genericOffsetCounter;
    uint8_t maxPhtSize;
    uint8_t headerCnt;
    ELFUP_ELFH ELFHeader;
    ELFUP_ELFPH *pht;
}ELFUP_Handle;

/**
 * @brief Initilizer the ELF parser state machine
 *
 * Internally this parser works based on mili-State machine design and works
 * on each character of an ELF file. This design is made so that the ELF
 * file which is being recieved over some interface (CAN/Ethernet/etc.) can be
 * parsed on the fly, since entire file is not available at a time. Other design
 * constraint is that this should not do any dynamic allocation. Since the size of
 * program header table  is not known before hand, therefore, applcaition should
 * allocate some space where program header table can be stored and this is done
 * by passing that via pPhtArr and pPhtArrSize.
 *
 * @param handle pointer to handle of parser
 * @param pPhtArr pointer to an array of ELFUP_ELFPH
 * @param pPhtArrSize size of pPhtArr
 * @return int16_t System_SUCCSESS if all okay
 */
int16_t ELFUP_init(ELFUP_Handle *handle, ELFUP_ELFPH *pPhtArr, uint8_t pPhtArrSize);

/**
 * @brief Update the parser with new data 
 * 
 * There are 2 ways this function returns an error. 1st is 
 * when wrong arguments are passed that is when this function will 
 * return SystemP_FAILURE. Other error is the failure to parse in case 
 * the ELF file is corrupted or is incorrect. This error is stored in 
 * handle.stateNext. In case of failture to parse, handle.stateNext
 * will have value of ELFUP_PARSER_STATE_ERROR.
 * 
 * @param handle pointer to handle of parser
 * @param byte byte of new data
 * @return int16_t SystemP_FAILURE if there was any failure of API
 */
int16_t ELFUP_update(ELFUP_Handle *handle, uint8_t byte);

/**
 * @brief Check if a byte is part of program segment or not.
 * 
 * @param handle pointer to handle of parser
 * @param offset offset of the byte whose info is required.
 * @param phtInfo pointer to memory area where the header info will be written.
 * @return int16_t SystemP_FAILURE if there was any failure of API
 */
int16_t ELFUP_isPartOfSegment(ELFUP_Handle *handle, uint32_t offset, ELFUP_ELFPH *phtInfo);

#ifdef __cplusplus
}
#endif

#endif /* __ELF_U_PARSER_H_ */