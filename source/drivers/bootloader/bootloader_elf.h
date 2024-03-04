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

#ifndef _BOOTLOADER_ELF_H_
#define _BOOTLOADER_ELF_H_

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

typedef struct Bootloader_ELFH32_s
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

} Bootloader_ELFH32; /* 52 bytes */

typedef struct Bootloader_ELFH64_s
{
    uint8_t  e_ident[E_IDENT];
    uint16_t e_type;
    uint16_t e_machine;
    uint32_t e_version;
    uint64_t e_entry;
    uint64_t e_phoff;
    uint64_t e_shoff;
    uint32_t e_flags;
    uint16_t e_ehsize;
    uint16_t e_phentsize;
    uint16_t e_phnum;
    uint16_t e_shentsize;
    uint16_t e_shnum;
    uint16_t e_shstrndx;

} Bootloader_ELFH64; /* 64 bytes */

typedef struct Bootloader_ELFPH32_s
{
	uint32_t type;
    uint32_t offset;
    uint32_t vaddr;
    uint32_t paddr;
    uint32_t filesz;
    uint32_t memsz;
    uint32_t flags;
    uint32_t align;

} Bootloader_ELFPH32; /* 32 bytes */

typedef struct Bootloader_ELFPH64_s
{
	uint32_t type;
    uint32_t flags;
    uint64_t offset;
    uint64_t vaddr;
    uint64_t paddr;
    uint64_t filesz;
    uint64_t memsz;
    uint64_t align;

} Bootloader_ELFPH64; /* 56 bytes */

typedef struct Bootloader_ELFNote_s
{
    uint32_t namesz;
    uint32_t descsz;
    uint32_t type;
} Bootloader_ELFNote;

typedef struct Bootloader_EntryPoint32_s
{
    uint32_t coreId;
    uint32_t entryPoint;

} Bootloader_EntryPoint32;

typedef struct Bootloader_EntryPoint64_s
{
    uint32_t coreId;
    uint64_t entryPoint;

} Bootloader_EntryPoint64;



#ifdef __cplusplus
}
#endif

#endif /* _BOOTLOADER_ELF_H_ */