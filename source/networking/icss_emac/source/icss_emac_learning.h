/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

#ifndef ICSS_EMAC_LEARNING_H_
#define ICSS_EMAC_LEARNING_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <networking/icss_emac/icss_emac.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *  @def  ICSS_EMAC_LEARNING_PORT_1
 *        Used to specify physical port 1 MII 0 (tx)
 */
#define ICSS_EMAC_LEARNING_PORT_1                       1

/**
 *  @def  ICSS_EMAC_LEARNING_PORT_2
 *        Used to specify physical port 2 MII 1 (tx)
 */
#define ICSS_EMAC_LEARNING_PORT_2                       2

/**
* @def ICSS_EMAC_LEARNING_NUMBUCKETS
*      Number of Buckets in the learning table(per port)
*/
#define ICSS_EMAC_LEARNING_NUMBUCKETS                   256
/**
* @def ICSS_EMAC_LEARNING_MAX_NUM_ENTRIES_PER_BUCKET
*      Number of entries per Bucket
*/
#define ICSS_EMAC_LEARNING_MAX_NUM_ENTRIES_PER_BUCKET   4

/**
* @def ICSS_EMAC_LEARNING_AGEING_COUNT
*      used for ageing, higher value means entries age out slower, max value is 255
*/
#define ICSS_EMAC_LEARNING_AGEING_COUNT                 (100U)

/**Compare MAC ID's. Uses memcmp internally */
#define COMPARE_MAC(x, y)   (memcmp((x), (y), 6U) == 0)
/**Copy MAC ID. Uses memcpy internally */
#define COPY_MAC(dst, src)  (memcpy((dst), (src), 6U))

/** Number of buckets in each port's learning table*/
#define ICSS_EMAC_LEARNING_TABLE_SIZE                   256

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
* @brief MAC ID
*
*/
typedef struct ICSS_EMAC_Mac_s
{
    /**MAC ID*/
    uint8_t macId[6];
} ICSS_EMAC_Mac;

/**
* @brief Structure for Individual bucket. A hash table is made up of several such buckets
*
*/
typedef struct ICSS_EMAC_HashBucket_s
{
    /**Four MAC Id per bucket*/
    ICSS_EMAC_Mac   mac[ICSS_EMAC_LEARNING_MAX_NUM_ENTRIES_PER_BUCKET];
    /**Timer count used for ageing and conflict resolution*/
    uint8_t         timerCount[ICSS_EMAC_LEARNING_MAX_NUM_ENTRIES_PER_BUCKET];
    /**Number of MAC entries in the bucket, 4 means it is full*/
    uint8_t         numEntries;
} ICSS_EMAC_HashBucket;

/**
* @brief Structure for the Hash Table. To be declared as an array of two. One for each port
*
*/
typedef struct ICSS_EMAC_HashTable_s
{
    /**Total number of entries in the hash table*/
    uint32_t                totalNumEntries;
    /*TODO: Review this change*/
    /**State of the hash table, see enum portState above*/
    uint32_t                state;
    /**Number of bucket entries*/
    ICSS_EMAC_HashBucket    entries[ICSS_EMAC_LEARNING_NUMBUCKETS];
} ICSS_EMAC_HashTable;

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */

/**
* @brief Update the learning table used to map MAC Id's and ports
*
* @param  icssEmacHandle ICSS EMAC Handle
* @param  macId MACID which is to be learned
* @param  portNum port number of incoming packet
* @param  tablePtr Hash Table pointer for the port for which table must be updated
* @param  exceptionCallBack Call back for protocol specific task
*
* @retval none
*/
void ICSS_EMAC_updateHashTable(ICSS_EMAC_Handle                 icssEmacHandle,
                               uint8_t                          *macId,
                               uint8_t                          portNum,
                               ICSS_EMAC_HashTable              *tablePtr,
                               const ICSS_EMAC_CallBackConfig   *exceptionCallBack);

/**
* @brief Clear the Learning table associated with a given port
*
* @param portNum Port number whose learning table is to be flushed
* @param  tablePtr Hash Table pointer for the port for which table is to be flushed
*
* @retval none
*/
void ICSS_EMAC_purgeTable(uint8_t portNum, ICSS_EMAC_HashTable *tablePtr);

/**
* @brief Remove old MAC entries from the table
*
* @param  portNum Portnumber whose learning table is to be flushed
* @param  tablePtr pointer to the hash table for corresponding portNum
*
* @retval none
*/
void ICSS_EMAC_ageingRoutine(uint8_t portNum, ICSS_EMAC_HashTable *tablePtr);

/**
* @brief Find given a MAC ID which port it exists
*
* @param  macId MAC ID which is to be found
*
* @retval tablePtr where the MAC ID exists, 0 means not found 1 means port 0 and 2 means port 1
*/
uint8_t ICSS_EMAC_findMAC(const uint8_t *macId, ICSS_EMAC_HashTable *tablePtr);

/**
* @brief Find given a MAC ID and remove it from the table
*
* @param  macId  MAC which is to be found
* @param  tablePtr Hash Table pointer for the port for which MAC ID (provided) to be removed
*
* @retval pass/fail. 0 means operation failed (mac not found)
*/
uint8_t ICSS_EMAC_removeMAC(const uint8_t *macId, ICSS_EMAC_HashTable *tablePtr);

/**
* @brief This takes in a hash key and the port number and removes the MAC ID if found in the other port
*
* @retval none
*/
void ICSS_EMAC_incrementCounter(ICSS_EMAC_HashTable *tablePtr);

/**
* @brief This takes in a hash key and the port number and removes the MAC ID if found in the other port
*
* @retval none
*/
void ICSS_EMAC_initLearningTable(ICSS_EMAC_HashTable *tablePtr);

/**
* @brief Change the Port state. Supported values are Learning/Not Learning/Locked
*
* @param  state Port state which is an Enum type. See corresponding in include file
* @param  tablePtr Hash Table pointer for the port for which state must be changed
*
* @retval none
*/
void ICSS_EMAC_changePortState(uint32_t state, ICSS_EMAC_HashTable *tablePtr);

/**
* @brief API to copy Macaddress
*
* @param dst destination address
 * @param src source address
*
* @retval none
*/
void ICSS_EMAC_copyMAC(uint8_t *dst, const uint8_t *src);

/* TODO: Review the need of this function*/
// /**
//  * Name : hashFuncGPMAC_asm
//  * The above function written in assembly
//  */
// uint8_t hashFuncGPMAC_asm(uint8_t* macId);
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef ICSS_EMAC_LEARNING_H_ */
