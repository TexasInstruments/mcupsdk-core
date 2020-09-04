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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "icss_emac_learning.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */
static uint8_t ICSS_EMAC_hashFuncGPMAC(const uint8_t *macId);

static uint8_t ICSS_EMAC_checkDuplicateMAC(uint8_t              key,
                                           const uint8_t        *macId,
                                           uint8_t              portNum,
                                           ICSS_EMAC_HashTable  *tablePtr);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void ICSS_EMAC_updateHashTable(ICSS_EMAC_Handle                 icssEmacHandle,
                               uint8_t                          *macId,
                               uint8_t                          portNum,
                               ICSS_EMAC_HashTable              *tablePtr,
                               const ICSS_EMAC_CallBackConfig   *exceptionCallBack)
{
    uint8_t                 key = 0;
    uint8_t                 i;
    uint8_t                 oldestTimestamp;
    uint8_t                 index = 0;
    uint8_t                 macFound;
    ICSS_EMAC_HashBucket    *bucket;
    ICSS_EMAC_HashTable     *table;
    ICSS_EMAC_CallBack      protException;
    void                    *protExceptionuser = exceptionCallBack->userArg;

    protException = exceptionCallBack->callBack;

    if((tablePtr == NULL) || (macId == NULL))
    {
        return;
    }

    if((macId[0] & 0x01U) > 0U)
    { /* MAC reversed - lowest bit indicates MC  */
        return;
    }

    /*TODO: Test this*/
    table = tablePtr + portNum - 1U;

    /* don't do anything for wrong port num  */
    if(portNum > (uint8_t)ICSS_EMAC_LEARNING_PORT_2)
    {
        return;
    }

    /*Call back for Protocol specific adaption*/
    if(protException != NULL)
    {
        if(protException((void *)icssEmacHandle, macId, protExceptionuser) == 0U)
        {
            return;
        }
    }
    /*get the key*/
    key = ICSS_EMAC_hashFuncGPMAC(macId);

    if((table->state == ICSS_EMAC_LEARNING_PORT_STATE_LOCKED) ||
       (table->state == ICSS_EMAC_LEARNING_PORT_STATE_NOT_LEARNING))
    {
        return; /*not allowed to make any changes in cases table is locked or if learning is disabled  */
    }
    /*go to first hash entry in the table*/
    bucket = &(table->entries[key]);
    macFound = 0; /*set to false*/

    /*If bucket is empty update the first entry else run through the bucket*/
    if(bucket->numEntries == 0)
    {
        /* make sure the SLT has no entry for this MAC on the other port  */
        ICSS_EMAC_checkDuplicateMAC(key, macId, portNum, tablePtr);

        /* add entry  */
        ICSS_EMAC_copyMAC((bucket->mac[0].macId), macId);

        bucket->timerCount[0] = 0;
        bucket->numEntries++;
        table->totalNumEntries++;
    } 
    else
    {
        oldestTimestamp = bucket->timerCount[0];

        for(i = 0; i < bucket->numEntries; i++)
        {
            /*Update all timer counts, for ageing*/
            if (COMPARE_MAC(macId, bucket->mac[i].macId))
            {
                bucket->timerCount[i] = 0;
                macFound = 1u;
                /*early exit  */
                break;
            }
            /*if multiple MAC ID's hash to the same bucket, timer count is used to resolve the conflict*/
            if (oldestTimestamp <=  bucket->timerCount[i])
            {
                oldestTimestamp = bucket->timerCount[i];
                index = i;
            }
        }
        if (macFound == 0U) 
        {
            /* make sure the SLT has no entry for this MAC on the other port  */
            ICSS_EMAC_checkDuplicateMAC(key, macId, portNum, tablePtr);

            if(bucket->numEntries == ICSS_EMAC_LEARNING_MAX_NUM_ENTRIES_PER_BUCKET) 
            {
                /*replace an existing entry(the least hit)*/
                ICSS_EMAC_copyMAC(bucket->mac[index].macId, macId);
                bucket->timerCount[index] = 0;
            }
            else
            {
                if (bucket->numEntries < ICSS_EMAC_LEARNING_MAX_NUM_ENTRIES_PER_BUCKET)
                {
                    ICSS_EMAC_copyMAC(bucket->mac[bucket->numEntries].macId, macId);

                    bucket->timerCount[bucket->numEntries] = 0;
                    bucket->numEntries++;
                    table->totalNumEntries++;
                }
            }
        }
    }
}                     

void ICSS_EMAC_purgeTable(uint8_t portNum, ICSS_EMAC_HashTable *tablePtr)
{
    int32_t i;
    /*don't do anything for wrong port num*/
    if(portNum <= (uint8_t)ICSS_EMAC_LEARNING_PORT_2)
    { 
        ICSS_EMAC_HashTable *table;
        table = tablePtr;

         /*not allowed to make any changes in cases table is locked  */
        if(table->state != ICSS_EMAC_LEARNING_PORT_STATE_LOCKED) 
        {
            /*make total entries to 0*/
            table->totalNumEntries = 0;
            /*Do a dirty deletion*/
            for(i=0; i<ICSS_EMAC_LEARNING_NUMBUCKETS;i++)
            {
                table->entries[i].numEntries = 0;
            }
        }
    }
    return;
}

void ICSS_EMAC_ageingRoutine(uint8_t portNum, ICSS_EMAC_HashTable *tablePtr)
{

    int32_t                 i;
    int32_t                 a;
    int32_t                 j = 0;
    int32_t                 size;
    int32_t                 numEntriesRemoved;
    ICSS_EMAC_HashTable     *table;
    ICSS_EMAC_HashBucket    *bucket;
    /*Right now this is hardcoded, modified only at compile time, can be added as an option later*/
    uint8_t                 ageingTime = ICSS_EMAC_LEARNING_AGEING_COUNT;

    /* don't do anything for wrong port num  */
    if(portNum <= (uint8_t)ICSS_EMAC_LEARNING_PORT_2)
    {
        /*get the table for the corresponding port*/
        table  = tablePtr;

        if(table->state != ICSS_EMAC_LEARNING_PORT_STATE_LOCKED) 
        {
            for(i=0; i < ICSS_EMAC_LEARNING_NUMBUCKETS; i++) 
            {
                bucket = &(table->entries[i]);
                size = bucket->numEntries;
                numEntriesRemoved = size;
                a = 0;
                while(a < size)
                {
                    if(ageingTime <= bucket->timerCount[a])
                    {
                        j = a;
                        /*as we find hits, we decrement the size of the bucket*/
                        size--;
                        while(j < size)
                        {
                            /*copy the data from below once a hole is created in the bucket due to retiring an entry*/
                            bucket->timerCount[j] = bucket->timerCount[j+1];
                            ICSS_EMAC_copyMAC(bucket->mac[j].macId, bucket->mac[j+1].macId);
                            j++;
                        }
                    } else {
                        a++;
                    }
                }
                /*Number of entries left*/
                bucket->numEntries = size;
                numEntriesRemoved -= size;

                /*Update the total number of entries removed  */
                table->totalNumEntries -= numEntriesRemoved;
            }
        }
    }
    return;
}

uint8_t ICSS_EMAC_findMAC(const uint8_t *macId, ICSS_EMAC_HashTable *tablePtr)
{
    uint8_t                 j;
    uint8_t                 key;
    uint8_t                 macFound[ICSS_EMAC_MAX_PORTS_PER_INSTANCE] = {0};
    uint8_t                 numPorts = 0;
    ICSS_EMAC_HashTable     *table;
    ICSS_EMAC_HashBucket    *bucket;
    uint8_t                 ret_val = 0U;

    /*get the key to lookup in the hash table*/
    key = ICSS_EMAC_hashFuncGPMAC(macId);

    /*Look in all port entries, break if found*/
    for(numPorts = 0; numPorts < ICSS_EMAC_MAX_PORTS_PER_INSTANCE; numPorts++)
    {
        table = tablePtr + numPorts;
        bucket = &(table->entries[key]);
        for (j = 0; j < bucket->numEntries; j++ )
        {
            if (COMPARE_MAC(bucket->mac[j].macId, macId))
            {
                /*set the flag and exit*/
                macFound[numPorts] = (uint8_t)1U;
                break;
            }
        }
    }
    /*since a return val of 0 means MAC Id not found 1 is added to distinguish port 0 from mac id not found,
     user to subtract 1 to get port number*/
    /*If MAC ID moves from port 1 to port 2, both ports will return false*/
    for(numPorts = 0; numPorts < ICSS_EMAC_MAX_PORTS_PER_INSTANCE; numPorts++) 
    {
        if(macFound[numPorts]) 
        {
            ret_val = numPorts + 1U;
            break;
        }
    }
    return ret_val;
}

uint8_t ICSS_EMAC_removeMAC(const uint8_t *macId, ICSS_EMAC_HashTable *tablePtr)
{
    uint8_t                 j = 0;
    uint8_t                 key = 0;
    uint8_t                 macFound[ICSS_EMAC_MAX_PORTS_PER_INSTANCE] = {0};
    uint8_t                 macIndex[ICSS_EMAC_MAX_PORTS_PER_INSTANCE] = {0};
    uint8_t                 numPorts = 0;
    ICSS_EMAC_HashTable     *table;
    ICSS_EMAC_HashBucket    *bucket;
    uint8_t                 ret_val = 0U;
    
    key = ICSS_EMAC_hashFuncGPMAC(macId);

    /*Look in all port entries, exit if true*/
    for(numPorts = 0; numPorts < ICSS_EMAC_MAX_PORTS_PER_INSTANCE; numPorts++)
    {
        table = tablePtr + numPorts;
        if(table->state != ICSS_EMAC_LEARNING_PORT_STATE_LOCKED) 
        {
            bucket = &(table->entries[key]);
            for (j = 0; j < bucket->numEntries; j++ ) 
            {
                if (COMPARE_MAC(bucket->mac[j].macId, macId)) 
                {
                    macFound[numPorts] = (uint8_t)1U;
                    macIndex[numPorts] = j;
                    break;
                }
            }
        }
    }

    for(numPorts = 0; numPorts < ICSS_EMAC_MAX_PORTS_PER_INSTANCE; numPorts++)
    {
        if(macFound[numPorts])
        {
            /*Remove for PORT 0*/
            table = tablePtr + numPorts;
            bucket = &(table->entries[key]);

            for(j = macIndex[numPorts]; j < (bucket->numEntries - ((uint8_t)1U)); j++)
            {
                bucket->timerCount[j] = bucket->timerCount[j+1U];
                ICSS_EMAC_copyMAC(bucket->mac[j].macId, bucket->mac[j+1U].macId);
            }
            bucket->numEntries--;
            table->totalNumEntries--;
            ret_val = 1U;
            break;
        }
    }
    return ret_val;
}

void ICSS_EMAC_incrementCounter(ICSS_EMAC_HashTable *tablePtr)
{
    int32_t                 i;
    int32_t                 j;
    ICSS_EMAC_HashTable     *table;
    ICSS_EMAC_HashBucket    *bucket;
    uint8_t                 port;

    /*Do for All ports*/
    for(port = 0; port < ICSS_EMAC_MAX_PORTS_PER_INSTANCE; port++)
    {
        table = tablePtr + port;
        if(table->state == ICSS_EMAC_LEARNING_PORT_STATE_LOCKED)
        {
            break; /*not allowed to make any changes in cases table is locked  */
        }
        for(i=0; i<ICSS_EMAC_LEARNING_NUMBUCKETS;i++)
        {
            bucket = &(table->entries[i]);
            for(j=0; j < bucket->numEntries; j++)
            {
                if(bucket->timerCount[j] <= ICSS_EMAC_LEARNING_AGEING_COUNT)
                {
                    bucket->timerCount[j]++;
                }
            }
        }
    }
    return;
}

void ICSS_EMAC_initLearningTable(ICSS_EMAC_HashTable *tablePtr)
{
    ICSS_EMAC_HashTable *table;
    uint8_t             ports = 0;

    /*Initialize all ports*/
    for(ports = 0; ports < ICSS_EMAC_MAX_PORTS_PER_INSTANCE; ports++)
    {
        table = tablePtr + ports;
        ICSS_EMAC_purgeTable(ports, table);
        table->state = ICSS_EMAC_LEARNING_PORT_STATE_LEARNING;
    }
}

void ICSS_EMAC_changePortState(uint32_t state, ICSS_EMAC_HashTable *tablePtr)
{
    tablePtr->state = state;
}

void ICSS_EMAC_copyMAC(uint8_t *dst, const uint8_t *src)
{
    uint8_t i;
    for(i = 0; i < 6U; i++)
    {
        *(dst) = *(src);
        dst++;
        src++;
    }
}

static uint8_t ICSS_EMAC_hashFuncGPMAC(const uint8_t *macId)
{
    uint8_t key;
    /*get the key*/
    key = macId[0] ^ macId[1] ^ macId[2] ^ macId[3] ^ macId[4] ^ macId[5];
    return key;    
}

static uint8_t ICSS_EMAC_checkDuplicateMAC(uint8_t              key,
                                           const uint8_t        *macId,
                                           uint8_t              portNum,
                                           ICSS_EMAC_HashTable  *tablePtr)
{
    ICSS_EMAC_HashBucket    *bucket;
    ICSS_EMAC_HashTable     *table;
    uint8_t                 i;
    uint8_t                 j;
    uint8_t                 macFound = 0;
    uint8_t                 port = 0;
    uint8_t                 ret_val = 0U;

    /*don't do anything for wrong port num*/
    if(portNum <= (uint8_t)ICSS_EMAC_LEARNING_PORT_2) 
    {
        /*Look in all other port tables for duplicates*/
        for(port = 0; port < ICSS_EMAC_MAX_PORTS_PER_INSTANCE; port++) 
        {
            /*Skip the current port, look in other ports*/
            if((portNum-1U) != port) 
            {
                table = tablePtr + port;
                bucket = &(table->entries[key]);

                for (j = 0; j < bucket->numEntries; j++ ) 
                {
                    if (COMPARE_MAC(bucket->mac[j].macId, macId)) 
                    {
                        macFound = 1U;
                        for(i = j; i < (bucket->numEntries - ((uint8_t)1U)); i++) 
                        {
                            bucket->timerCount[i] = bucket->timerCount[i+1U];
                            ICSS_EMAC_copyMAC(bucket->mac[i].macId, bucket->mac[i+1U].macId);
                        }
                        bucket->numEntries--;
                        table->totalNumEntries--;
                        break;
                    }
            
                }
                /*MAC Found, no need to look in other ports*/
                if(macFound == 1U)
                {
                    break;
                }
            }
        }
        ret_val = macFound;
    }
    return ret_val;    
}
