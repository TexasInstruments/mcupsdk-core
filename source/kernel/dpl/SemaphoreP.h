/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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

 #ifndef SEMAPHOREP_H
 #define SEMAPHOREP_H
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 #include <stdint.h>
 #include <kernel/dpl/SystemP.h>
 
 /**
  * \defgroup KERNEL_DPL_SEMAPHORE APIs for Semaphore
  * \ingroup KERNEL_DPL
  *
  * For more details and example usage, see \ref KERNEL_DPL_SEMAPHORE_PAGE
  *
  * @{
  */
 
  #if defined(OS_NORTOS)
  /**
   * \brief Semaphore object for NoRTOS
   */
  typedef struct SemaphoreP_Object_ {
      uint32_t type;
      uint32_t maxCount;
      volatile uint32_t count;
      volatile uint32_t nestCount;
  } SemaphoreP_Object;
  
  #elif defined (OS_FREERTOS) || defined (OS_FREERTOS_SMP) || defined (OS_FREERTOS_MPU)
  /**
   * \brief Semaphore object for FreeRTOS
   */
  #include <FreeRTOS.h>
  #include <task.h>
  #include <semphr.h>
  typedef struct SemaphoreP_Object_ {
      StaticSemaphore_t semObj;
      SemaphoreHandle_t semHndl;
      uint32_t isRecursiveMutex;
  } SemaphoreP_Object;
  
  #elif defined(OS_SAFERTOS)
  /**
   * \brief Semaphore object for SafeRTOS
   */
  #include <SafeRTOS.h>
  #include <queue.h>
  #include <semaphore.h>
  #include <mutex.h>
  typedef struct SemaphoreP_Object_ {
      uint64_t semObj[(safertosapiQUEUE_OVERHEAD_BYTES / sizeof(uint64_t) + 1)];
      xSemaphoreHandle semHndl;
      uint32_t isRecursiveMutex;
  } SemaphoreP_Object;
  
  #else
  #error "Define OS_NORTOS, OS_FREERTOS, or OS_SAFERTOS"
  #endif
 
 /**
  * \brief Create a mutex semaphore object
  *
  * \param obj [out] created object
  *
  * \return \ref SystemP_SUCCESS on success, \ref SystemP_FAILURE on error
  */
 int32_t SemaphoreP_constructMutex(SemaphoreP_Object *obj);
 
 /**
  * \brief Create a binary semaphore object
  *
  * \param obj [out] created object
  * \param initValue [in] Initial value of the binary semaphore, MUST be 0 or 1
  *
  * \return \ref SystemP_SUCCESS on success, \ref SystemP_FAILURE on error
  */
 int32_t SemaphoreP_constructBinary(SemaphoreP_Object *obj, uint32_t initValue);
 
 /**
  * \brief Create a counting semaphore object
  *
  * \param obj [out] created object
  * \param initValue [in] Initial value of the counting semaphore, MUST be between 0 .. maxValue
  * \param maxValue [in] Maximum value of counting semaphore
  *
  * \return \ref SystemP_SUCCESS on success, \ref SystemP_FAILURE on error
  */
 int32_t SemaphoreP_constructCounting(SemaphoreP_Object *obj, uint32_t initValue, uint32_t maxValue);
 
 /**
  * \brief Cleanup, delete, destruct a semaphore object
  *
  * \param obj [in] semaphore object
  */
 void SemaphoreP_destruct(SemaphoreP_Object *obj);
 
 /**
  * \brief Post a semaphore object or unlock a mutex
  *
  * \param obj [in] semaphore object
  */
 void SemaphoreP_post(SemaphoreP_Object *obj);
 
 /**
  * \brief Pend on a semaphore object or lock a mutex
  *
  * \param obj [in] semaphore object
  * \param timeToWaitInTicks [in] amount of time to block waiting for semaphore to be available, in units of system ticks (see \ref KERNEL_DPL_CLOCK_PAGE)
  *
  * \return \ref SystemP_SUCCESS on successful acquire of the semaphore
  * \return \ref SystemP_TIMEOUT on failure to acquire the semaphore due to timeout condition
  * \return \ref SystemP_FAILURE on failure to acquire the semaphore due to other conditions
  */
 int32_t SemaphoreP_pend(SemaphoreP_Object *obj, uint32_t timeToWaitInTicks);
 
 /*!
  *  @brief  Function to return the count of a semaphore.
  *
  *  @param  obj  A SemaphoreP_Handle
  *
  *  @return The count of the semaphore
  */
 
 int32_t SemaphoreP_getCount(SemaphoreP_Object *obj);
 /** @} */
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* SEMAPHOREP_H */
 
