/*
 *  Copyright (C) 2018-2026 Texas Instruments Incorporated
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


#ifndef _TUSB_OSAL_THREADX_H_
#define _TUSB_OSAL_THREADX_H_

#include "tx_api.h"
#include "ClockP.h"

#ifdef __cplusplus
extern "C" {
#endif

/*--------------------------------------------------------------------+
 TASK API
* --------------------------------------------------------------------*/
static inline void osal_task_delay(uint32_t msec)
{
  uint32_t ticks;

  ticks = ClockP_usecToTicks(msec * 1000u);

  tx_thread_sleep(ticks);
}

/*--------------------------------------------------------------------+
* Semaphore API
*--------------------------------------------------------------------*/
typedef TX_SEMAPHORE osal_semaphore_def_t;
typedef TX_SEMAPHORE* osal_semaphore_t;

static inline osal_semaphore_t osal_semaphore_create(osal_semaphore_def_t* semdef)
{
  UINT status;

  status = tx_semaphore_create(semdef, "cdn_osal semaphore", 0u);
  if(status != TX_SUCCESS) {
      return NULL;
  }

  return semdef;
}

static inline bool osal_semaphore_post(osal_semaphore_t sem_hdl, bool in_isr)
{
  UINT status;

  status = tx_semaphore_put(sem_hdl);
  if(status != TX_SUCCESS) {
      return false;
  }

  return true;
}

static inline bool osal_semaphore_wait (osal_semaphore_t sem_hdl, uint32_t msec)
{
  ULONG ticks;
  UINT status;

  if(msec == OSAL_TIMEOUT_WAIT_FOREVER) {
      ticks = TX_WAIT_FOREVER;
  } else {
      ticks = ClockP_usecToTicks(msec * 1000u);
  }

  status = tx_semaphore_get(sem_hdl, ticks);
  if(status != TX_SUCCESS) {
      return false;
  }

  return true;
}

static inline void osal_semaphore_reset(osal_semaphore_t const sem_hdl)
{
  tx_semaphore_delete(sem_hdl);
}

/*--------------------------------------------------------------------+
* MUTEX API (priority inheritance)
* --------------------------------------------------------------------*/
typedef TX_MUTEX osal_mutex_def_t;
typedef TX_MUTEX* osal_mutex_t;

static inline osal_mutex_t osal_mutex_create(osal_mutex_def_t* mdef)
{
  UINT status;

  status = tx_mutex_create(mdef, "cdn_osal mutex", TX_TRUE);
  if(status != TX_SUCCESS) {
      return NULL;
  }

  return mdef;
}

static inline bool osal_mutex_lock (osal_mutex_t mutex_hdl, uint32_t msec)
{
  ULONG ticks;
  UINT status;

  if(msec == OSAL_TIMEOUT_WAIT_FOREVER) {
      ticks = TX_WAIT_FOREVER;
  } else {
      ticks = ClockP_usecToTicks(msec * 1000u);
  }

  status = tx_mutex_get(mutex_hdl, ticks);
  if(status != TX_SUCCESS) {
      return false;
  }

  return true;
}

static inline bool osal_mutex_unlock(osal_mutex_t mutex_hdl)
{
  UINT status;

  status = tx_mutex_put(mutex_hdl);
  if(status != TX_SUCCESS) {
      return false;
  }

  return true;
}

/*--------------------------------------------------------------------+
* QUEUE API
* -------------------------------------------------------------------*/

/* role device/host is used by OS NONE for mutex (disable usb isr) only*/
#define OSAL_QUEUE_DEF(_role, _name, _depth, _type) \
  static _type _name##_##buf[_depth];\
  osal_queue_def_t _name = { .depth = _depth, .item_sz = sizeof(_type), .buf = _name##_##buf };

typedef struct
{
  uint16_t depth;
  uint16_t item_sz;
  void*    buf;

  TX_QUEUE sq;
}osal_queue_def_t;

typedef TX_QUEUE* osal_queue_t;

static inline osal_queue_t osal_queue_create(osal_queue_def_t* qdef)
{
  ULONG item_size;
  UINT status;

  if(qdef == NULL) {
      return NULL;
  }

  item_size = ((qdef->item_sz + 3u) / 4u);

  status = tx_queue_create(&qdef->sq, "cdn_osal queue", item_size, qdef->buf, ((qdef->depth * qdef->item_sz)  / sizeof(ULONG)));
  if(status != TX_SUCCESS) {
      return NULL;
  }

  return &qdef->sq;
}

static inline bool osal_queue_receive(osal_queue_t qhdl, void* data)
{
  UINT status;

  status = tx_queue_receive(qhdl, data, TX_WAIT_FOREVER);
  if(status != TX_SUCCESS) {
      return false;
  }

  return true;
}

static inline bool osal_queue_send(osal_queue_t qhdl, void const * data, bool in_isr)
{
  UINT status;
  ULONG wait;

  (void)in_isr;

  if(in_isr == true) {
      wait = TX_NO_WAIT;
  } else {
      wait = TX_WAIT_FOREVER;
  }

  status = tx_queue_send(qhdl, (void *)data, wait);
  if(status != TX_SUCCESS) {
      return false;
  }

  return true;
}

static inline bool osal_queue_empty(osal_queue_t qhdl)
{
  if(qhdl == NULL) {
      return true;
  }

  if(qhdl->tx_queue_enqueued == 0u) {
      return true;
  } else {
      return false;
  }
}

#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_OSAL_FREERTOS_H_ */
