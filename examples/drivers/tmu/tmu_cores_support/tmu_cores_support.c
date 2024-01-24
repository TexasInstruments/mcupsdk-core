
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



#include <stdint.h>
#include <mathlib/trig/ti_arm_trig.h>
#include <mathlib/trig/ti_tmu_trig.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"


typedef struct {

    Float32 alpha;
    Float32 beta;
    Float32 angle;
    Float32 sD;
    Float32 sQ;
    Float32 sine_val;
    Float32 cos_val;


} PARK;

typedef struct {

    Float32 alpha;
    Float32 beta;
    Float32 angle;
    Float32 sD;
    Float32 sQ;
    Float32 sine_val;
    Float32 cos_val;


} INV_PARK;

#define PARK_DEFAULTS { 0, \
                        0, \
                        0, \
                        0, \
                        0, \
                        0, \
                        0  \
                      }

#define INVPARK_DEFAULTS { 0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0, \
                           0  \
                         }

void park(PARK *obj);
void inverse_park(INV_PARK * obj);

void tmu_cores_support_main()
{
   PARK park1;
   INV_PARK ipark1;

   park1.angle = 30;
   park1.alpha = 500;
   park1.beta = 400;

   ipark1.angle = 30;
   ipark1.sD = 150;
   ipark1.sQ = 275;

   park(&park1);
   inverse_park(&ipark1);

   DebugP_log("TMU Cores Support Test Started ... \r\n");
   DebugP_log("Park Transform value : %f , %f \r\n", park1.sD, park1.sQ);
   DebugP_log("Inverse Park Transform value : %f , %f \r\n", ipark1.alpha, ipark1.beta);
   DebugP_log("TMU Cores Support Test Passed ... \r\n");
   DebugP_log("All tests have passed!! ... \r\n");


}

void park(PARK *obj)
{

    obj->sine_val = ti_tmu_sin_pu(obj->angle);
    obj->cos_val = ti_tmu_cos_pu(obj->angle);

    obj->sD = (obj->alpha * obj->cos_val) + (obj->beta * obj->sine_val);
    obj->sQ = (obj->beta * obj->cos_val) - (obj->alpha * obj->sine_val);

}

void inverse_park(INV_PARK *obj)
{

   obj->sine_val = ti_tmu_sin_pu(obj->angle);
   obj->cos_val = ti_tmu_cos_pu(obj->angle);

   obj->alpha = (obj->sD * obj->cos_val) - (obj->sQ * obj->sine_val);
   obj->beta = (obj->sD * obj->sine_val) + (obj->sQ * obj->cos_val);
}