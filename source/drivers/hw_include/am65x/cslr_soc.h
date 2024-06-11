/********************************************************************
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
*
*/

#ifndef CSLR_SOC_IN_H_
#define CSLR_SOC_IN_H_

/* Memory map header files */
#include <drivers/hw_include/am65x/cslr_soc_defines.h>
#include <drivers/hw_include/am65x/cslr_navss_defines.h>
#include <drivers/hw_include/am65x/cslr_soc_baseaddress.h>
#include <drivers/hw_include/am65x/cslr_soc_icss_baseaddress.h>
#include <drivers/hw_include/am65x/cslr_soc_r5_baseaddress.h>

/* Interrupt map header files */
#include <drivers/hw_include/am65x/cslr_intr_gic0.h>
#include <drivers/hw_include/am65x/cslr_intr_mcu0.h>
#include <drivers/hw_include/am65x/cslr_intr_esm0.h>
#include <drivers/hw_include/am65x/cslr_intr_esm1.h>
#include <drivers/hw_include/am65x/cslr_intr_esm2.h>
#include <drivers/hw_include/am65x/cslr_intr_polarity.h>


/* PSC, LPSC indices header file */
#include <drivers/hw_include/am65x/csl_soc_psc.h>

/* Firewall header files */
#include <drivers/hw_include/am65x/csl_soc_firewalls.h>

/* Ctrl MMR header files */
#include <drivers/hw_include/am65x/cslr_soc_ctrl_mmr.h>

#include <drivers/hw_include/am65x/csl_psilcfg_thread_map.h>

#endif /* CSLR_SOC_IN_H_ */
