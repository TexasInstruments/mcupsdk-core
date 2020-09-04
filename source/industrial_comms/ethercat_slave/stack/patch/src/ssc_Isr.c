/*!
* \file ssc_Isr.c
*
* \brief
* Beckhoff SSC Integration ISR.
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

#include <ssc.h>
#include <ssc_Isr.h>

/* PRU */
#include <pru.h>

/* EtherCATSlave PRU */
#include <protocols/ethercat_slave/include/tiesc_pruss_intc_mapping.h>

/* BKHF */
#include <ecatappl.h>

/*! <!-- Description: -->
 *
 *  \brief
 *  Generic EtherCAT ISR
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pArgs_p     For future enhancement.
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
inline void SSC_ISR_ecat(void *pArgs_p)
{
    OSALUNREF_PARM(pArgs_p);

#ifndef ENABLE_PDI_TASK
#ifndef ENABLE_PDI_SWI
    PDI_Isr();
#else
    PDI_Swi();
#endif
#endif
}

/*! <!-- Description: -->
 *
 *  \brief
 *  EtherCAT SyncManager 0 ISR
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pArgs_p     For future enhancement.
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
inline void SSC_ISR_sync0(void *pArgs_p)
{
    OSALUNREF_PARM(pArgs_p);

    Sync0_Isr();
}

/*! <!-- Description: -->
 *
 *  \brief
 *  EtherCAT SyncManager 1 ISR
 *
 *  <!-- Parameters and return values: -->
 *
 *  \param[in]  pArgs_p     For future enhancement.
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
inline void SSC_ISR_sync1(void *pArgs_p)
{
    OSALUNREF_PARM(pArgs_p);

    Sync1_Isr();
}

/*! <!-- Description: -->
 *
 *  \brief
 *  External call interface for PDI_Isr
 *
 *  <!-- Example: -->
 *
 *  \par Example
 *  \code{.c}
 *  #include <ssc.h>
 *
 *  // the Call
 *  SSC_ISR_pdi();
 *  \endcode
 *
 *  <!-- Group: -->
 *
 *  \ingroup ssc
 *
 * */
inline void SSC_ISR_pdi(void)
{
    PDI_Isr();
}

//*************************************************************************************************
