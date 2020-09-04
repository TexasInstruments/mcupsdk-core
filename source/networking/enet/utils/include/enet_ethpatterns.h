/* ======================================================================
 *   Copyright (C) 2020 Texas Instruments Incorporated
 *
 *   All rights reserved. Property of Texas Instruments Incorporated.
 *   Restricted rights to use, duplicate or disclose this code are
 *   granted through contract.
 *
 *   The program may not be used without the written permission
 *   of Texas Instruments Incorporated or against the terms and conditions
 *   stipulated in the agreement under which this program has been
 *   supplied.
 * ====================================================================
 */

#ifndef ENETAPP_ETHPATTERNS_H_
#define ENETAPP_ETHPATTERNS_H_

#ifdef __cplusplus
extern "C"
{
#endif


#define ENETAPP_ETHPATTERN_LENGTH                     (1500U)

extern uint8_t Enet_DataPattern1[ENETAPP_ETHPATTERN_LENGTH];

extern uint8_t Enet_DataPattern2[ENETAPP_ETHPATTERN_LENGTH];

extern uint8_t Enet_DataPattern3[ENETAPP_ETHPATTERN_LENGTH];

extern uint8_t Enet_DataPattern4[ENETAPP_ETHPATTERN_LENGTH];


#ifdef __cplusplus
}
#endif

#endif /* ENETAPP_ETHPATTERNS_H_ */
