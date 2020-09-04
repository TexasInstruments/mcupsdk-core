/**
 * @file hsr_prp_prvmib.c
 * @brief Contains private mib and Mib2 implementation
 *
 * \par
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
 * \par
 */
/* ========================================================================== */
/*                             Includes                                       */
/* ========================================================================== */

#include "hsr_prp_prvmib.h"

#if LWIP_SNMP

#include <string.h>
#include <stdio.h>

#include "lwip/snmp.h"
#include "lwip/apps/snmp.h"
#include "lwip/apps/snmp_core.h"
#include "lwip/sys.h"
#include "lwip/netif.h"


#include "lwip/apps/snmp_table.h"
#include "lwip/apps/snmp_scalar.h"
/*PRP Related Header files*/
#include <industrial_comms/hsr_prp/icss_fwhal/hsrPrp_red_snmp.h>
#include <hsr_prp_menu.h>

#include <networking/icss_emac/icss_emac.h>
#include <networking/icss_emac/source/icss_emac_local.h>
#include <networking/icss_emac/lwipif/inc/lwip2icss_emac.h>
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
extern ICSS_EMAC_Handle emachandle;
extern PRUICSS_Handle prusshandle;
extern hsrPrpHandle *hsrPrphandle;
extern Lwip2Emac_Handle Lwipif_handle;
struct netif *pvt_netif;

u8_t ocstr_syscontact[] = "Texas Instruments Incorporated";
u8_t ocstr_syslocation[] = "Texas Instruments";
u8_t ocstr_sysdescr[] = "PRP_Demo";
u8_t ocstr_sysname[] = "TI Application: HSR/PRP";
const char snmp_rd_wr_comunity[] = "private";
u16_t     ocstrlen;

RED_NODE_TABLE  nodeTable;
ICSS_EMAC_PruStatistics pruStats;
ICSS_EMAC_HostStatistics *hostStatsPtr;

/* mib-2.system.sysDescr */
static const u8_t   sysdescr_default[] = "prp";
static const u8_t  *sysdescr           = sysdescr_default;
static const u16_t *sysdescr_len       = NULL; /* use strlen for determining len */

/** mib-2.system.sysContact */
static const u8_t   syscontact_default[]     = "Texas Instruments Incorporated";
static const u8_t  *syscontact               = syscontact_default;
static const u16_t *syscontact_len           = NULL; /* use strlen for determining len */
static u8_t        *syscontact_wr            = NULL; /* if writable, points to the same buffer as syscontact (required for correct constness) */
static u16_t       *syscontact_wr_len        = NULL; /* if writable, points to the same buffer as syscontact_len (required for correct constness) */
static u16_t        syscontact_bufsize       = 0;    /* 0=not writable */

/** mib-2.system.sysName */
static const u8_t   sysname_default[]        = "TI Application: HSR/PRP";
static const u8_t  *sysname                  = sysname_default;
static const u16_t *sysname_len              = NULL; /* use strlen for determining len */
static u8_t        *sysname_wr               = NULL; /* if writable, points to the same buffer as sysname (required for correct constness) */
static u16_t       *sysname_wr_len           = NULL; /* if writable, points to the same buffer as sysname_len (required for correct constness) */
static u16_t        sysname_bufsize          = 0;    /* 0=not writable */

/** mib-2.system.sysLocation */
static const u8_t   syslocation_default[]    = "Texas Instruments";
static const u8_t  *syslocation              = syslocation_default;
static const u16_t *syslocation_len           = NULL; /* use strlen for determining len */
static u8_t        *syslocation_wr            = NULL; /* if writable, points to the same buffer as syslocation (required for correct constness) */
static u16_t       *syslocation_wr_len        = NULL; /* if writable, points to the same buffer as syslocation_len (required for correct constness) */
static u16_t        syslocation_bufsize       = 0;    /* 0=not writable */
/* export MIB */
const struct snmp_mib prp_prv;
const struct snmp_mib prp_mib2;
static const struct snmp_mib *mibs[] = {
  &prp_mib2,
  &prp_prv
};
struct ifEntry
{
    u32_t ifIndex;
    void * ifDescr;
    u32_t ifType;
    u32_t ifMtu;
    u32_t ifSpeed;
    u32_t ifPhysAddress;
    u32_t ifAdminStatus;
    u32_t ifOperStatus;
    u32_t ifLastChange;
    u32_t ifInOctets;
    u32_t ifInUcastPkts;
    u32_t ifInNUcastPkts;
    u32_t ifInDiscards;
    u32_t ifInErrors;
    u32_t ifInUnknownProtos;
    u32_t ifOutOctets;
    u32_t ifOutUcastPkts;
    u32_t ifOutNUcastPkts;
    u32_t ifOutDiscards;
    u32_t ifOutErrors;
    u32_t ifOutQLen;
    u32_t ifSpecific;
};
/* ========================================================================== */
/*                           Macro & Type Defs                                */
/* ========================================================================== */

#define NUMBER_OF_PORTS 3
#define MAC_LENGTH 6U
#define MIB2_COPY_SYSUPTIME_TO(ptrToVal) (*(ptrToVal) = (sys_now() / 10))
#define SNMP_SYSSERVICES ((1 << 6) | (1 << 3) | ((IP_FORWARD) << 2))

/* ========================================================================== */
/*                       Function Declarations                                */
/* ========================================================================== */
static s16_t lreConfigurationGeneralGroup_get_value(const struct snmp_scalar_array_node_def *node, void *value);
static s16_t lreConfigurationInterfaces_get_value(struct snmp_node_instance* instance, void* value);
static snmp_err_t lreConfigurationInterfaces_set_value(struct snmp_node_instance* instance, u16_t len, void *value);
static snmp_err_t lreConfigurationInterfaces_get_cell_instance(const u32_t* column, const u32_t* row_oid, u8_t row_oid_len, struct snmp_node_instance* cell_instance);
static snmp_err_t lreConfigurationInterfaces_get_next_cell_instance(const u32_t* column, struct snmp_obj_id* row_oid, struct snmp_node_instance* cell_instance);
static snmp_err_t lreInterfaceStatsTable_get_cell_instance(const u32_t* column, const u32_t* row_oid, u8_t row_oid_len, struct snmp_node_instance* cell_instance);
static snmp_err_t lreInterfaceStatsTable_get_next_cell_instance(const u32_t* column, struct snmp_obj_id* row_oid, struct snmp_node_instance* cell_instance);
static s16_t lreInterfaceStatsTable_get_value(struct snmp_node_instance* instance, void* value);
static snmp_err_t lreNodesTable_get_cell_instance(const u32_t* column, const u32_t* row_oid, u8_t row_oid_len, struct snmp_node_instance* cell_instance);
static snmp_err_t lreNodesTable_get_next_cell_instance(const u32_t* column, struct snmp_obj_id* row_oid, struct snmp_node_instance* cell_instance);
static s16_t lreNodesTable_get_value(struct snmp_node_instance* instance, void* value);
static s16_t system_get_value(const struct snmp_scalar_array_node_def *node, void *value);
static snmp_err_t system_set_test(const struct snmp_scalar_array_node_def *node, u16_t len, void *value);
static snmp_err_t system_set_value(const struct snmp_scalar_array_node_def *node, u16_t len, void *value);
static snmp_err_t interfaces_Table_get_cell_instance(const u32_t *column, const u32_t *row_oid, u8_t row_oid_len, struct snmp_node_instance *cell_instance);
static snmp_err_t interfaces_Table_get_next_cell_instance(const u32_t *column, struct snmp_obj_id *row_oid, struct snmp_node_instance *cell_instance);
static s16_t interfaces_Table_get_value(struct snmp_node_instance *instance, void *value);
static snmp_err_t interfaces_Table_set_test(struct snmp_node_instance *instance, u16_t len, void *value);
static snmp_err_t interfaces_Table_set_value(struct snmp_node_instance *instance, u16_t len, void *value);
static s16_t interfaces_get_value(struct snmp_node_instance *instance, void *value);

/************************************************************************************
 *               Interface MIB
 ************************************************************************************/

struct   ifEntry ifp[NUMBER_OF_PORTS];

/* list of allowed value ranges for incoming OID */
static const struct snmp_oid_range interfaces_Table_oid_ranges[] = {
  { 0, 4 }
};

static const struct snmp_table_col_def interfaces_Table_columns[] = {
  {  1, SNMP_ASN1_TYPE_INTEGER,      SNMP_NODE_INSTANCE_READ_ONLY }, /* ifIndex */
  {  2, SNMP_ASN1_TYPE_OCTET_STRING, SNMP_NODE_INSTANCE_READ_ONLY }, /* ifDescr */
  {  3, SNMP_ASN1_TYPE_INTEGER,      SNMP_NODE_INSTANCE_READ_ONLY }, /* ifType */
  {  4, SNMP_ASN1_TYPE_INTEGER,      SNMP_NODE_INSTANCE_READ_ONLY }, /* ifMtu */
  {  5, SNMP_ASN1_TYPE_GAUGE,        SNMP_NODE_INSTANCE_READ_ONLY }, /* ifSpeed */
  {  6, SNMP_ASN1_TYPE_OCTET_STRING, SNMP_NODE_INSTANCE_READ_ONLY }, /* ifPhysAddress */
#if !SNMP_SAFE_REQUESTS
  {  7, SNMP_ASN1_TYPE_INTEGER,      SNMP_NODE_INSTANCE_READ_WRITE }, /* ifAdminStatus */
#else
  {  7, SNMP_ASN1_TYPE_INTEGER,      SNMP_NODE_INSTANCE_READ_ONLY }, /* ifAdminStatus */
#endif
  {  8, SNMP_ASN1_TYPE_INTEGER,      SNMP_NODE_INSTANCE_READ_ONLY }, /* ifOperStatus */
  {  9, SNMP_ASN1_TYPE_TIMETICKS,    SNMP_NODE_INSTANCE_READ_ONLY }, /* ifLastChange */
  { 10, SNMP_ASN1_TYPE_COUNTER,      SNMP_NODE_INSTANCE_READ_ONLY }, /* ifInOctets */
  { 11, SNMP_ASN1_TYPE_COUNTER,      SNMP_NODE_INSTANCE_READ_ONLY }, /* ifInUcastPkts */
  { 12, SNMP_ASN1_TYPE_COUNTER,      SNMP_NODE_INSTANCE_READ_ONLY }, /* ifInNUcastPkts */
  { 13, SNMP_ASN1_TYPE_COUNTER,      SNMP_NODE_INSTANCE_READ_ONLY }, /* ifInDiscarts */
  { 14, SNMP_ASN1_TYPE_COUNTER,      SNMP_NODE_INSTANCE_READ_ONLY }, /* ifInErrors */
  { 15, SNMP_ASN1_TYPE_COUNTER,      SNMP_NODE_INSTANCE_READ_ONLY }, /* ifInUnkownProtos */
  { 16, SNMP_ASN1_TYPE_COUNTER,      SNMP_NODE_INSTANCE_READ_ONLY }, /* ifOutOctets */
  { 17, SNMP_ASN1_TYPE_COUNTER,      SNMP_NODE_INSTANCE_READ_ONLY }, /* ifOutUcastPkts */
  { 18, SNMP_ASN1_TYPE_COUNTER,      SNMP_NODE_INSTANCE_READ_ONLY }, /* ifOutNUcastPkts */
  { 19, SNMP_ASN1_TYPE_COUNTER,      SNMP_NODE_INSTANCE_READ_ONLY }, /* ifOutDiscarts */
  { 20, SNMP_ASN1_TYPE_COUNTER,      SNMP_NODE_INSTANCE_READ_ONLY }, /* ifOutErrors */
  { 21, SNMP_ASN1_TYPE_GAUGE,        SNMP_NODE_INSTANCE_READ_ONLY }, /* ifOutQLen */
  { 22, SNMP_ASN1_TYPE_OBJECT_ID,    SNMP_NODE_INSTANCE_READ_ONLY }  /* ifSpecific */
};

#if !SNMP_SAFE_REQUESTS
static const struct snmp_table_node interfaces_Table = SNMP_TABLE_CREATE(
      2, interfaces_Table_columns,
      interfaces_Table_get_cell_instance, interfaces_Table_get_next_cell_instance,
      interfaces_Table_get_value, interfaces_Table_set_test, interfaces_Table_set_value);
#else
static const struct snmp_table_node interfaces_Table = SNMP_TABLE_CREATE(
      2, interfaces_Table_columns,
      interfaces_Table_get_cell_instance, interfaces_Table_get_next_cell_instance,
      interfaces_Table_get_value, NULL, NULL);
#endif

static const struct snmp_scalar_node interfaces_Number = SNMP_SCALAR_CREATE_NODE_READONLY(1, SNMP_ASN1_TYPE_INTEGER, interfaces_get_value);

static const struct snmp_node *const interface_nodes[] = {
  &interfaces_Number.node.node,
  &interfaces_Table.node.node
};
const struct snmp_tree_node snmp_mib2_interface_root = SNMP_CREATE_TREE_NODE(2, interface_nodes);

/************************************************************************************
 *               System MIB
 ************************************************************************************/
static const struct snmp_scalar_array_node_def system_nodes[] = {
  {1, SNMP_ASN1_TYPE_OCTET_STRING, SNMP_NODE_INSTANCE_READ_ONLY},  /* sysDescr */
  {2, SNMP_ASN1_TYPE_OBJECT_ID,    SNMP_NODE_INSTANCE_READ_ONLY},  /* sysObjectID */
  {3, SNMP_ASN1_TYPE_TIMETICKS,    SNMP_NODE_INSTANCE_READ_ONLY},  /* sysUpTime */
  {4, SNMP_ASN1_TYPE_OCTET_STRING, SNMP_NODE_INSTANCE_READ_WRITE}, /* sysContact */
  {5, SNMP_ASN1_TYPE_OCTET_STRING, SNMP_NODE_INSTANCE_READ_WRITE}, /* sysName */
  {6, SNMP_ASN1_TYPE_OCTET_STRING, SNMP_NODE_INSTANCE_READ_WRITE}, /* sysLocation */
  {7, SNMP_ASN1_TYPE_INTEGER,      SNMP_NODE_INSTANCE_READ_ONLY}   /* sysServices */
};
const struct snmp_scalar_array_node snmp_mib2_system_node = SNMP_SCALAR_CREATE_ARRAY_NODE(1, system_nodes, system_get_value, system_set_test, system_set_value);
/************************************************************************************
 *               PRP PRIV MIB
 ************************************************************************************/
/*----------------------------------------------lreStatistics----------------------------------*/
static const struct snmp_oid_range lreInterfaceStatsTable_oid_ranges[] = {
  { 0, 1 },
};

static const struct snmp_oid_range lreNodesTable_oid_ranges[] = {
  { 0, 0xff },
};

// static const struct snmp_oid_range lreProxyNodeTable_oid_ranges[] = {
//   { 0, 0xff },
// };

// static snmp_err_t lreProxyNodeTable_get_cell_instance(const u32_t* column, const u32_t* row_oid, u8_t row_oid_len, struct snmp_node_instance* cell_instance);
// static snmp_err_t lreProxyNodeTable_get_next_cell_instance(const u32_t* column, struct snmp_obj_id* row_oid, struct snmp_node_instance* cell_instance);
// static s16_t      lreProxyNodeTable_get_value(struct snmp_node_instance* instance, void* value);
/* lreStatistics.lreStatisticsInterfaceGroup.lreStatisticsInterfaces.lreProxyNodeTable *//*lreProxyNodeTable .1.0.62439.2.21.1.1.0.3.1.{1,2} */
// static const struct snmp_table_col_def lreProxyNodeTable_col[] = {
//   {  1, SNMP_ASN1_TYPE_UNSIGNED32,    SNMP_NODE_INSTANCE_READ_ONLY },         /* lreProxyNodesIndex */
//   {  2, SNMP_ASN1_TYPE_OCTET_STRING,  SNMP_NODE_INSTANCE_READ_ONLY },        /* lreProxyNodesMacAddress */
// };

/* lreStatistics.lreStatisticsInterfaceGroup.lreStatisticsInterfaces.lreNodesTable*//*lreNodesTable .1.0.62439.2.21.1.1.0.2.1.{1...5} */
static const struct snmp_table_col_def lreNodesTable_col[] = {
    {  1, SNMP_ASN1_TYPE_UNSIGNED32,    SNMP_NODE_INSTANCE_READ_ONLY },      /* lreNodesIndex */
    {  2, SNMP_ASN1_TYPE_OCTET_STRING,  SNMP_NODE_INSTANCE_READ_ONLY },      /* lreNodesMacAddress */
    {  3, SNMP_ASN1_TYPE_TIMETICKS,     SNMP_NODE_INSTANCE_READ_ONLY },      /* lreTimeLastSeenA */
    {  4, SNMP_ASN1_TYPE_TIMETICKS,     SNMP_NODE_INSTANCE_READ_ONLY },      /* lreTimeLastSeenB */
    {  5, SNMP_ASN1_TYPE_INTEGER,       SNMP_NODE_INSTANCE_READ_ONLY },      /* lreRemNodeType */
};

/* lreStatistics.lreStatisticsInterfaceGroup.lreStatisticsInterfaces.lreInterfaceStatsTable *//*lreInterfaceStatsTable .1.0.62439.2.21.1.1.0.1.1.{1...26} */
static const struct snmp_table_col_def lreInterfaceStatsTable_col[] = {
    {  1, SNMP_ASN1_TYPE_UNSIGNED32,    SNMP_NODE_INSTANCE_READ_ONLY },      /* lreInterfacesStatsIndex */
    {  2, SNMP_ASN1_TYPE_COUNTER,       SNMP_NODE_INSTANCE_READ_ONLY },      /* lreCountTxA */
    {  3, SNMP_ASN1_TYPE_COUNTER,       SNMP_NODE_INSTANCE_READ_ONLY },      /* lreCountTxB */
    {  4, SNMP_ASN1_TYPE_COUNTER,       SNMP_NODE_INSTANCE_READ_ONLY },      /* lreCountTxC */
    {  5, SNMP_ASN1_TYPE_COUNTER,       SNMP_NODE_INSTANCE_READ_ONLY },      /* lreCntErrWrongLanA */
    {  6, SNMP_ASN1_TYPE_COUNTER,       SNMP_NODE_INSTANCE_READ_ONLY },      /* lreCntErrWrongLanB */
    {  7, SNMP_ASN1_TYPE_COUNTER,       SNMP_NODE_INSTANCE_READ_ONLY },      /* lreCntErrWrongLanC */
    {  8, SNMP_ASN1_TYPE_COUNTER,       SNMP_NODE_INSTANCE_READ_ONLY },      /* lreCountRxA */
    {  9, SNMP_ASN1_TYPE_COUNTER,       SNMP_NODE_INSTANCE_READ_ONLY },      /* lreCountRxB*/
    { 10, SNMP_ASN1_TYPE_COUNTER,       SNMP_NODE_INSTANCE_READ_ONLY },      /* lreCountRxC */
    { 11, SNMP_ASN1_TYPE_COUNTER,       SNMP_NODE_INSTANCE_READ_ONLY },      /* lreCntErrorsA */
    { 12, SNMP_ASN1_TYPE_COUNTER,       SNMP_NODE_INSTANCE_READ_ONLY },      /* lreCntErrorsB */
    { 13, SNMP_ASN1_TYPE_COUNTER,       SNMP_NODE_INSTANCE_READ_ONLY },       /* lreCntErrorsC */
    { 14, SNMP_ASN1_TYPE_INTEGER,       SNMP_NODE_INSTANCE_READ_ONLY },      /* lreCntNodes */
    { 15, SNMP_ASN1_TYPE_INTEGER,       SNMP_NODE_INSTANCE_READ_ONLY },      /* lreCntProxyNodes */
    { 16, SNMP_ASN1_TYPE_COUNTER,       SNMP_NODE_INSTANCE_READ_ONLY },      /* lreCntUniqueA */
    { 17, SNMP_ASN1_TYPE_COUNTER,       SNMP_NODE_INSTANCE_READ_ONLY },      /* lreCntUniqueB */
    { 18, SNMP_ASN1_TYPE_COUNTER,       SNMP_NODE_INSTANCE_READ_ONLY },      /* lreCntUniqueC */
    { 19, SNMP_ASN1_TYPE_COUNTER,       SNMP_NODE_INSTANCE_READ_ONLY },      /* lreCntDuplicateA */
    { 20, SNMP_ASN1_TYPE_COUNTER,       SNMP_NODE_INSTANCE_READ_ONLY },      /* lreCntDuplicateB */
    { 21, SNMP_ASN1_TYPE_COUNTER,       SNMP_NODE_INSTANCE_READ_ONLY },      /* lreCntDuplicateC */
    { 22, SNMP_ASN1_TYPE_COUNTER,       SNMP_NODE_INSTANCE_READ_ONLY },      /* lreCntMultiA */
    { 23, SNMP_ASN1_TYPE_COUNTER,       SNMP_NODE_INSTANCE_READ_ONLY },       /* lreCntMultiB */
    { 24, SNMP_ASN1_TYPE_COUNTER,       SNMP_NODE_INSTANCE_READ_ONLY },      /* lreCntMultiC */
    { 25, SNMP_ASN1_TYPE_COUNTER,       SNMP_NODE_INSTANCE_READ_ONLY },      /* lreCntOwnRxA */
    { 26, SNMP_ASN1_TYPE_COUNTER,       SNMP_NODE_INSTANCE_READ_ONLY },      /* lreCntOwnRxB */
};
/* lreStatistics.lreStatisticsInterfaceGroup.lreStatisticsInterfaces.lreProxyNodeTable .1.0.62439.2.21.1.1.0.0.3 */
// static const struct snmp_table_node lreProxyNodeTable = SNMP_TABLE_CREATE(
//   3, lreProxyNodeTable_col, lreProxyNodeTable_get_cell_instance,  lreProxyNodeTable_get_next_cell_instance,
//   lreProxyNodeTable_get_value, NULL, NULL);

/* lreStatistics.lreStatisticsInterfaceGroup.lreStatisticsInterfaces.lreNodesTable .1.0.62439.2.21.1.1.0.0.2.1 */
static const struct snmp_table_node lreNodesTable = SNMP_TABLE_CREATE(
  2, lreNodesTable_col, lreNodesTable_get_cell_instance,  lreNodesTable_get_next_cell_instance,
  lreNodesTable_get_value, NULL, NULL);

/* lreStatistics.lreStatisticsInterfaceGroup.lreStatisticsInterfaces.lreInterfaceStatsTable .1.0.62439.2.21.1.1.0.0.1 */
static const struct snmp_table_node lreInterfaceStatsTable = SNMP_TABLE_CREATE(
  1, lreInterfaceStatsTable_col, lreInterfaceStatsTable_get_cell_instance,  lreInterfaceStatsTable_get_next_cell_instance,
  lreInterfaceStatsTable_get_value, NULL, NULL);

/* lreStatistics.lreStatisticsInterfaceGroup.lreStatisticsInterfaces .1.0.62439.2.21.1.1.0 */
static const struct snmp_node* const lreStatisticsInterfaces_nodes[] = {
  &lreInterfaceStatsTable.node.node,
  &lreNodesTable.node.node,
  // &lreProxyNodeTableGroup.node.node,
};
static const struct snmp_tree_node lreStatisticsInterfaceGroupNode1 = SNMP_CREATE_TREE_NODE(0, lreStatisticsInterfaces_nodes);
/* lreStatistics.lreStatisticsInterfaceGroup .1.0.62439.2.21.1.1 */
static const struct snmp_node* const lreStatisticsInterfaceGroup_nodes[] = {
  &lreStatisticsInterfaceGroupNode1.node,
};
static const struct snmp_tree_node lreStatisticsInterfaceGroup = SNMP_CREATE_TREE_NODE(1, lreStatisticsInterfaceGroup_nodes);
/* lreStatistics .1.0.62439.2.21.1 */
static const struct snmp_node* const lreStatistics_nodes[] = {
  &lreStatisticsInterfaceGroup.node,
};
static const struct snmp_tree_node lreStatistics = SNMP_CREATE_TREE_NODE(1, lreStatistics_nodes);

/*----------------------------------------------lreConfiguration----------------------------------*/
/* lreConfiguration.lreConfigurationGeneralGroup.{1,2}*/ /*1.0.62439.2.21.0.0.{1,2}*/
static const struct snmp_scalar_array_node_def lreConfigurationGeneralGroup_col[] = {
    {  1, SNMP_ASN1_TYPE_OCTET_STRING,    SNMP_NODE_INSTANCE_READ_ONLY },   /* lreManufacturerName */
    {  2, SNMP_ASN1_TYPE_INTEGER,         SNMP_NODE_INSTANCE_READ_ONLY },   /* lreInterfaceCount */
};

/* lreCfg_GenGrp .1.0.62439.2.21.0.0 */
static const struct snmp_scalar_array_node lreConfigurationGeneralGroup = SNMP_SCALAR_CREATE_ARRAY_NODE(
  0, lreConfigurationGeneralGroup_col,
  lreConfigurationGeneralGroup_get_value, NULL, NULL);

/* lreConfiguration.lreConfigurationInterfaceGroup.lreConfigurationInterfaces.{1...18} */ /*(1.0.62439.2.21.0.1.0.1.1.{1...18)*/
static const struct snmp_table_col_def lreConfigurationInterfaces_col[] = {
    {  1, SNMP_ASN1_TYPE_UNSIGNED32,    SNMP_NODE_INSTANCE_READ_ONLY },      /* lreInterfaceConfigIndex */
    {  2, SNMP_ASN1_TYPE_INTEGER,       SNMP_NODE_INSTANCE_READ_WRITE },      /* lreRowStatus */
    {  3, SNMP_ASN1_TYPE_INTEGER,       SNMP_NODE_INSTANCE_READ_WRITE },      /* lreNodeType */
    {  4, SNMP_ASN1_TYPE_OCTET_STRING,  SNMP_NODE_INSTANCE_READ_WRITE },      /* lreNodeName */
    {  5, SNMP_ASN1_TYPE_OCTET_STRING,  SNMP_NODE_INSTANCE_READ_ONLY },      /* lreVersionName */
    {  6, SNMP_ASN1_TYPE_OCTET_STRING,  SNMP_NODE_INSTANCE_READ_WRITE },      /* lreMacAddress */
    {  7, SNMP_ASN1_TYPE_INTEGER,       SNMP_NODE_INSTANCE_READ_WRITE },      /* lrePortAdminStateA */
    {  8, SNMP_ASN1_TYPE_INTEGER,       SNMP_NODE_INSTANCE_READ_WRITE },      /* lrePortAdminStateB */
    {  9, SNMP_ASN1_TYPE_INTEGER,       SNMP_NODE_INSTANCE_READ_ONLY },      /* lreLinkStatusA */
    { 10, SNMP_ASN1_TYPE_INTEGER,       SNMP_NODE_INSTANCE_READ_ONLY },      /* lreLinkStatusB */
    { 11, SNMP_ASN1_TYPE_INTEGER,       SNMP_NODE_INSTANCE_READ_WRITE },      /* lreDuplicateDiscard */
    { 12, SNMP_ASN1_TYPE_INTEGER,       SNMP_NODE_INSTANCE_READ_WRITE },      /* lreTransparentReception */
    { 13, SNMP_ASN1_TYPE_INTEGER,       SNMP_NODE_INSTANCE_READ_WRITE },       /* lreHsrLreMode */
    { 14, SNMP_ASN1_TYPE_INTEGER,       SNMP_NODE_INSTANCE_READ_WRITE },      /* lreSwitchingEndMode */
    { 15, SNMP_ASN1_TYPE_INTEGER,       SNMP_NODE_INSTANCE_READ_WRITE },      /* lreRedBoxIdentity */
    { 16, SNMP_ASN1_TYPE_INTEGER,       SNMP_NODE_INSTANCE_READ_WRITE },      /* lreEvaluateSupervision */
    { 17, SNMP_ASN1_TYPE_INTEGER,       SNMP_NODE_INSTANCE_READ_WRITE },      /* lreNodesTableClear */
    { 18, SNMP_ASN1_TYPE_INTEGER,       SNMP_NODE_INSTANCE_READ_WRITE },      /* lreProxyNodeTableClear */

};

/* lreConfiguration.lreConfigurationInterfaceGroup .1.0.62439.2.21.0.1.0.1.1 */
static const struct snmp_table_node lreConfigurationInterfaces = SNMP_TABLE_CREATE(
  1, lreConfigurationInterfaces_col, lreConfigurationInterfaces_get_cell_instance,  lreConfigurationInterfaces_get_next_cell_instance,
  lreConfigurationInterfaces_get_value, snmp_set_test_ok, lreConfigurationInterfaces_set_value);

  /* lreConfiguration.lreConfigurationInterfaceGroup.lreConfigurationInterfaces.node1 .1.0.62439.2.21.0.1.0 */
static const struct snmp_node* const lreConfigurationInterfaceGroupNode1_nodes[] = {
  &lreConfigurationInterfaces.node.node,
};
static const struct snmp_tree_node lreConfigurationInterfaceGroupNode1 = SNMP_CREATE_TREE_NODE(0, lreConfigurationInterfaceGroupNode1_nodes);
/* lreConfiguration.lreConfigurationInterfaceGroup.lreConfigurationInterfaces .1.0.62439.2.21.0.1. */
static const struct snmp_node* const lreConfigurationInterfaceGroup_nodes[] = {
  &lreConfigurationInterfaceGroupNode1.node,
};
static const struct snmp_tree_node lreConfigurationInterfaceGroup = SNMP_CREATE_TREE_NODE(1, lreConfigurationInterfaceGroup_nodes);
/* lreConfiguration .1.0.62439.2.21.0 */
static const struct snmp_node* const lreConfiguration_nodes[] = {
  &lreConfigurationGeneralGroup.node.node,
  &lreConfigurationInterfaceGroup.node
};
static const struct snmp_tree_node lreConfiguration = SNMP_CREATE_TREE_NODE(0, lreConfiguration_nodes);
/*----------------------------------------------mib_prp-----------------------------------------*/
/* lre .1.0.62439.2.21*/
static const struct snmp_node* const lre_nodes[] = {
  &lreConfiguration.node,
  &lreStatistics.node
};
static const struct snmp_tree_node lre = SNMP_CREATE_TREE_NODE(21, lre_nodes);    /*lre*/

static const u32_t prvmib_base_oid[] = { 1, 0, 62439, 2, 21 };
const struct snmp_mib prp_prv = SNMP_MIB_CREATE(prvmib_base_oid, &lre.node); /*lre*/
/************************************************************************************
 *               MIB 2
 ************************************************************************************/
static const struct snmp_node *const mib2_nodes[] = {
  &snmp_mib2_system_node.node.node,
  &snmp_mib2_interface_root.node,
};

static const struct snmp_tree_node mib2_root = SNMP_CREATE_TREE_NODE(1, mib2_nodes);

static const u32_t  mib2_base_oid_arr[] = { 1, 3, 6, 1, 2, 1 };
const struct snmp_mib prp_mib2 = SNMP_MIB_CREATE(mib2_base_oid_arr, &mib2_root.node);

/* ========================================================================== */
/*                            Local Functions                                */
/* ========================================================================== */

uint16_t numberOfInterfaces()
{
    return NUMBER_OF_PORTS;
}

char *getIfDescr(int interface)
{
    if(interface == 0)
    {
        return "TI Host Port";
    }

    else if(interface == 1)
    {
        return "TI Phy Port 1";
    }

    if(interface == 2)
    {
        return "TI Phy Port 2";
    }

    /* interface number out of bounds*/
    return 0;
}

int getIfType()
{
    return 1;
}

u32_t getIfMtu()
{
    u32_t mtu = 0;
    pvt_netif = Lwipif_handle->netif;
    if(Lwipif_handle->initDone)
    {
      mtu = pvt_netif->mtu;
    }
    return mtu;
}

u32_t getIfSpeed(int interface)
{
    ETHPHY_SpeedDuplexConfig speedDuplexConfig;
    int index = interface-1;
    int status;

    if(interface == 1)
    {
        status = ETHPHY_command(((ICSS_EMAC_Object *)emachandle->object)->ethphyHandle[index], ETHPHY_CMD_GET_SPEED_AND_DUPLEX_CONFIG, (void *)&speedDuplexConfig, sizeof(speedDuplexConfig));
    }
    if(interface == 2)
    {
      status = ETHPHY_command(((ICSS_EMAC_Object *)emachandle->object)->ethphyHandle[index], ETHPHY_CMD_GET_SPEED_AND_DUPLEX_CONFIG, (void *)&speedDuplexConfig, sizeof(speedDuplexConfig));
    }

    if(SystemP_SUCCESS != status)
    {
        speedDuplexConfig.config = ETHPHY_SPEED_DUPLEX_CONFIG_INVALID;
    }
    switch(speedDuplexConfig.config)
    {
        case  ETHPHY_SPEED_DUPLEX_CONFIG_100FD:
            return (uint32_t)Hundred_Mbps;
        case  ETHPHY_SPEED_DUPLEX_CONFIG_100HD:
            return (uint32_t)Hundred_Mbps;
        case  ETHPHY_SPEED_DUPLEX_CONFIG_10FD:
            return (uint32_t)Ten_Mbps;
        case  ETHPHY_SPEED_DUPLEX_CONFIG_10HD:
            return (uint32_t)Ten_Mbps;
        default:
          /* Use 100M Full Duplex configuration by default */
          return (uint32_t)Hundred_Mbps;
    }
    return 0;
}

void getIfStatus(u32_t *ifAdminStatus, u32_t *ifOperStatus, u32_t *ifLastChange, int port_no)
{
    int tempOperStatus;
    /*ifAdminStatus only supports 1 - UP as for now */
    *ifAdminStatus = 1;
    tempOperStatus = *ifOperStatus;

    /*ifAdminStatus is 1 and Port Link is up, then ifOperStatus is UP(1) */
    if(*ifAdminStatus == 1 && port_no == 1)
    {
        *ifOperStatus = 1;
    }

    /*ifAdminStatus is 1 and Port Link is Down, then ifOperStatus is DOWN(2) */
    else if(*ifAdminStatus == 1 && port_no == 0)
    {
        *ifOperStatus = 2;
    }

    /*ifAdminStatus is 1 and Port Link is not 1/0, then ifOperStatus is UNKNOWN(4) */
    else if(*ifAdminStatus == 1)
    {
        *ifOperStatus = 4;
    }

    /*Status changed */
    if(tempOperStatus != *ifOperStatus)
    {
      pvt_netif = Lwipif_handle->netif;
      if(Lwipif_handle->initDone)
      {
        *ifLastChange = pvt_netif->ts;
      }
    }
}

void getDriverStats(int port_no)
{
    int ret_code;
    if(port_no)
    {
        ret_code = ICSS_EMAC_ioctl(emachandle, ICSS_EMAC_IOCTL_STAT_CTRL_GET, port_no, (void *)(&pruStats));
        hostStatsPtr = (ICSS_EMAC_HostStatistics *)(((ICSS_EMAC_Object *)emachandle->object)->hostStat);
        if(ret_code < 0)
        {
            DebugP_log("ICSS_EMAC_IOCTL Failed with error code: %d\n", ret_code);
            return;
        }
    }
}
u32_t getIfInOctets(int port_no)
{
    getDriverStats(port_no);
    if(port_no == 0)
    {
        return ((hostStatsPtr)->rxOctets + (hostStatsPtr + 1)->rxOctets);
    }
    else
    {
        return pruStats.rxOctets;
    }

}

u32_t getIfInUcastPkts(int port_no)
{
    getDriverStats(port_no);
    if(port_no == 0)
    {
        return ((hostStatsPtr)->rxUcast + (hostStatsPtr + 1)->rxUcast);
    }
    else
    {
        return pruStats.rxUcast;
    }
}

u32_t getIfInNUcastPkts(int port_no)
{
    getDriverStats(port_no);
    if(port_no == 0)
    {
        return ((hostStatsPtr)->rxBcast + (hostStatsPtr + 1)->rxBcast + (hostStatsPtr)->rxMcast + (hostStatsPtr + 1)->rxMcast);
    }
    else
    {
        return pruStats.rxMcast + pruStats.rxBcast;
    }
}

u32_t getIfInDiscards(int port_no)
{
    getDriverStats(port_no);
    if(port_no == 0)
    {
        return 0;
    }
    else
    {
        return pruStats.droppedPackets;
    }
}

u32_t getIfInErrors(int port_no)
{
    getDriverStats(port_no);
    if(port_no == 0)
    {
        return 0;
    }
    else
    {
        return 0 /*pruStats.rxErrorFrames*/;
    }
}

u32_t getIfInUnkownProtos(int port_no)
{
    getDriverStats(port_no);
    if(port_no == 0)
    {
        return ((hostStatsPtr)->rxUnknownProtocol + (hostStatsPtr + 1)->rxUnknownProtocol);
    }
    else if(port_no == 1 )
    {
        return (hostStatsPtr)->rxUnknownProtocol;
    }
    if(port_no == 2)
    {
        return (hostStatsPtr + 1)->rxUnknownProtocol;
    }
    return 0;
}

u32_t getIfOutOctets(int port_no)
{
    getDriverStats(port_no);
    if(port_no == 0)
    {
        return ((hostStatsPtr)->txOctets + (hostStatsPtr + 1)->txOctets);
    }
    else
    {
        return pruStats.txOctets;
    }
}

u32_t getIfOutUcastPkts(int port_no)
{
    getDriverStats(port_no);
    if(port_no == 0)
    {
        return ((hostStatsPtr)->txUcast + (hostStatsPtr + 1)->txUcast);
    }
    else
    {
        return pruStats.txUcast;
    }
}

u32_t getIfOutNUcastPkts(int port_no)
{
    getDriverStats(port_no);
    if(port_no == 0)
    {
        return((hostStatsPtr)->txBcast + (hostStatsPtr + 1)->txBcast + (hostStatsPtr)->txMcast + (hostStatsPtr + 1)->txMcast);
    }
    else
    {
        return pruStats.txMcast + pruStats.txBcast;
    }
}

u32_t getIfOutDiscards(int port_no)
{
    /*NOT IMPLEMENTED*/
    return 0;
}

u32_t getIfOutErrors(int port_no)
{
    /*NOT IMPLEMENTED*/
    return 0;
}

void snmp_example_init(void)
{
    /*Set System related SNMP OIDs*/
    ocstrlen = strlen((const char *)ocstr_syscontact);
    snmp_mib2_set_syscontact(&ocstr_syscontact[0], &ocstrlen, ocstrlen);
    ocstrlen = strlen((const char *)ocstr_syslocation);
    snmp_mib2_set_syslocation(&ocstr_syslocation[0], &ocstrlen, ocstrlen);
    ocstrlen = strlen((const char *)ocstr_sysdescr);
    snmp_mib2_set_sysdescr(&ocstr_sysdescr[0], &ocstrlen);
    ocstrlen = strlen((const char *)ocstr_sysname);
    snmp_mib2_set_sysname(&ocstr_sysname[0], &ocstrlen, ocstrlen);

    /*Set Read and Write community Strings*/
    snmp_set_community(snmp_rd_wr_comunity);
    snmp_set_community_write(snmp_rd_wr_comunity);

    /*Provide MIBs to SNMP example*/
    snmp_set_mibs(mibs, LWIP_ARRAYSIZE(mibs));
    /*Initialise SNMP Example*/
    snmp_init();
}

static const struct snmp_oid_range lreConfigurationInterfaces_oid_ranges[] ={
  { 0, 1 }, };
static s16_t
lreConfigurationGeneralGroup_get_value(const struct snmp_scalar_array_node_def *node, void *value)
{
    u32_t *value_u32 = (u32_t *)value;
    u16_t value_len;
    switch (node->oid) {
      case 1: /* lreManufacturerName */
        value_len = strlen(getLreManufacturerName());
        MEMCPY(value, (u32_t *)getLreManufacturerName(), value_len);
        break;
      case 2: /* lreInterfaceCount */
        *value_u32 = getLreInterfaceCount();
        value_len = sizeof(*value_u32);
        break;
    }
    return value_len;
}

static s16_t
lreConfigurationInterfaces_get_value(struct snmp_node_instance* instance, void* value)
{
    u32_t *value_u32 = (u32_t *)value;
    u16_t value_len;
    // u8_t ocstr[20] = {0};
    switch (SNMP_TABLE_GET_COLUMN_FROM_OID(instance->instance_oid.id)) {
    case 1: /* lreInterfaceConfigIndex */
        *value_u32 = instance->reference.u32;
        value_len = sizeof(*value_u32);
        break;
    case 2: /* lreRowStatus */
        *value_u32 = LRE_ROW_STATUS_ACTIVE;
        value_len = sizeof(*value_u32);
        break;
    case 3: /* lreNodeType */
        *value_u32 = getLreNodeType();
        value_len = sizeof(*value_u32);
        break;
    case 4: /* lreNodeName */
        value_len = strlen(getLreNodeName());
        MEMCPY(value, (u32_t *)getLreNodeName(), value_len);
        break;
    case 5: /* lreVersionName */
        value_len = strlen(getLreVersionName());
        MEMCPY(value, (u32_t *)getLreVersionName(), value_len);
        break;
    case 6: /* lreMacAddress */
        value_len = sizeof((void*)(&(((ICSS_EMAC_Object *)emachandle->object)->macId[0])));
        memcpy(value, (void*)(&(((ICSS_EMAC_Object *)emachandle->object)->macId[0])), value_len);
        break;
    case 7: /* lrePortAdminStateA */
        *value_u32 = getLrePortAdminState(RED_PORT_A, emachandle);
        value_len = sizeof(*value_u32);
        break;
    case 8: /* lrePortAdminStateB */
        *value_u32 = getLrePortAdminState(RED_PORT_B, emachandle);
        value_len = sizeof(*value_u32);
        break;
    case 9: /* lreLinkStatusA */
        *value_u32 =  (((ICSS_EMAC_Object *)emachandle->object)->linkStatus[RED_PORT_A -
                            1] ? LRE_LINK_UP : LRE_LINK_DOWN);
        value_len = sizeof(*value_u32);
        break;
    case 10: /* lreLinkStatusB */
        *value_u32 = ((ICSS_EMAC_Object *)emachandle->object)->linkStatus[RED_PORT_B -
                            1] ? LRE_LINK_UP : LRE_LINK_DOWN;
        value_len = sizeof(*value_u32);
        break;
    case 11: /* lreDuplicateDiscard */
        *value_u32 = getLreDuplicateDiscard(prusshandle);
        value_len = sizeof(*value_u32);
        break;
    case 12: /* lreTransparentReception */
        *value_u32 = getLreTransparentReception(prusshandle);
        value_len = sizeof(*value_u32);
        break;
    case 13: /* lreHsrLreMode */
        *value_u32 = getLreHsrLREMode(prusshandle);
        value_len = sizeof(*value_u32);
        break;
    case 14: /* lreSwitchingEndMode */
        *value_u32 = getLreSwitchingEndNode();
        value_len = sizeof(*value_u32);
        break;
    case 15: /* lreRedBoxIdentity */
        *value_u32 = getLreRedBoxIdentity();
        value_len = sizeof(*value_u32);
        break;
    case 16: /* lreEvaluateSupervision */
        *value_u32 = getLreEvaluateSupervision();
        value_len = sizeof(*value_u32);
        break;
    case 17: /*  lreNodesTableClear */
        *value_u32 = LRE_TABLE_NO_OP;
        value_len = sizeof(*value_u32);
        break;
    case 18: /* lreProxyNodeTableClear */
        *value_u32 = LRE_TABLE_NO_OP;
        value_len = sizeof(*value_u32);
        break;
    default:
        *value_u32 = LRE_TABLE_NO_OP;
        value_len = sizeof(*value_u32);
        break;
    }
  return value_len;
}


static snmp_err_t
lreConfigurationInterfaces_set_value(struct snmp_node_instance* instance, u16_t len, void *value)
{
    s32_t *sint_ptr = (s32_t *)value;
    switch (SNMP_TABLE_GET_COLUMN_FROM_OID(instance->instance_oid.id)) {
    case 7:  /* lrePortAdminStateA */
        if ((*sint_ptr == LRE_PORT_NOT_ACTIVE) || (*sint_ptr == LRE_PORT_ACTIVE))
        {
            (void)setLrePortAdminState(RED_PORT_A, (LrePortAdminState_t)*sint_ptr);
        }
        else
        {
            return SNMP_ERR_WRONGVALUE;
        }
        break;
    case 8:  /* lrePortAdminStateB */
        if ((*sint_ptr == LRE_PORT_NOT_ACTIVE) || (*sint_ptr == LRE_PORT_ACTIVE))
        {
            (void)setLrePortAdminState(RED_PORT_B, (LrePortAdminState_t)*sint_ptr);
        }
        else
        {
            return SNMP_ERR_WRONGVALUE;
        }
        break;
    case 11:  /* lreDuplicateDiscard */
        if ((*sint_ptr == LRE_DD_DO_NOT_DISCARD) || (*sint_ptr == LRE_DD_DISCARD))
        {
            (void)setLreDuplicateDiscard(hsrPrphandle, (LreDuplicateDiscard_t)*sint_ptr, prusshandle);
        }
        else
        {
            return SNMP_ERR_WRONGVALUE;
        }
        break;
    case 12:  /* lreTransparentReception */
        if ((*sint_ptr == LRE_TR_REMOVE_RCT) || (*sint_ptr == LRE_TR_PASS_RCT))
        {
            (void)setLreTransparentReception((LreTransparentReception_t)*sint_ptr, prusshandle);
        }
        else
        {
            return SNMP_ERR_WRONGVALUE;
        }
        break;
    case 13:  /* lreHsrLREMode */
        if ((*sint_ptr == HSR_COMMON_MODE_H) || (*sint_ptr == HSR_COMMON_MODE_M))
        {
            (void)setLreHsrLREMode((HSRMode_t)*sint_ptr, prusshandle);
        }
        else
        {
            return SNMP_ERR_WRONGVALUE;
        }
        break;
    case 17:  /* lreNodeTableClear */
        if ((*sint_ptr == LRE_TABLE_NO_OP) || (*sint_ptr == LRE_TABLE_CLEAR))
        {
            (void)setLreNodeTableClear(hsrPrphandle, (LreTableOperation_t)*sint_ptr);
        }
        else
        {
            return SNMP_ERR_WRONGVALUE;
        }
        break;
    default:
        return SNMP_ERR_WRONGVALUE;

    }
    return SNMP_ERR_NOERROR;
}

static snmp_err_t
lreConfigurationInterfaces_get_cell_instance(const u32_t* column, const u32_t* row_oid, u8_t row_oid_len, struct snmp_node_instance* cell_instance)
{
    size_t i;
    u32_t interfaceCount;
    u32_t interface_num;

    LWIP_UNUSED_ARG(column);

    /* check if incoming OID length and if values are in plausible range */
    if(!snmp_oid_in_range(row_oid, row_oid_len, lreConfigurationInterfaces_oid_ranges, LWIP_ARRAYSIZE(lreConfigurationInterfaces_oid_ranges)))
    {
        return SNMP_ERR_NOSUCHINSTANCE;
    }
    /* get sensor index from incoming OID */
    interface_num = row_oid[0];
    interfaceCount = (u32_t)getLreInterfaceCount();
    if(interfaceCount <= 0)
    {
      return SNMP_ERR_NOSUCHINSTANCE;
    }
    /* find interface with index */
    for(i=1; i<interfaceCount+1; i++) {
        if(i == interface_num)
        {
            /* store sensor index for subsequent operations (get/test/set) */
            cell_instance->reference.u32 = (u32_t)i;
            return SNMP_ERR_NOERROR;
        }
    }

    /* not found */
    return SNMP_ERR_NOSUCHINSTANCE;
}

static snmp_err_t
lreConfigurationInterfaces_get_next_cell_instance(const u32_t* column, struct snmp_obj_id* row_oid, struct snmp_node_instance* cell_instance)
{
    size_t i;
    u32_t interfaceCount;
    struct snmp_next_oid_state state;
    u32_t result_temp[LWIP_ARRAYSIZE(lreConfigurationInterfaces_oid_ranges)];

    LWIP_UNUSED_ARG(column);

    /* init struct to search next oid */
    snmp_next_oid_init(&state, row_oid->id, row_oid->len, result_temp, LWIP_ARRAYSIZE(lreConfigurationInterfaces_oid_ranges));

    interfaceCount = (u32_t)getLreInterfaceCount();
    if(interfaceCount <= 0)
    {
      return SNMP_ERR_NOSUCHINSTANCE;
    }
  /* iterate over all possible OIDs to find the next one */
    for(i=1; i<interfaceCount+1; i++) {
      u32_t test_oid[LWIP_ARRAYSIZE(lreConfigurationInterfaces_oid_ranges)];

      test_oid[0] = i;

      /* check generated OID: is it a candidate for the next one? */
      snmp_next_oid_check(&state, test_oid, LWIP_ARRAYSIZE(lreConfigurationInterfaces_oid_ranges), (void*)i);
    }

    /* did we find a next one? */
    if(state.status == SNMP_NEXT_OID_STATUS_SUCCESS) {
      snmp_oid_assign(row_oid, state.next_oid, state.next_oid_len);
      /* store sensor index for subsequent operations (get/test/set) */
      cell_instance->reference.u32 = LWIP_CONST_CAST(u32_t, state.reference);
      return SNMP_ERR_NOERROR;
    }

    /* not found */
    return SNMP_ERR_NOSUCHINSTANCE;
}





static snmp_err_t lreInterfaceStatsTable_get_cell_instance(const u32_t* column, const u32_t* row_oid, u8_t row_oid_len, struct snmp_node_instance* cell_instance)
{
   size_t i;
    u32_t interfaceCount;
    u32_t interface_num;

    LWIP_UNUSED_ARG(column);

    /* check if incoming OID length and if values are in plausible range */
    if(!snmp_oid_in_range(row_oid, row_oid_len, lreInterfaceStatsTable_oid_ranges, LWIP_ARRAYSIZE(lreInterfaceStatsTable_oid_ranges))) {
    return SNMP_ERR_NOSUCHINSTANCE;
    }
    /* get sensor index from incoming OID */
    interface_num = row_oid[0];
    interfaceCount = (u32_t)getLreInterfaceCount();
    if(interfaceCount <= 0)
    {
      return SNMP_ERR_NOSUCHINSTANCE;
    }
    /* find interface with index */
    for(i=1; i<interfaceCount+1; i++) {
        if(i == interface_num) {
        /* store sensor index for subsequent operations (get/test/set) */
        cell_instance->reference.u32 = (u32_t)i;
        return SNMP_ERR_NOERROR;
        }
    }

    /* not found */
    return SNMP_ERR_NOSUCHINSTANCE;
}

static snmp_err_t lreInterfaceStatsTable_get_next_cell_instance(const u32_t* column, struct snmp_obj_id* row_oid, struct snmp_node_instance* cell_instance)
{
    size_t i;
    u32_t interfaceCount;
    struct snmp_next_oid_state state;
    u32_t result_temp[LWIP_ARRAYSIZE(lreInterfaceStatsTable_oid_ranges)];

    LWIP_UNUSED_ARG(column);

    /* init struct to search next oid */
    snmp_next_oid_init(&state, row_oid->id, row_oid->len, result_temp, LWIP_ARRAYSIZE(lreInterfaceStatsTable_oid_ranges));

    interfaceCount = (u32_t)getLreInterfaceCount();
    if(interfaceCount <= 0)
    {
      return SNMP_ERR_NOSUCHINSTANCE;
    }
  /* iterate over all possible OIDs to find the next one */
    for(i=1; i<interfaceCount+1; i++) {
      u32_t test_oid[LWIP_ARRAYSIZE(lreInterfaceStatsTable_oid_ranges)];

      test_oid[0] = i;

      /* check generated OID: is it a candidate for the next one? */
      snmp_next_oid_check(&state, test_oid, LWIP_ARRAYSIZE(lreInterfaceStatsTable_oid_ranges), (void*)i);
    }

    /* did we find a next one? */
    if(state.status == SNMP_NEXT_OID_STATUS_SUCCESS) {
      snmp_oid_assign(row_oid, state.next_oid, state.next_oid_len);
      /* store sensor index for subsequent operations (get/test/set) */
      cell_instance->reference.u32 = LWIP_CONST_CAST(u32_t, state.reference);
      return SNMP_ERR_NOERROR;
    }

    /* not found */
    return SNMP_ERR_NOSUCHINSTANCE;
}
static s16_t lreInterfaceStatsTable_get_value(struct snmp_node_instance* instance, void* value)
{
    s32_t *value_s32 = (s32_t *)value;
    u32_t *value_u32 = (u32_t *)value;
    s16_t value_len = sizeof(*value_s32);
    s32_t retVal = 0;

    // u8_t ocstr[20] = {0};
    switch (SNMP_TABLE_GET_COLUMN_FROM_OID(instance->instance_oid.id)) {
    case 1: /* lreInterfacesStatsIndex */
        *value_u32 = instance->reference.u32;
        value_len = sizeof(*value_u32);
        break;
    case 2: /* lreCountTxA */
        getLreInterfaceStats(LRE_IF_STATS_CNT_TX_A, &retVal, prusshandle);
        break;
    case 3: /* lreCountTxB */
        getLreInterfaceStats(LRE_IF_STATS_CNT_TX_B, &retVal, prusshandle);
        break;
    case 4: /* lreCountTxC */
        getLreInterfaceStats(LRE_IF_STATS_CNT_TX_C, &retVal, prusshandle);
        break;
    case 5: /* lreCntErrWrongLanA */
        getLreInterfaceStats(LRE_IF_STATS_CNT_ERR_WRONG_LAN_A, &retVal, prusshandle);
        break;
    case 6: /* lreCntErrWrongLanB */
        getLreInterfaceStats(LRE_IF_STATS_CNT_ERR_WRONG_LAN_B, &retVal, prusshandle);
        break;
    case 7: /* lreCntErrWrongLanC */
        getLreInterfaceStats(LRE_IF_STATS_CNT_ERR_WRONG_LAN_C, &retVal, prusshandle);
        break;
    case 8: /* lreCountRxA */
        getLreInterfaceStats(LRE_IF_STATS_CNT_RX_A, &retVal, prusshandle);
        break;
    case 9: /* lreCountRxB */
        getLreInterfaceStats(LRE_IF_STATS_CNT_RX_B, &retVal, prusshandle);
        break;
    case 10: /* lreCountRxC */
        getLreInterfaceStats(LRE_IF_STATS_CNT_RX_C, &retVal, prusshandle);
        break;
    case 11: /* lreCntErrorsA */
        getLreInterfaceStats(LRE_IF_STATS_CNT_ERRORS_A, &retVal, prusshandle);
        break;
    case 12: /* lreCntErrorsB */
        getLreInterfaceStats(LRE_IF_STATS_CNT_ERRORS_B, &retVal, prusshandle);
        break;
    case 13: /* lreCntErrorsC */
        getLreInterfaceStats(LRE_IF_STATS_CNT_ERRORS_C, &retVal, prusshandle);
        break;
    case 14: /* lreCntNodes */
        getLreInterfaceStats(LRE_IF_STATS_CNT_NODES, &retVal, prusshandle);
        break;
    case 15: /* lreCntProxyNodes */
        getLreInterfaceStats(LRE_IF_STATS_CNT_PROXY_NODES, &retVal, prusshandle);
        break;
    case 16: /* lreCntUniqueA */
        getLreInterfaceStats(LRE_IF_STATS_CNT_UNIQUE_A, &retVal, prusshandle);
        break;
    case 17: /*  lreCntUniqueB */
        getLreInterfaceStats(LRE_IF_STATS_CNT_UNIQUE_B, &retVal, prusshandle);
        break;
    case 18: /* lreCntUniqueC */
        getLreInterfaceStats(LRE_IF_STATS_CNT_UNIQUE_C, &retVal, prusshandle);
        break;
    case 19: /* lreCntDuplicateA */
        getLreInterfaceStats(LRE_IF_STATS_CNT_DUPLICATE_A, &retVal, prusshandle);
        break;
    case 20: /* lreCntDuplicateB */
        getLreInterfaceStats(LRE_IF_STATS_CNT_DUPLICATE_B, &retVal, prusshandle);
        break;
    case 21: /* lreCntDuplicateC */
        getLreInterfaceStats(LRE_IF_STATS_CNT_DUPLICATE_C, &retVal, prusshandle);
        break;
    case 22: /* lreCntMultiA */
        getLreInterfaceStats(LRE_IF_STATS_CNT_MULTI_A, &retVal, prusshandle);
        break;
    case 23: /* lreCntMultiB */
        getLreInterfaceStats(LRE_IF_STATS_CNT_MULTI_B, &retVal, prusshandle);
        break;
    case 24: /* lreCntMultiC */
        getLreInterfaceStats(LRE_IF_STATS_CNT_MULTI_C, &retVal, prusshandle);
        break;
    case 25: /* lreCntOwnRxA  */
        getLreInterfaceStats(LRE_IF_STATS_CNT_OWN_RX_A, &retVal, prusshandle);
        break;
    case 26: /* lreCntOwnRxB */
        getLreInterfaceStats(LRE_IF_STATS_CNT_OWN_RX_B, &retVal, prusshandle);
        break;
    default:
        *value_s32 = (u32_t)instance->reference.u32;
        value_len = sizeof(*value_s32);
        break;
    }
    *value_s32 = (u32_t)retVal;
    return value_len;
  }

static snmp_err_t lreNodesTable_get_cell_instance(const u32_t* column, const u32_t* row_oid, u8_t row_oid_len, struct snmp_node_instance* cell_instance)
{
    size_t i;
    s32_t nodes;
    s32_t node_num;

    LWIP_UNUSED_ARG(column);

    /* check if incoming OID length and if values are in plausible range */
    if(!snmp_oid_in_range(row_oid, row_oid_len, lreNodesTable_oid_ranges, LWIP_ARRAYSIZE(lreNodesTable_oid_ranges))) {
    return SNMP_ERR_NOSUCHINSTANCE;
    }
    /* get sensor index from incoming OID */
    node_num = row_oid[0];
    nodes = getLreNodeTableSize(prusshandle);

    if(nodes <= 0)
    {
      return SNMP_ERR_NOSUCHINSTANCE;
    }
    /* find interface with index */
    for(i=1; i<nodes+1; i++) {
        if(i == node_num) {
        /* store node index for subsequent operations (get/test/set) */
        cell_instance->reference.s32 = (s32_t)(i-1); /*Storing nodeIndex = node-1 */
        /*Initialise NodeTable to get info about specific node*/
        nodeTable.max = nodes;
        nodeTable.cnt = 0;
        nodeTable.entries = (RED_NODE_TABLE_ENTRY *)malloc(sizeof(RED_NODE_TABLE_ENTRY) * nodes);
        if(nodeTable.entries == NULL)
        {
          DebugP_log("PrvMib:Can't Allocate Nodetable\n ");
          return SNMP_ERR_NOSUCHINSTANCE;
        }

        nodes = getLreNodeTable(&nodeTable, hsrPrphandle);

        if(nodes <= 0)
        {
            free(nodeTable.entries);
            DebugP_log("PrvMib:No nodes in Nodetable\n ");
            return SNMP_ERR_NOSUCHINSTANCE;
        }
        return SNMP_ERR_NOERROR;
        }
    }

    /* not found */
  return SNMP_ERR_NOSUCHINSTANCE;
}
static snmp_err_t lreNodesTable_get_next_cell_instance(const u32_t* column, struct snmp_obj_id* row_oid, struct snmp_node_instance* cell_instance)
{
    size_t i;
    s32_t nodes;
    struct snmp_next_oid_state state;
    u32_t result_temp[LWIP_ARRAYSIZE(lreNodesTable_oid_ranges)];

    LWIP_UNUSED_ARG(column);

    /* init struct to search next oid */
    snmp_next_oid_init(&state, row_oid->id, row_oid->len, result_temp, LWIP_ARRAYSIZE(lreNodesTable_oid_ranges));

    nodes = getLreNodeTableSize(prusshandle);

    if(nodes <= 0)
    {
      return SNMP_ERR_NOSUCHINSTANCE;
    }
    /*Iterate over all oid to find next*/
    for(i=1; i<nodes+1; i++) {
    u32_t test_oid[LWIP_ARRAYSIZE(lreNodesTable_oid_ranges)];

    test_oid[0] = i;

    /* check generated OID: is it a candidate for the next one? */
    snmp_next_oid_check(&state, test_oid, LWIP_ARRAYSIZE(lreNodesTable_oid_ranges), (void*)i);
    }

  /* did we find a next one? */
    if(state.status == SNMP_NEXT_OID_STATUS_SUCCESS) {
      snmp_oid_assign(row_oid, state.next_oid, state.next_oid_len);
      /* store sensor index for subsequent operations (get/test/set) */
      cell_instance->reference.u32 = LWIP_CONST_CAST(u32_t, state.reference);
      return SNMP_ERR_NOERROR;
    }

    /* not found */
  return SNMP_ERR_NOSUCHINSTANCE;
}

static s16_t lreNodesTable_get_value(struct snmp_node_instance* instance, void* value)
{
    u32_t *value_u32 = (u32_t *)value;
    s32_t *value_s32 = (s32_t *)value;
    u16_t value_len = sizeof(*value_u32);
    s32_t nodeIndex = instance->reference.s32;
    switch (SNMP_TABLE_GET_COLUMN_FROM_OID(instance->instance_oid.id)) {
      case 1: /* lreNodesIndex */
        *value_s32 = nodeIndex;
        value_len = sizeof(*value_s32);
        break;
      case 2: /* lreNodesMacAddress */
        value_len = sizeof(nodeTable.entries[nodeIndex].src);
        MEMCPY(value, nodeTable.entries[nodeIndex].src, value_len);
        break;
      case 3: /* lreTimeLastSeenA */
        *value_u32 = nodeTable.entries[nodeIndex].timeLasSeenA;;
        value_len = sizeof(*value_u32);
        break;
      case 4: /* lreTimeLastSeenB */
        *value_u32 = nodeTable.entries[nodeIndex].timeLasSeenB;
        value_len = sizeof(*value_u32);
        break;
      case 5: /* lreRemNodeType */
        *value_u32 = getLreRemNodeType(nodeTable.entries[nodeIndex].status);
        value_len = sizeof(*value_u32);
        break;
  }
  return value_len;
}


void
snmp_mib2_set_sysdescr(const u8_t *str, const u16_t *len)
{
    if (str != NULL) {
      sysdescr     = str;
      sysdescr_len = len;
    }
}

void
snmp_mib2_set_syscontact(u8_t *ocstr, u16_t *ocstrlen, u16_t bufsize)
{
  if (ocstr != NULL) {
    syscontact         = ocstr;
    syscontact_wr      = ocstr;
    syscontact_len     = ocstrlen;
    syscontact_wr_len  = ocstrlen;
    syscontact_bufsize = bufsize;
  }
}

void
snmp_mib2_set_sysname(u8_t *ocstr, u16_t *ocstrlen, u16_t bufsize)
{
  if (ocstr != NULL) {
    sysname         = ocstr;
    sysname_wr      = ocstr;
    sysname_len     = ocstrlen;
    sysname_wr_len  = ocstrlen;
    sysname_bufsize = bufsize;
  }
}

void
snmp_mib2_set_syslocation(u8_t *ocstr, u16_t *ocstrlen, u16_t bufsize)
{
  if (ocstr != NULL) {
    syslocation         = ocstr;
    syslocation_wr      = ocstr;
    syslocation_len     = ocstrlen;
    syslocation_wr_len  = ocstrlen;
    syslocation_bufsize = bufsize;
  }
}

static s16_t
system_get_value(const struct snmp_scalar_array_node_def *node, void *value)
{
  const u8_t  *var = NULL;
  const s16_t *var_len;
  u16_t result;

  switch (node->oid) {
    case 1: /* sysDescr */
      var     = sysdescr;
      var_len = (const s16_t *)sysdescr_len;
      break;
    case 2: { /* sysObjectID */
      const struct snmp_obj_id *dev_enterprise_oid = snmp_get_device_enterprise_oid();
      MEMCPY(value, dev_enterprise_oid->id, dev_enterprise_oid->len * sizeof(u32_t));
      return dev_enterprise_oid->len * sizeof(u32_t);
    }
    case 3: /* sysUpTime */
      MIB2_COPY_SYSUPTIME_TO((u32_t *)value);
      return sizeof(u32_t);
    case 4: /* sysContact */
      var     = syscontact;
      var_len = (const s16_t *)syscontact_len;
      break;
    case 5: /* sysName */
      var     = sysname;
      var_len = (const s16_t *)sysname_len;
      break;
    case 6: /* sysLocation */
      var     = syslocation;
      var_len = (const s16_t *)syslocation_len;
      break;
    case 7: /* sysServices */
      *(s32_t *)value = SNMP_SYSSERVICES;
      return sizeof(s32_t);
    default:
      LWIP_DEBUGF(SNMP_MIB_DEBUG, ("system_get_value(): unknown id: %"S32_F"\n", node->oid));
      return 0;
  }

  /* handle string values (OID 1,4,5 and 6) */
  LWIP_ASSERT("", (value != NULL));
  if (var_len == NULL) {
    result = (s16_t)strlen((const char *)var);
  } else {
    result = *var_len;
  }
  MEMCPY(value, var, result);
  return result;
}

static snmp_err_t
system_set_test(const struct snmp_scalar_array_node_def *node, u16_t len, void *value)
{
  snmp_err_t ret = SNMP_ERR_WRONGVALUE;
  const u16_t *var_bufsize  = NULL;
  const u16_t *var_wr_len;

  LWIP_UNUSED_ARG(value);

  switch (node->oid) {
    case 4: /* sysContact */
      var_bufsize  = &syscontact_bufsize;
      var_wr_len   = syscontact_wr_len;
      break;
    case 5: /* sysName */
      var_bufsize  = &sysname_bufsize;
      var_wr_len   = sysname_wr_len;
      break;
    case 6: /* sysLocation */
      var_bufsize  = &syslocation_bufsize;
      var_wr_len   = syslocation_wr_len;
      break;
    default:
      LWIP_DEBUGF(SNMP_MIB_DEBUG, ("system_set_test(): unknown id: %"S32_F"\n", node->oid));
      return ret;
  }

  /* check if value is writable at all */
  if (*var_bufsize > 0) {
    if (var_wr_len == NULL) {
      /* we have to take the terminating 0 into account */
      if (len < *var_bufsize)
      {
          ret = SNMP_ERR_NOERROR;
      }
    }
    else
    {
        if (len <= *var_bufsize)
        {
            ret = SNMP_ERR_NOERROR;
        }
    }
  }
  else
  {
      ret = SNMP_ERR_NOTWRITABLE;
  }

  return ret;
}

static snmp_err_t
system_set_value(const struct snmp_scalar_array_node_def *node, u16_t len, void *value)
{
  u8_t  *var_wr = NULL;
  u16_t *var_wr_len;

  switch (node->oid) {
    case 4: /* sysContact */
      var_wr     = syscontact_wr;
      var_wr_len = syscontact_wr_len;
      break;
    case 5: /* sysName */
      var_wr     = sysname_wr;
      var_wr_len = sysname_wr_len;
      break;
    case 6: /* sysLocation */
      var_wr     = syslocation_wr;
      var_wr_len = syslocation_wr_len;
      break;
    default:
      LWIP_DEBUGF(SNMP_MIB_DEBUG, ("system_set_value(): unknown id: %"S32_F"\n", node->oid));
      return SNMP_ERR_GENERROR;
  }

  /* no need to check size of target buffer, this was already done in set_test method */
  LWIP_ASSERT("", var_wr != NULL);
  MEMCPY(var_wr, value, len);

  if (var_wr_len == NULL)
  {
      /* add terminating 0 */
      var_wr[len] = 0;
  }
  else
  {
      *var_wr_len = len;
  }

  return SNMP_ERR_NOERROR;
}

static s16_t
interfaces_get_value(struct snmp_node_instance *instance, void *value)
{
  if (instance->node->oid == 1)
  {
      s32_t *sint_ptr = (s32_t *)value;
      *sint_ptr = numberOfInterfaces();;
      return sizeof(*sint_ptr);
  }

  return 0;
}

static snmp_err_t
interfaces_Table_get_cell_instance(const u32_t *column, const u32_t *row_oid, u8_t row_oid_len, struct snmp_node_instance *cell_instance)
{
  size_t i;
  s32_t interfaces;
  u32_t ifIndex;

  LWIP_UNUSED_ARG(column);

  /* check if incoming OID length and if values are in plausible range */
  if (!snmp_oid_in_range(row_oid, row_oid_len, interfaces_Table_oid_ranges, LWIP_ARRAYSIZE(interfaces_Table_oid_ranges))) {
    return SNMP_ERR_NOSUCHINSTANCE;
  }

  /* get netif index from incoming OID */
  ifIndex = row_oid[0];
  interfaces = numberOfInterfaces();
  /* find netif with index */
  for(i = 0; i < interfaces; i++)
    {
    if (i == ifIndex) {
      /* store netif pointer for subsequent operations (get/test/set) */
      cell_instance->reference.s32 = (s32_t)(i);
      return SNMP_ERR_NOERROR;
    }
  }
  if(i >= interfaces)
    {
        return SNMP_ERR_NOSUCHINSTANCE;
    }
  /* not found */
  return SNMP_ERR_NOSUCHINSTANCE;
}

static snmp_err_t
interfaces_Table_get_next_cell_instance(const u32_t *column, struct snmp_obj_id *row_oid, struct snmp_node_instance *cell_instance)
{
    size_t i;
    s32_t interfaces;
    struct snmp_next_oid_state state;
    u32_t result_temp[LWIP_ARRAYSIZE(interfaces_Table_oid_ranges)];

    LWIP_UNUSED_ARG(column);

    /* init struct to search next oid */
    snmp_next_oid_init(&state, row_oid->id, row_oid->len, result_temp, LWIP_ARRAYSIZE(interfaces_Table_oid_ranges));

    /* iterate over all possible OIDs to find the next one */
    interfaces = numberOfInterfaces();
    /* find netif with index */
    for(i = 0; i < interfaces; i++)
      {
      u32_t test_oid[LWIP_ARRAYSIZE(interfaces_Table_oid_ranges)];
      test_oid[0] = i;

      /* check generated OID: is it a candidate for the next one? */
      snmp_next_oid_check(&state, test_oid, LWIP_ARRAYSIZE(interfaces_Table_oid_ranges), (void*)i);
    }

    /* did we find a next one? */
    if (state.status == SNMP_NEXT_OID_STATUS_SUCCESS)
    {
        snmp_oid_assign(row_oid, state.next_oid, state.next_oid_len);
        /* store netif pointer for subsequent operations (get/test/set) */
        cell_instance->reference.u32 = LWIP_CONST_CAST(u32_t, state.reference);
        return SNMP_ERR_NOERROR;
    }

    /* not found */
    return SNMP_ERR_NOSUCHINSTANCE;
}

static s16_t
interfaces_Table_get_value(struct snmp_node_instance *instance, void *value)
{
    s32_t interface = instance->reference.s32;
    ifp[interface].ifIndex = interface + 1;
    u32_t *value_u32 = (u32_t *)value;
    u16_t value_len;

    switch (SNMP_TABLE_GET_COLUMN_FROM_OID(instance->instance_oid.id)) {
      case 1: /* ifIndex */
        *value_u32 = interface + 1;
        value_len = sizeof(*value_u32);
        break;
      case 2: /* ifDescr */
        value_len = strlen(getIfDescr(interface));
        MEMCPY(value, (u32_t*)getIfDescr(interface), value_len);
        break;
      case 3: /* ifType */
        getIfType();
        *value_u32 = getIfType();;
        value_len = sizeof(*value_u32);
        break;
      case 4: /* ifMtu */
        ifp[interface].ifMtu = getIfMtu();
        *value_u32 = getIfMtu();
        value_len = sizeof(*value_u32);
        break;
      case 5: /* ifSpeed */
        ifp[interface].ifSpeed = getIfSpeed(interface);
        *value_u32 = getIfSpeed(interface);
        value_len = sizeof(*value_u32);
        break;
      case 6: /* ifPhysAddress */
        value_len = sizeof((void*)(&(((ICSS_EMAC_Object *)emachandle->object)->macId[0])));
        memcpy(value, (void*)(&(((ICSS_EMAC_Object *)emachandle->object)->macId[0])), value_len);
          break;
        break;
      case 7: /* ifAdminStatus */
        getIfStatus(&(ifp[interface].ifAdminStatus), &(ifp[interface].ifOperStatus), &(ifp[interface].ifLastChange), interface);
        *value_u32 = ifp[interface].ifAdminStatus;
        value_len = sizeof(*value_u32);
        break;
      case 8: /* ifOperStatus */
        getIfStatus(&(ifp[interface].ifAdminStatus), &(ifp[interface].ifOperStatus), &(ifp[interface].ifLastChange), interface);
        *value_u32 = ifp[interface].ifOperStatus;
        value_len = sizeof(*value_u32);
        break;
      case 9: /* ifLastChange */
        getIfStatus(&(ifp[interface].ifAdminStatus), &(ifp[interface].ifOperStatus), &(ifp[interface].ifLastChange), interface);
        *value_u32 = ifp[interface].ifLastChange;
        value_len = sizeof(*value_u32);
        break;
      case 10: /* ifInOctets */
        ifp[interface].ifInOctets = getIfInOctets(interface);
        *value_u32 = getIfInOctets(interface);
        value_len = sizeof(*value_u32);
        break;
      case 11: /* ifInUcastPkts */
        ifp[interface].ifInUcastPkts = getIfInUcastPkts(interface);
        *value_u32 = getIfInUcastPkts(interface);
        value_len = sizeof(*value_u32);
        break;
      case 12: /* ifInNUcastPkts */
        ifp[interface].ifInNUcastPkts = getIfInNUcastPkts(interface);
        *value_u32 =getIfInNUcastPkts(interface);
        value_len = sizeof(*value_u32);
        break;
      case 13: /* ifInDiscards */
        ifp[interface].ifInDiscards = getIfInDiscards(interface);
        *value_u32 = ifp[interface].ifInDiscards;
        value_len = sizeof(*value_u32);
        break;
      case 14: /* ifInErrors */
        ifp[interface].ifInErrors = getIfInErrors(interface);
        *value_u32 = getIfInErrors(interface);
        value_len = sizeof(*value_u32);
        break;
      case 15: /* ifInUnkownProtos */
        ifp[interface].ifInUnknownProtos = getIfInUnkownProtos(interface);
        *value_u32 = getIfInUnkownProtos(interface);
        value_len = sizeof(*value_u32);
        break;
      case 16: /* ifOutOctets */
        ifp[interface].ifOutOctets = getIfOutOctets(interface);
        *value_u32 = getIfOutOctets(interface);
        value_len = sizeof(*value_u32);
        break;
      case 17: /* ifOutUcastPkts */
        ifp[interface].ifOutUcastPkts = getIfOutUcastPkts(interface);
        *value_u32 = getIfOutUcastPkts(interface);
        value_len = sizeof(*value_u32);
        break;
      case 18: /* ifOutNUcastPkts */
        ifp[interface].ifOutNUcastPkts = getIfOutNUcastPkts(interface);
        *value_u32 = getIfOutNUcastPkts(interface);
        value_len = sizeof(*value_u32);
        break;
      case 19: /* ifOutDiscarts */
        ifp[interface].ifOutDiscards = getIfOutDiscards(interface);
        *value_u32 = getIfOutDiscards(interface);
        value_len = sizeof(*value_u32);
        break;
      case 20: /* ifOutErrors */
        ifp[interface].ifOutErrors = getIfOutErrors(interface);
        *value_u32 = getIfOutErrors(interface);
        value_len = sizeof(*value_u32);
        break;
      case 21: /* ifOutQLen */
        *value_u32 = ifp[interface].ifOutQLen;
        value_len = sizeof(*value_u32);
        break;
      /** @note returning zeroDotZero (0.0) no media specific MIB support */
      case 22: /* ifSpecific */
        *value_u32 = 0;
        value_len = sizeof(*value_u32);
        break;
      default:
        return 0;
    }

    return value_len;
}

#if !SNMP_SAFE_REQUESTS
static snmp_err_t
interfaces_Table_set_test(struct snmp_node_instance *instance, u16_t len, void *value)
{
    s32_t *sint_ptr = (s32_t *)value;

    /* stack should never call this method for another column,
    because all other columns are set to readonly */
    LWIP_ASSERT("Invalid column", (SNMP_TABLE_GET_COLUMN_FROM_OID(instance->instance_oid.id) == 7));
    LWIP_UNUSED_ARG(len);

    if (*sint_ptr == 1 || *sint_ptr == 2)
    {
        return SNMP_ERR_NOERROR;
    }

    return SNMP_ERR_WRONGVALUE;
}

static snmp_err_t
interfaces_Table_set_value(struct snmp_node_instance *instance, u16_t len, void *value)
{
    struct netif *netif = (struct netif *)instance->reference.ptr;
    s32_t *sint_ptr = (s32_t *)value;

    /* stack should never call this method for another column,
    because all other columns are set to readonly */
    LWIP_ASSERT("Invalid column", (SNMP_TABLE_GET_COLUMN_FROM_OID(instance->instance_oid.id) == 7));
    LWIP_UNUSED_ARG(len);

    if (*sint_ptr == 1)
    {
        netif_set_up(netif);
    }
    else if (*sint_ptr == 2)
    {
        netif_set_down(netif);
    }

    return SNMP_ERR_NOERROR;
}

#endif /* SNMP_SAFE_REQUESTS */



#endif /* LWIP_SNMP && LRECFG_PRV_MIB*/
