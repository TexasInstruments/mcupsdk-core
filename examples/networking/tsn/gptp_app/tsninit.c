/*
 *  Copyright (c) Texas Instruments Incorporated 2023
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
/*                              Include Files                                 */
/* ========================================================================== */

#include <tsn_combase/combase.h>
#include <tsn_unibase/unibase_binding.h>
#include "yangs/tsn_data.h"
#include "yangs/yang_db_runtime.h"
#include "yangs/yang_modules.h"
#include <tsn_gptp/gptpman.h>
#include <tsn_gptp/tilld/lld_gptp_private.h>
#include <tsn_gptp/gptpconf/gptpgcfg.h>
#include <tsn_gptp/gptpconf/xl4-extmod-xl4gptp.h>
#include "tsn_uniconf/yangs/ieee1588-ptp_access.h"
#include <tsn_uniconf/ucman.h>
#include <tsn_uniconf/uc_dbal.h>
#include "enet_types.h"
#include "debug_log.h"
#include "tsninit.h"

typedef void* (*on_module_start)(void *arg);

struct tsn_app_ctx
{
    char *db_name; // Specify same DB for all modules.
    bool init_flag;
    ucman_data_t uc_ctx;
    char netdev[MAX_NUMBER_ENET_DEVS][CB_MAX_NETDEVNAME];
    uint8_t netdev_size;
    CB_SEM_T uc_ready_sem;
};
static struct tsn_app_ctx g_app_ctx;

struct module_ctx
{
    bool stop_flag;
    int task_priority;
    CB_THREAD_T task_handle;
    const char *task_name;
    uint8_t *stack_buffer;
    uint32_t stack_size;
    on_module_start module_runner;
    struct tsn_app_ctx *app_ctx;
};

#define UNICONF_TASK_PRIORITY   (2)
#define GPTP_TASK_PRIORITY      (2)

#define GPTP_TASK_NAME        "gptp2d_task"
#define UNICONF_TASK_NAME    "uniconf_task"

#define TSN_TSK_STACK_SIZE (16U*1024)
#define TSN_TSK_STACK_ALIGN (TSN_TSK_STACK_SIZE)


static uint8_t g_gptp_stack_buf[TSN_TSK_STACK_SIZE] __attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

typedef struct gptpoptd
{
    char *devlist;
    const char **conf_files;
    const char *db_file;
    int domain_num;
    int instnum;
    int numconf;
} gptpoptd_t;

static uint8_t g_uniconf_stack_buf[TSN_TSK_STACK_SIZE] __attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

#define MAX_KEY_SIZE      (256)

typedef struct
{
    char *name;
    char *val;
} dbname_val_t;

static dbname_val_t g_gptp_port_ds[] =
{
    {"port-enable", "true"},
    {"log-announce-interval", "0"},
    {"gptp-cap-receipt-timeout", "3"},
    {"announce-receipt-timeout", "3"},
    {"log-sync-interval", "-3"},
    {"minor-version-number", "1"},
    {"initial-log-announce-interval", "0"},
    {"sync-receipt-timeout", "3"},
    {"initial-log-sync-interval", "-3"},
    {"current-log-sync-interval", "-3"},
    {"current-log-gptp-cap-interval", "3"},
    {"initial-log-pdelay-req-interval", "0"},
    {"current-log-pdelay-req-interval", "0"},
    {"allowed-lost-responses", "9"},
    {"allowed-faults", "9"},
    {"mean-link-delay-thresh", "0x27100000"}
};

static dbname_val_t g_gptp_default_ds[] =
{
    {"priority1", "248"},
    {"priority2", "248"},
    {"external-port-config-enable", "false"},
    {"time-source", "internal-oscillator"},
    {"ptp-timescale", "true"},
    {"clock-quality/clock-class", "cc-default"},
    {"clock-quality/clock-accuracy", "ca-time-accurate-to-250-ns"},
    {"clock-quality/offset-scaled-log-variance", "0x436a"}
};

static void config_gptp_port_ds(yang_db_runtime_dataq_t *ydrd, int instance,
                             int domain, int port_index)
{
    int i;
    char buffer[MAX_KEY_SIZE];

    for (i = 0; i < sizeof(g_gptp_port_ds)/sizeof(g_gptp_port_ds[0]); i++)
    {
        snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/ports/port|port-index:%d|/port-ds/%s",
                 instance, domain, port_index, g_gptp_port_ds[i].name);

        yang_db_runtime_put_oneline(ydrd, buffer, g_gptp_port_ds[i].val,
                                    YANG_DB_ONHW_NOACTION);
    }
}

static void config_gptp_default_ds(yang_db_runtime_dataq_t *ydrd, int instance,
                                   int domain)
{
    int i;
    char buffer[MAX_KEY_SIZE];

    for (i = 0; i < sizeof(g_gptp_default_ds)/sizeof(g_gptp_default_ds[0]); i++)
    {
        snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/default-ds/%s",
                 instance, domain, g_gptp_default_ds[i].name);
        yang_db_runtime_put_oneline(ydrd, buffer, g_gptp_default_ds[i].val,
                                    YANG_DB_ONHW_NOACTION);
    }
}

static int gptp_yang_config(int instance, int domain,
            const char *netdevs[], int numOfPorts)
{
    uc_dbald *dbald = NULL;
    xl4_data_data_t *xdd = NULL;
    yang_db_runtime_dataq_t *ydrd = NULL;
    char buffer[MAX_KEY_SIZE];
    char value_str[32];
    char *plus;
    int i, res = 0;
    int timeout_ms = 500;

    do {
        res = uniconf_ready(NULL, UC_CALLMODE_THREAD, timeout_ms);
        if (res)
        {
            DPRINT("The uniconf must be run first !");
            break;
        }
        dbald = uc_dbal_open(NULL, "w", UC_CALLMODE_THREAD);
        if (!dbald)
        {
            DPRINT("Failed to open DB!");
            res = -1;
            break;
        }
        xdd = xl4_data_init(dbald);
        if (!xdd)
        {
            DPRINT("Failed to init xl4 data");
            res = -1;
            break;
        }
        ydrd = yang_db_runtime_init(xdd, dbald, NULL);
        if (!ydrd)
        {
            DPRINT("Failed to init yang db runtime");
            res = -1;
            break;
        }

        plus = ((instance | domain) != 0) ? "+": "";
        snprintf(buffer, sizeof(buffer), "/ieee1588-ptp/ptp/instance-domain-map%s",
                 plus);
        snprintf(value_str, sizeof(value_str), "0x%04x", instance<<8|domain);
        yang_db_runtime_put_oneline(ydrd, buffer,
                                    value_str, YANG_DB_ONHW_NOACTION);

        /* set for default-ds */
        config_gptp_default_ds(ydrd, instance, domain);

        // portindex starts from 1
        for (i = 0; i < numOfPorts; i++)
        {
            snprintf(buffer, sizeof(buffer),
                     "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                     "/ports/port|port-index:%d|/underlying-interface",
                     instance, domain, i+1);
            yang_db_runtime_put_oneline(ydrd, buffer, (char*)netdevs[i],
                                        YANG_DB_ONHW_NOACTION);

            /* set for port-ds */
            config_gptp_port_ds(ydrd, instance, domain, i+1);
        }
        /* disable performance by default */
        snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/performance-monitoring-ds/enable",
                 instance, domain);
        yang_db_runtime_put_oneline(ydrd, buffer, "false", YANG_DB_ONHW_NOACTION);
    } while (0);
    if (ydrd)
    {
        yang_db_runtime_close(ydrd);
    }
    if (xdd)
    {
        xl4_data_close(xdd);
    }
    if (dbald)
    {
        uc_dbal_close(dbald, UC_CALLMODE_THREAD);
    }

    return res;
}

typedef struct
{
    char *name;
    int item;
    int val;
} dbint_val_t;

static dbint_val_t g_gptp_nonyang_ds[] =
{
    {"SINGLE_CLOCK_MODE", XL4_EXTMOD_XL4GPTP_SINGLE_CLOCK_MODE, 1},
    {"USE_HW_PHASE_ADJUSTMENT", XL4_EXTMOD_XL4GPTP_USE_HW_PHASE_ADJUSTMENT, 1},
    {"CLOCK_COMPUTE_INTERVAL_MSEC", XL4_EXTMOD_XL4GPTP_CLOCK_COMPUTE_INTERVAL_MSEC, 100},
    {"FREQ_OFFSET_IIR_ALPHA_START_VALUE", XL4_EXTMOD_XL4GPTP_FREQ_OFFSET_IIR_ALPHA_START_VALUE, 1},
    {"FREQ_OFFSET_IIR_ALPHA_STABLE_VALUE", XL4_EXTMOD_XL4GPTP_FREQ_OFFSET_IIR_ALPHA_STABLE_VALUE, 4},
    {"PHASE_OFFSET_IIR_ALPHA_START_VALUE", XL4_EXTMOD_XL4GPTP_PHASE_OFFSET_IIR_ALPHA_START_VALUE, 1},
    {"PHASE_OFFSET_IIR_ALPHA_STABLE_VALUE", XL4_EXTMOD_XL4GPTP_PHASE_OFFSET_IIR_ALPHA_STABLE_VALUE, 4},
};

static int gptp_nonyang_config(int instance)
{
    int i;
    int res;

    for (i = 0; i < sizeof(g_gptp_nonyang_ds)/sizeof(g_gptp_nonyang_ds[0]); i++)
    {
        res = gptpgcfg_set_item(instance, g_gptp_nonyang_ds[i].item,  YDBI_CONFIG,
                    &g_gptp_nonyang_ds[i].val, sizeof(g_gptp_nonyang_ds[i].val));
        if (res == 0)
        {
            DPRINT("%s:XL4_EXTMOD_XL4GPTP_%s=%d", __func__,
                   g_gptp_nonyang_ds[i].name, g_gptp_nonyang_ds[i].val);
        }
        else
        {
            DPRINT("%s: failed to set nonyange param: %s",
                   __func__, g_gptp_nonyang_ds[i].name);
            break;
        }
    }
    return res;
}

static void *gptp_task(void *arg)
{
    int i;
    int res = 0;
    struct module_ctx *mdctx = (struct module_ctx *)arg;
    struct tsn_app_ctx * ctx = mdctx->app_ctx;
    gptpoptd_t gpoptd;
    bool stop_flag = false;
    const char *netdevs[CB_MAX_NETDEVNAME];

    memset(&gpoptd, 0, sizeof(gpoptd));
    /* Waiting for the uniconf to be ready */
    CB_SEM_WAIT(&ctx->uc_ready_sem);
    gpoptd.instnum=0;
    gpoptd.domain_num=0;
    gpoptd.db_file = ctx->db_name;
    gpoptd.conf_files=NULL;

    for (i = 0; i < ctx->netdev_size; i++)
    {
        netdevs[i] = ctx->netdev[i];
    }
    if (gpoptd.numconf == 0)
    {
        /* There is no config file is specified, set config file for gptp*/
        res = gptp_yang_config(gpoptd.instnum, gpoptd.domain_num,
                netdevs, ctx->netdev_size);
        if (res)
        {
            DPRINT("Failed to set gptp run time config");
        }
    }

    if (!res)
    {
        DPRINT("%s: started, dbname: %s", __func__, ctx->db_name);
        res = gptpgcfg_init(gpoptd.db_file, gpoptd.conf_files, gpoptd.instnum, true);
        if (res)
        {
            DPRINT("%s: gptpgcfg_init() error", __func__);
        }
        if (!res)
        {
            res = gptp_nonyang_config(gpoptd.instnum);
            if (res)
            {
                DPRINT("%s: gptp_nonyang_config() error", __func__);
            }
        }

        if (!res)
        {
            /* This function has a true loop inside */
            res = gptpman_run(gpoptd.instnum, netdevs, ctx->netdev_size,
                              gpoptd.domain_num, NULL, &stop_flag);
            if (res)
            {
                DPRINT("%s: gptpman_run() error", __func__);
            }
        }
    }

    return NULL;
}

static void *uniconf_task(void *arg)
{
    struct module_ctx *mdctx = (struct module_ctx *)arg;
    struct tsn_app_ctx * ctx = mdctx->app_ctx;

    ctx->uc_ctx.ucmode = UC_CALLMODE_THREAD|UC_CALLMODE_UNICONF;
    ctx->uc_ctx.stoprun = &mdctx->stop_flag;
    ctx->uc_ctx.hwmod="";
    ctx->uc_ctx.ucmanstart = g_app_ctx.uc_ready_sem;
    ctx->uc_ctx.dbname = g_app_ctx.db_name;

    return uniconf_main(&ctx->uc_ctx);
}

static struct module_ctx g_ctx_table[] =
{
    {.task_priority = UNICONF_TASK_PRIORITY, .task_name = UNICONF_TASK_NAME,
     .stack_buffer = g_uniconf_stack_buf, .stack_size = sizeof(g_uniconf_stack_buf),
     .module_runner = uniconf_task, .app_ctx = &g_app_ctx},
    {.task_priority = GPTP_TASK_PRIORITY, .task_name = GPTP_TASK_NAME,
     .stack_buffer = g_gptp_stack_buf, .stack_size = sizeof(g_gptp_stack_buf),
     .module_runner = gptp_task, .app_ctx = &g_app_ctx},
};

int tsn_app_init(tsn_app_cfg_t *cfg)
{
    unibase_init_para_t init_para;

    ubb_default_initpara(&init_para);
    init_para.ub_log_initstr="4,ubase:45,cbase:45,gptp:55,uconf:45";
    init_para.cbset.gettime64=cb_lld_gettime64;

    if (cfg->console_out)
    {
        init_para.cbset.console_out=LOG_OUTPUT;
        debug_log_init(cfg->console_out);
    }
    int i = 0;
    while (cfg->netdevs[i] != NULL && i < MAX_NUMBER_ENET_DEVS)
    {
        if (strlen(cfg->netdevs[i]) < CB_MAX_NETDEVNAME)
        {
            strcpy(&g_app_ctx.netdev[g_app_ctx.netdev_size][0],
                   cfg->netdevs[i]);
            g_app_ctx.netdev_size++;
        }
        i++;
    }
    unibase_init(&init_para);
    ubb_memory_out_init(NULL, 0);

    return 0;
}

void tsn_app_deinit(void)
{
    debug_log_deinit();
    unibase_close();
}

int tsn_app_start(void)
{
    cb_tsn_thread_attr_t attr;

    if (CB_SEM_INIT(&g_app_ctx.uc_ready_sem, 0, 0) < 0)
    {
        DPRINT("Failed to initialize uc_ready_sem semaphore!");
        return ENET_EFAIL;
    }

    g_app_ctx.db_name = NULL; // Specify full path of a DB file from file system.
    for (int i = 0; i < sizeof(g_ctx_table)/sizeof(g_ctx_table[0]); i++)
    {
        cb_tsn_thread_attr_init(&attr, g_ctx_table[i].task_priority,
                                g_ctx_table[i].stack_size,
                                g_ctx_table[i].task_name);
        cb_tsn_thread_attr_set_stackaddr(&attr, g_ctx_table[i].stack_buffer);
        if (CB_THREAD_CREATE(&g_ctx_table[i].task_handle, &attr,
                             g_ctx_table[i].module_runner, &g_ctx_table[i]) < 0)
        {
            DPRINT("Failed to create task!");
            return ENET_EFAIL;
        }
    }
    return 0;
}

void tsn_app_stop(void)
{
    CB_SEM_DESTROY(&g_app_ctx.uc_ready_sem);
    for (int i = 0; i < sizeof(g_ctx_table)/sizeof(g_ctx_table[0]); i++)
    {
        CB_THREAD_JOIN(g_ctx_table[i].task_handle, NULL);
        g_ctx_table[i].task_handle = NULL;
    }
}
