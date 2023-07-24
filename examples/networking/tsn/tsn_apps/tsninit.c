#include <tsn_combase/combase.h>
#include <tsn_unibase/unibase_binding.h>
#include "yangs/tsn_data.h"
#include "yangs/yang_db_runtime.h"
#include "yangs/yang_modules.h"
#include <tsn_gptp/gptpman.h>
#include <tsn_gptp/gptpconf/gptpgcfg.h>
#include "tsn_uniconf/yangs/ieee1588-ptp_access.h"
#include <tsn_uniconf/ucman.h>
#include <tsn_uniconf/uc_dbal.h>
#include "enet_types.h"
#include "debug_log.h"
#include "tsninit.h"
#include "uc_client.h"

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
#define UCCLIENT_TASK_PRIORITY  (2)

#define GPTP_TASK_NAME		"gptp2d_task"
#define UNICONF_TASK_NAME	"uniconf_task"
#ifndef DISABLE_UC_CLIENT
#define UCLIENT_TASK_NAME	"ucclient_task"
#endif

#define TSN_TSK_STACK_SIZE (16U*1024)
#define TSN_TSK_STACK_ALIGN (TSN_TSK_STACK_SIZE)


static uint8_t g_gptp_stack_buf[TSN_TSK_STACK_SIZE] __attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));

typedef struct gptpdpd
{
    const char **netdevs;
} gptpdpd_t;

typedef struct gptpoptd
{
    char *devlist;
    const char **conf_files;
    const char *db_file;
    int netdnum;
    int domain_num;
    int instnum;
    int numconf;
} gptpoptd_t;

static uint8_t g_uniconf_stack_buf[TSN_TSK_STACK_SIZE] __attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));
#ifndef DISABLE_UC_CLIENT
static uint8_t g_uclient_stack_buf[TSN_TSK_STACK_SIZE] __attribute__ ((aligned(TSN_TSK_STACK_ALIGN)));
#endif

#define MAX_KEY_SIZE      (256)
static int set_gptp_run_time_config(int instance, int domain,
                                    const char *netdevs[], int numOfPorts,
                                    int rate)
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
            DPRINT("The uniconf must be run first !"LINE_FEED);
            break;
        }
        dbald = uc_dbal_open(NULL, "w", UC_CALLMODE_THREAD);
        if (!dbald)
        {
            DPRINT("Failed to open DB!"LINE_FEED);
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

        plus= ((instance | domain) != 0) ? "+": "";
        snprintf(buffer, sizeof(buffer), "/ieee1588-ptp/ptp/instance-domain-map%s",
                 plus);
        snprintf(value_str, sizeof(value_str), "0x%04x", instance<<8|domain);
        yang_db_runtime_put_oneline(ydrd, buffer,
                                    value_str, YANG_DB_ONHW_NOACTION);

        // portindex starts from 1
        for (i = 0; i < numOfPorts; i++)
        {
            snprintf(buffer, sizeof(buffer),
                     "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                     "/ports/port|port-index:%d|/underlying-interface", instance, domain, i+1);
            snprintf(value_str, sizeof(value_str), "%s", netdevs[i]);
            yang_db_runtime_put_oneline(ydrd, buffer,
                                        value_str, YANG_DB_ONHW_NOACTION);

            snprintf(buffer, sizeof(buffer),
                     "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                     "/ports/port|port-index:%d|/port-ds/port-enable", instance, domain, i+1);
            strcpy(value_str, "true");
            yang_db_runtime_put_oneline(ydrd, buffer,
                                        value_str, YANG_DB_ONHW_NOACTION);

            snprintf(buffer, sizeof(buffer),
                     "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                     "/ports/port|port-index:%d|/port-ds/log-announce-interval",
                     instance, domain, i+1);
            strcpy(value_str, "0");
            yang_db_runtime_put_oneline(ydrd, buffer,
                                        value_str, YANG_DB_ONHW_NOACTION);

            snprintf(buffer, sizeof(buffer),
                     "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                     "/ports/port|port-index:%d|/port-ds/gptp-cap-receipt-timeout",
                     instance, domain, i+1);
            strcpy(value_str, "3");
            yang_db_runtime_put_oneline(ydrd, buffer,
                                        value_str, YANG_DB_ONHW_NOACTION);

            snprintf(buffer, sizeof(buffer),
                     "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                     "/ports/port|port-index:%d|/port-ds/announce-receipt-timeout",
                     instance, domain, i+1);
            strcpy(value_str, "3");
            yang_db_runtime_put_oneline(ydrd, buffer,
                                        value_str, YANG_DB_ONHW_NOACTION);

            snprintf(buffer, sizeof(buffer),
                     "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                     "/ports/port|port-index:%d|/port-ds/log-sync-interval",
                     instance, domain, i+1);
            strcpy(value_str, "-3");
            yang_db_runtime_put_oneline(ydrd, buffer,
                                        value_str, YANG_DB_ONHW_NOACTION);

            snprintf(buffer, sizeof(buffer),
                     "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                     "/ports/port|port-index:%d|/port-ds/minor-version-number",
                     instance, domain, i+1);
            strcpy(value_str, "1");
            yang_db_runtime_put_oneline(ydrd, buffer,
                                        value_str, YANG_DB_ONHW_NOACTION);

            snprintf(buffer, sizeof(buffer),
                     "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                     "/ports/port|port-index:%d|/port-ds/initial-log-announce-interval",
                     instance, domain, i+1);
            strcpy(value_str, "0");
            yang_db_runtime_put_oneline(ydrd, buffer,
                                        value_str, YANG_DB_ONHW_NOACTION);

            snprintf(buffer, sizeof(buffer),
                     "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                     "/ports/port|port-index:%d|/port-ds/sync-receipt-timeout",
                     instance, domain, i+1);
            strcpy(value_str, "3");
            yang_db_runtime_put_oneline(ydrd, buffer,
                                        value_str, YANG_DB_ONHW_NOACTION);

            snprintf(buffer, sizeof(buffer),
                     "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                     "/ports/port|port-index:%d|/port-ds/initial-log-sync-interval",
                     instance, domain, i+1);
            strcpy(value_str, "-3");
            yang_db_runtime_put_oneline(ydrd, buffer,
                                        value_str, YANG_DB_ONHW_NOACTION);

            snprintf(buffer, sizeof(buffer),
                     "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                     "/ports/port|port-index:%d|/port-ds/current-log-sync-interval",
                     instance, domain, i+1);
            strcpy(value_str, "-3");
            yang_db_runtime_put_oneline(ydrd, buffer,
                                        value_str, YANG_DB_ONHW_NOACTION);

            snprintf(buffer, sizeof(buffer),
                     "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                     "/ports/port|port-index:%d|/port-ds/current-log-gptp-cap-interval",
                     instance, domain, i+1);
            strcpy(value_str, "3");
            yang_db_runtime_put_oneline(ydrd, buffer,
                                        value_str, YANG_DB_ONHW_NOACTION);

            snprintf(buffer, sizeof(buffer),
                     "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                     "/ports/port|port-index:%d|/port-ds/initial-log-pdelay-req-interval",
                     instance, domain, i+1);
            strcpy(value_str, "0");
            yang_db_runtime_put_oneline(ydrd, buffer,
                                        value_str, YANG_DB_ONHW_NOACTION);

            snprintf(buffer, sizeof(buffer),
                     "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                     "/ports/port|port-index:%d|/port-ds/current-log-pdelay-req-interval",
                     instance, domain, i+1);
            strcpy(value_str, "0");
            yang_db_runtime_put_oneline(ydrd, buffer,
                                        value_str, YANG_DB_ONHW_NOACTION);
            snprintf(buffer, sizeof(buffer),
                     "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                     "/ports/port|port-index:%d|/port-ds/allowed-lost-responses",
                     instance, domain, i+1);
            strcpy(value_str, "9");
            yang_db_runtime_put_oneline(ydrd, buffer,
                                        value_str, YANG_DB_ONHW_NOACTION);

            snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/ports/port|port-index:%d|/port-ds/allowed-faults",
                     instance, domain, i+1);
            strcpy(value_str, "9");
            yang_db_runtime_put_oneline(ydrd, buffer,
                                        value_str, YANG_DB_ONHW_NOACTION);

            snprintf(buffer, sizeof(buffer),
                     "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                     "/ports/port|port-index:%d|/port-ds/mean-link-delay-thresh",
                     instance, domain, i+1);
            strcpy(value_str, "0x27100000"); //10000<<16, 10000nsec
            yang_db_runtime_put_oneline(ydrd, buffer,
                                        value_str, YANG_DB_ONHW_NOACTION);
        }

        snprintf(buffer, sizeof(buffer),
              "/ieee1588-ptp/ptp/instances/instance|instance-index:%d|"
                     "/ports/port|port-index:%d|/port-ds/log-sync-interval",
                     instance, 0);
        strcpy(value_str, "-3");
        yang_db_runtime_put_oneline(ydrd, buffer,
                                    value_str, YANG_DB_ONHW_NOACTION);

        snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/ports/port|port-index:%d|/port-ds/current-log-pdelay-req-interval",
                 instance, domain, 0);
        strcpy(value_str, "0");
        yang_db_runtime_put_oneline(ydrd, buffer,
                                    value_str, YANG_DB_ONHW_NOACTION);

        snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/ports/port|port-index:%d|/port-ds/initial-log-pdelay-req-interval",
                 instance, domain, 0);
        strcpy(value_str, "0");
        yang_db_runtime_put_oneline(ydrd, buffer,
                                    value_str, YANG_DB_ONHW_NOACTION);

        snprintf(buffer, sizeof(buffer),
                     "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                     "/ports/port|port-index:%d|/port-ds/log-announce-interval",
                     instance, domain, 0);
        strcpy(value_str, "0");
        yang_db_runtime_put_oneline(ydrd, buffer,
                                    value_str, YANG_DB_ONHW_NOACTION);

        snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/ports/port|port-index:%d|/port-ds/current-log-sync-interval",
                 instance, domain, 0);
        strcpy(value_str, "-3");
        yang_db_runtime_put_oneline(ydrd, buffer,
                                    value_str, YANG_DB_ONHW_NOACTION);

        snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/ports/port|port-index:%d|/port-ds/initial-log-sync-interval",
                 instance, domain, 0);
        strcpy(value_str, "-3");
        yang_db_runtime_put_oneline(ydrd, buffer,
                                    value_str, YANG_DB_ONHW_NOACTION);
        snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/ports/port|port-index:%d|/port-ds/sync-receipt-timeout",
                 instance, domain, 0);
        strcpy(value_str, "3");
        yang_db_runtime_put_oneline(ydrd, buffer,
                                    value_str, YANG_DB_ONHW_NOACTION);
        snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/ports/port|port-index:%d|/port-ds/port-enable", instance, domain, 0);
        strcpy(value_str, "true");

        yang_db_runtime_put_oneline(ydrd, buffer,
                                    value_str, YANG_DB_ONHW_NOACTION);
        snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/ports/port|port-index:%d|/port-ds/current-log-gptp-cap-interval",
                 instance, domain, 0);
        strcpy(value_str, "3");
        yang_db_runtime_put_oneline(ydrd, buffer,
                                    value_str, YANG_DB_ONHW_NOACTION);

        snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/ports/port|port-index:%d|/port-ds/gptp-cap-receipt-timeout",
                 instance, domain, 0);
        strcpy(value_str, "3");
        yang_db_runtime_put_oneline(ydrd, buffer,
                                    value_str, YANG_DB_ONHW_NOACTION);

        snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/ports/port|port-index:%d|/port-ds/initial-log-announce-interval",
                 instance, domain, 0);
        strcpy(value_str, "0");
        yang_db_runtime_put_oneline(ydrd, buffer,
                                    value_str, YANG_DB_ONHW_NOACTION);

        snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/ports/port|port-index:%d|/port-ds/announce-receipt-timeout",
                 instance, domain, 0);
        strcpy(value_str, "3");
        yang_db_runtime_put_oneline(ydrd, buffer,
                                    value_str, YANG_DB_ONHW_NOACTION);
        snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/ports/port|port-index:%d|/port-ds/minor-version-number",
                 instance, domain, 0);
        strcpy(value_str, "1");
        yang_db_runtime_put_oneline(ydrd, buffer,
                                    value_str, YANG_DB_ONHW_NOACTION);

        // Use port=0 data for all ports
        snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/ports/port|port-index:0|/port-ds/allowed-lost-responses",
                 instance, domain);
        strcpy(value_str, "9");
        yang_db_runtime_put_oneline(ydrd, buffer,
                                    value_str, YANG_DB_ONHW_NOACTION);

        snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/ports/port|port-index:0|/port-ds/allowed-faults",
                 instance, domain);
        strcpy(value_str, "9");
        yang_db_runtime_put_oneline(ydrd, buffer,
                                    value_str, YANG_DB_ONHW_NOACTION);

        snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/ports/port|port-index:0|/port-ds/mean-link-delay-thresh",
                 instance, domain);
        strcpy(value_str, "0x27100000"); //10000<<16, 10000nsec
        yang_db_runtime_put_oneline(ydrd, buffer,
                                    value_str, YANG_DB_ONHW_NOACTION);

        snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/default-ds/priority1",
                 instance, domain);
        snprintf(value_str, sizeof(value_str), "%d", 240+instance);
        yang_db_runtime_put_oneline(ydrd, buffer,
                                    value_str, YANG_DB_ONHW_NOACTION);

        snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/default-ds/priority2",
                 instance, domain);
        strcpy(value_str, "245");
        yang_db_runtime_put_oneline(ydrd, buffer,
                                    value_str, YANG_DB_ONHW_NOACTION);

        snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/default-ds/external-port-config-enable",
                 instance, domain);
        strcpy(value_str, "false");
        yang_db_runtime_put_oneline(ydrd, buffer,
                                    value_str, YANG_DB_ONHW_NOACTION);

        snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/default-ds/time-source",
                 instance, domain);
        strcpy(value_str, "internal-oscillator");
        yang_db_runtime_put_oneline(ydrd, buffer,
                                    value_str, YANG_DB_ONHW_NOACTION);

        snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/default-ds/ptp-timescale",
                 instance, domain);
        strcpy(value_str, "true");
        yang_db_runtime_put_oneline(ydrd, buffer,
                                    value_str, YANG_DB_ONHW_NOACTION);

        snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/default-ds/clock-quality/clock-class",
                 instance, domain);
        strcpy(value_str, "cc-default");
        yang_db_runtime_put_oneline(ydrd, buffer,
                                    value_str, YANG_DB_ONHW_NOACTION);

        snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/default-ds/clock-quality/clock-accuracy",
                 instance, domain);
        strcpy(value_str, "ca-time-accurate-to-250-ns");
        yang_db_runtime_put_oneline(ydrd, buffer,
                                    value_str, YANG_DB_ONHW_NOACTION);

        snprintf(buffer, sizeof(buffer),
                 "/ieee1588-ptp/ptp/instances/instance|instance-index:%d,%d|"
                 "/default-ds/clock-quality/offset-scaled-log-variance",
                 instance, domain);
        strcpy(value_str, "0x436a");
        yang_db_runtime_put_oneline(ydrd, buffer,
                                    value_str, YANG_DB_ONHW_NOACTION);
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

static void *gptp_task(void *arg)
{
    struct module_ctx *mdctx = (struct module_ctx *)arg;
    struct tsn_app_ctx * ctx = mdctx->app_ctx;
    int res = 0;
    int i;

    uint8_t use_hwphase = 1;
    uint8_t single_clock = 1;
    gptpdpd_t gpdpd;
    gptpoptd_t gpoptd;
    bool stop_flag = false;

    memset(&gpdpd, 0, sizeof(gpdpd));
    memset(&gpoptd, 0, sizeof(gpoptd));

    /* Waiting for the uniconf to be ready */
    CB_SEM_WAIT(&ctx->uc_ready_sem);
    gpoptd.instnum=0;
    gpoptd.domain_num=0;
    gpoptd.db_file = ctx->db_name;
    gpoptd.conf_files=NULL;

    gpoptd.netdnum=ctx->netdev_size;
    gpdpd.netdevs=UB_SD_GETMEM(GPTP_SMALL_ALLOC,
                               (gpoptd.netdnum+1) * sizeof(char *));
    for (i=0; i<gpoptd.netdnum; i++)
    {
        gpdpd.netdevs[i] = ctx->netdev[i];
    }
    gpdpd.netdevs[i] = NULL;

    if (gpoptd.numconf == 0)
    {
        /* There is no config file is specified, set config file for gptp*/
        res = set_gptp_run_time_config(gpoptd.instnum, gpoptd.domain_num, gpdpd.netdevs,
                                       gpoptd.netdnum, 0);
        if (res)
        {
            DPRINT("Failed to set gptp run time config"LINE_FEED);
        }
    }

    if (!res)
    {
        DPRINT("%s: started."LINE_FEED, __func__);
        DPRINT("%s: dbname=%s"LINE_FEED, __func__, ctx->db_name);
        if (gptpgcfg_init(gpoptd.db_file, gpoptd.conf_files, gpoptd.instnum, true) < 0)
        {
            DPRINT("%s: gptpgcfg_init() error"LINE_FEED, __func__);
            return NULL;
        }

        if(gptpgcfg_set_item(gpoptd.instnum, XL4_EXTMOD_XL4GPTP_SINGLE_CLOCK_MODE, YDBI_CONFIG, &single_clock, sizeof(single_clock)) < 0)
        {
            DPRINT("%s: gptpgcfg_set_item() XL4_EXTMOD_XL4GPTP_SINGLE_CLOCK_MODE error"LINE_FEED, __func__);
            return NULL;
        }
        if(gptpgcfg_set_item(gpoptd.instnum, XL4_EXTMOD_XL4GPTP_USE_HW_PHASE_ADJUSTMENT, YDBI_CONFIG, &use_hwphase, sizeof(use_hwphase)) < 0)
        {
            DPRINT("%s: gptpgcfg_set_item() XL4_EXTMOD_XL4GPTP_USE_HW_PHASE_ADJUSTMENT error"LINE_FEED, __func__);
            return NULL;
        }

        /* This function has a true loop inside */
        if (gptpman_run(gpoptd.instnum, gpdpd.netdevs,
                        gpoptd.netdnum, gpoptd.domain_num, NULL, &stop_flag) < 0)
        {
            DPRINT("%s: gptpman_run() error"LINE_FEED, __func__);
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

#ifndef DISABLE_UC_CLIENT
static void* ucclient_task(void *arg)
{
    struct module_ctx *mdctx = (struct module_ctx *)arg;
    struct tsn_app_ctx * ctx = mdctx->app_ctx;
    char *netdev[MAX_NUMBER_ENET_DEVS+1] = {0};
    int i;

    for (i = 0; i < ctx->netdev_size; i++)
    {
        netdev[i] = &ctx->netdev[i][0];
    }
    netdev[i] = NULL;
    return uc_client_main(netdev);
}
#endif

static struct module_ctx g_ctx_table[] =
{
    {.task_priority = UNICONF_TASK_PRIORITY, .task_name = UNICONF_TASK_NAME,
     .stack_buffer = g_uniconf_stack_buf, .stack_size = sizeof(g_uniconf_stack_buf),
     .module_runner = uniconf_task, .app_ctx = &g_app_ctx},
#ifndef DISABLE_UC_CLIENT
    {.task_priority = UCCLIENT_TASK_PRIORITY, .task_name = UCLIENT_TASK_NAME,
     .stack_buffer = g_uclient_stack_buf, .stack_size = sizeof(g_uclient_stack_buf),
     .module_runner = ucclient_task, .app_ctx = &g_app_ctx},
#endif
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

    if (CB_SEM_INIT(&g_app_ctx.uc_ready_sem, 0, 1) < 0)
    {
        DPRINT("Failed to initialize uc_ready_sem semaphore!"LINE_FEED);
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
            DPRINT("Failed to create task!"LINE_FEED);
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
