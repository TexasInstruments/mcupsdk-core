/*
 *  Copyright (c) Texas Instruments Incorporated 2023
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *	Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 *
 *	Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in the
 *	documentation and/or other materials provided with the
 *	distribution.
 *
 *	Neither the name of Texas Instruments Incorporated nor the names of
 *	its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
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
#include <string.h>
#include <errno.h>
#include <tsn_combase/combase.h>
#include <tsn_unibase/unibase_binding.h>
#include "yangs/tsn_data.h"
#include "yangs/yang_db_runtime.h"
#include "yangs/yang_modules.h"
#include "ucman.h"
#include "debug_log.h"

#define Z_FREE(x) do { free(x); (x) = NULL; } while (0)

typedef struct ucclient_opt
{
    const char *dbname;
    int waitdb;
    char **netdevs; // A list of ifname (string) terminated by NULL.
} ucclient_opt_t;

struct tas_gatecmd_entry
{
    uint32_t time_interval;
    uint8_t gate_state_mask;
} tas_gatecmd_entry;

#define TAS_MAX_CMD_LISTS (8)
#define CMD_LIST_LENGTH   (8)
#define MAX_KEY_SIZE      (512)
#define MAX_VALUE_SIZE    (16)
#define WAIT_DB_TIME_MS   (1000)

struct tas_config_param
{
    const char *ifname;
    uint32_t base_time_seconds;
    uint32_t base_time_nanoseconds;
    struct tas_gatecmd_entry cmd_entry[TAS_MAX_CMD_LISTS];
    uint8_t cmd_entry_len;
    uint8_t num_of_tc;
    bool gate_enabled;
    uint32_t cycle_time_numerator;
    uint32_t cycle_time_denominator;
};

#define OPERATION_NAME_FMT ("/ietf-interfaces/interfaces/interface|name:%s|/bridge-port/gate-parameter-table/admin-control-list/gate-control-entry|index:%d|/operation-name")
#define TIME_INTERVAL_KEY_FMT ("/ietf-interfaces/interfaces/interface|name:%s|/bridge-port/gate-parameter-table/admin-control-list/gate-control-entry|index:%d|/time-interval-value")
#define GATE_MASK_KEY_FMT ("/ietf-interfaces/interfaces/interface|name:%s|/bridge-port/gate-parameter-table/admin-control-list/gate-control-entry|index:%d|/gate-states-value")
#define NUM_OF_TRAFFIC_CLASS_FMT ("/ietf-interfaces/interfaces/interface|name:%s|/bridge-port/traffic-class/traffic-class-table/number-of-traffic-classes")

#define CYCLE_TIME_NUM_FMT ("/ietf-interfaces/interfaces/interface|name:%s|/bridge-port/gate-parameter-table/admin-cycle-time/numerator")
#define CYCLE_TIME_DENOMINATOR_FMT ("/ietf-interfaces/interfaces/interface|name:%s|/bridge-port/gate-parameter-table/admin-cycle-time/denominator")

#define BASE_TIME_SECONDS_FMT ("/ietf-interfaces/interfaces/interface|name:%s|/bridge-port/gate-parameter-table/admin-base-time/seconds")
#define BASE_TIME_NANOSECONDS_FMT ("/ietf-interfaces/interfaces/interface|name:%s|/bridge-port/gate-parameter-table/admin-base-time/nanoseconds")

#define GATE_ENABLE_FMT ("/ietf-interfaces/interfaces/interface|name:%s|/bridge-port/gate-parameter-table/gate-enabled")

#define OPER_CYCLE_TIME_NUM_FMT ("/ietf-interfaces/interfaces/interface|name:%s|/bridge-port/gate-parameter-table/oper-cycle-time/numerator")
#define OPER_CYCLE_TIME_DENOMINATOR_FMT ("/ietf-interfaces/interfaces/interface|name:%s|/bridge-port/gate-parameter-table/oper-cycle-time/denominator")

#define OPER_BASE_TIME_SECONDS_FMT ("/ietf-interfaces/interfaces/interface|name:%s|/bridge-port/gate-parameter-table/oper-base-time/seconds")
#define OPER_BASE_TIME_NANOSECONDS_FMT ("/ietf-interfaces/interfaces/interface|name:%s|/bridge-port/gate-parameter-table/oper-base-time/nanoseconds")

static void set_tas_config_parameter(yang_db_runtime_dataq_t *ydrd,
                                     struct tas_config_param *cfg)
{
    char buffer[MAX_KEY_SIZE];
    char value_str[MAX_VALUE_SIZE];
    int i, n, res = 0;

    snprintf(buffer, sizeof(buffer), NUM_OF_TRAFFIC_CLASS_FMT, cfg->ifname);
    snprintf(value_str, sizeof(value_str), "%d", cfg->num_of_tc);
    res = yang_db_runtime_put_oneline(ydrd, buffer,
                                      value_str, YANG_DB_ONHW_NOACTION);
    if (res)
    {
        DPRINT("Failed to write one line of tc to the DB"LINE_FEED);
    }
    else
    {
        for (i = 0; i< cfg->cmd_entry_len; i++)
        {
            snprintf(buffer, sizeof(buffer),
                     OPERATION_NAME_FMT, cfg->ifname, i);
            strcpy(value_str, "set-gate-states");
            res = yang_db_runtime_put_oneline(ydrd, buffer,
                                              value_str, YANG_DB_ONHW_NOACTION);
            if (res)
            {
                DPRINT("Failed to set gate states"LINE_FEED);
                break;
            }

            n = snprintf(buffer, sizeof(buffer),
                         TIME_INTERVAL_KEY_FMT, cfg->ifname, i);
            if (n >= sizeof(buffer))
            {
                DPRINT("buffer overflow"LINE_FEED);
                res = -1;
                break;
            }
            n = snprintf(value_str, sizeof(value_str), "%d",
                         cfg->cmd_entry[i].time_interval);
            if (n >= sizeof(value_str))
            {
                DPRINT("value_str overflow"LINE_FEED);
                res = -1;
                break;
            }
            res = yang_db_runtime_put_oneline(ydrd, buffer, value_str,
                                              YANG_DB_ONHW_NOACTION);
            if (res) {
                DPRINT("Failed to write one line to the DB"LINE_FEED);
                break;
            }

            snprintf(buffer, sizeof(buffer), GATE_MASK_KEY_FMT, cfg->ifname, i);
            snprintf(value_str, sizeof(value_str), "%d",
                     cfg->cmd_entry[i].gate_state_mask);
            res = yang_db_runtime_put_oneline(ydrd, buffer, value_str,
                                              YANG_DB_ONHW_NOACTION);
            if (res)
            {
                DPRINT("Failed to write one line of gate state to DB"LINE_FEED);
                break;
            }
        }

        while (res == 0)
        {
            snprintf(buffer, sizeof(buffer), CYCLE_TIME_NUM_FMT, cfg->ifname);
            snprintf(value_str, sizeof(value_str), "%d",
                     cfg->cycle_time_numerator);
            res = yang_db_runtime_put_oneline(ydrd, buffer, value_str,
                                              YANG_DB_ONHW_NOACTION);
            if (res)
            {
                DPRINT("Failed to write cycletime numerator the DB"LINE_FEED);
                break;
            }
            snprintf(buffer, sizeof(buffer), CYCLE_TIME_DENOMINATOR_FMT, cfg->ifname);
            snprintf(value_str, sizeof(value_str), "%d", cfg->cycle_time_denominator);
            res = yang_db_runtime_put_oneline(ydrd, buffer, value_str, YANG_DB_ONHW_NOACTION);
            if (res)
            {
                DPRINT("Failed to write cycletime denominator the DB"LINE_FEED);
                break;
            }

            snprintf(buffer, sizeof(buffer), BASE_TIME_SECONDS_FMT, cfg->ifname);
            snprintf(value_str, sizeof(value_str), "%d", cfg->base_time_seconds);
            res = yang_db_runtime_put_oneline(ydrd, buffer, value_str, YANG_DB_ONHW_NOACTION);
            if (res)
            {
                DPRINT("Failed to set basetime seconds to the DB"LINE_FEED);
                break;
            }

            snprintf(buffer, sizeof(buffer), BASE_TIME_NANOSECONDS_FMT, cfg->ifname);
            snprintf(value_str, sizeof(value_str), "%d", cfg->base_time_nanoseconds);
            res = yang_db_runtime_put_oneline(ydrd, buffer, value_str, YANG_DB_ONHW_NOACTION);
            if (res)
            {
                DPRINT("Failed to set basetime nanoseconds to the DB"LINE_FEED);
                break;
            }
            snprintf(buffer, sizeof(buffer), GATE_ENABLE_FMT, cfg->ifname);
            snprintf(value_str, sizeof(value_str), "%s", cfg->gate_enabled? "true": "false");
            res = yang_db_runtime_put_oneline(ydrd, buffer, value_str, YANG_DB_ONHW_NOACTION);
            if (res)
            {
                DPRINT("Failed to set gate_enabled to the DB"LINE_FEED);
            }
            break;
        }
    }
}

#define OPER_TIME_INTERVAL_KEY_FMT ("/ietf-interfaces/interfaces/interface|name:%s|/bridge-port/gate-parameter-table/oper-control-list/gate-control-entry|index:%d|/time-interval-value")
#define OPER_GATE_MASK_KEY_FMT ("/ietf-interfaces/interfaces/interface|name:%s|/bridge-port/gate-parameter-table/oper-control-list/gate-control-entry|index:%d|/gate-states-value")

void show_tas_config_parameter(yang_db_runtime_dataq_t *ydrd,
                               const char *ifname)
{
    char buffer[MAX_KEY_SIZE];
    int res;
    uint32_t cycle_time_numerator = 0, cycle_time_denominator = 0;
    uint64_t base_time_seconds = 0;
    uint32_t base_time_nanoseconds = 0;
    uint32_t time_interval[8] = {0};
    uint8_t gate_mask[8] = {0};
    int i;

    void *value = NULL;
    uint32_t vsize = 0;
    snprintf(buffer, sizeof(buffer), OPER_CYCLE_TIME_NUM_FMT, ifname);
    res=yang_db_runtime_get_oneline(ydrd, buffer, &value, &vsize);
    if (res < 0)
    {
        DPRINT("Failed to get oper cycle time numerator from the DB"LINE_FEED);
    } else
    {
        cycle_time_numerator = *(uint32_t *)value;
        Z_FREE(value);
    }

    snprintf(buffer, sizeof(buffer), OPER_CYCLE_TIME_DENOMINATOR_FMT, ifname);
    res=yang_db_runtime_get_oneline(ydrd, buffer, &value, &vsize);
    if (res < 0)
    {
        DPRINT("Failed to get oper cycle time demoninator from DB"LINE_FEED);
    } else
    {
        cycle_time_denominator = *(uint32_t *)value;
        Z_FREE(value);
    }

    snprintf(buffer, sizeof(buffer), OPER_BASE_TIME_SECONDS_FMT, ifname);
    res=yang_db_runtime_get_oneline(ydrd, buffer, &value, &vsize);
    if (res < 0)
    {
        DPRINT("Faild to get oper base time seconds from the DB"LINE_FEED);
    }
    else
    {
        base_time_seconds = *(uint64_t *)value;
        Z_FREE(value);
    }
    snprintf(buffer, sizeof(buffer), OPER_BASE_TIME_NANOSECONDS_FMT, ifname);
    res=yang_db_runtime_get_oneline(ydrd, buffer, &value, &vsize);
    if (res < 0)
    {
        DPRINT("Faild to get oper base time nanoseconds from the DB"LINE_FEED);
    }
    else
    {
        base_time_nanoseconds = *(uint32_t *)value;
        Z_FREE(value);
    }
    DPRINT("Oper parameters: basetimesec: %lld, basetimenanosec: %d, cycletimenum: %d, cycletimedenom: %d"LINE_FEED,
           base_time_seconds, base_time_nanoseconds, cycle_time_numerator, cycle_time_denominator);
    for (i = 0; i < 8; i++)
    {
        snprintf(buffer, sizeof(buffer), OPER_TIME_INTERVAL_KEY_FMT, ifname, i);
        res = yang_db_runtime_get_oneline(ydrd, buffer, &value, &vsize);
        if (res < 0)
        {
            DPRINT("Failed to get the oper time interval from DB"LINE_FEED);
        }
        else
        {
            time_interval[i] = *(uint32_t *)value;
            Z_FREE(value);
        }

        snprintf(buffer, sizeof(buffer), OPER_GATE_MASK_KEY_FMT, ifname, i);
        res = yang_db_runtime_get_oneline(ydrd, buffer, &value, &vsize);
        if (res < 0)
        {
            DPRINT("Failed to get the gatemask from DB"LINE_FEED);
        }
        else
        {
            gate_mask[i] = *(uint8_t *)value;
            Z_FREE(value);
        }
    }
    DPRINT("oper-list: time_interval: %d %d %d %d %d %d %d %d"LINE_FEED,
           time_interval[0], time_interval[1], time_interval[2], time_interval[3],
           time_interval[4], time_interval[5], time_interval[6], time_interval[7]);
    DPRINT("oper-list: gatemask: %02x %02x %02x %02x %02x %02x %02x %02x"LINE_FEED,
           gate_mask[0], gate_mask[1], gate_mask[2], gate_mask[3],
           gate_mask[4], gate_mask[5], gate_mask[6], gate_mask[7]);
}

#define FRAME_PREEMPT_PRIORITY_FMT ("/ietf-interfaces/interfaces/interface|name:%s|/bridge-port/frame-preemption-parameters/frame-preemption-status-table/priority%d")

struct frame_preempt_config_param
{
    const char *ifname;
    uint8_t prioiry_preempt[8];
};

static void
set_frame_preempt_config_parameter(yang_db_runtime_dataq_t *ydrd,
                                   struct frame_preempt_config_param *prm)
{
    char buffer[MAX_KEY_SIZE];
    char value_str[MAX_VALUE_SIZE];
    int i, res;

    for (i = 0; i < 8; i++)
    {
        snprintf(buffer, sizeof(buffer), FRAME_PREEMPT_PRIORITY_FMT, prm->ifname, i);
        snprintf(value_str, sizeof(value_str), "%d", prm->prioiry_preempt[i]);
        res = yang_db_runtime_put_oneline(ydrd, buffer, value_str, YANG_DB_ONHW_NOACTION);
        if (res)
        {
            DPRINT("Failed to set preemption status for priority %d "LINE_FEED, i);
            break;
        }
    }
}

#define OPER_STATUS_FMT ("/ietf-interfaces/interfaces/interface|name:%s|/oper-status")
#define IF_INDEX_FMT ("/ietf-interfaces/interfaces/interface|name:%s|/if-index")
#define PHYS_ADDRESS_FMT ("/ietf-interfaces/interfaces/interface|name:%s|/phys-address")
#define SPEED_FMT ("/ietf-interfaces/interfaces/interface|name:%s|/speed")
static void show_link_status(yang_db_runtime_dataq_t *ydrd, const char *ifname)
{
    char buffer[MAX_KEY_SIZE];
    int res;
    void *value = NULL;
    uint32_t vsize = 0;
    uint32_t oper_status, index;
    uint8_t macaddr[6];
    uint64_t speed;

    snprintf(buffer, sizeof(buffer), OPER_STATUS_FMT, ifname);
    res=yang_db_runtime_get_oneline(ydrd, buffer, &value, &vsize);
    if (res < 0)
    {
        DPRINT("Faild to get oper-status from DB"LINE_FEED);
    }
    oper_status = *(uint32_t *)value;
    Z_FREE(value);

    snprintf(buffer, sizeof(buffer), IF_INDEX_FMT, ifname);
    res=yang_db_runtime_get_oneline(ydrd, buffer, &value, &vsize);
    if (res < 0)
    {
        DPRINT("Faild to get if-index from DB"LINE_FEED);
    }
    index = *(uint32_t *)value;
    Z_FREE(value);

    snprintf(buffer, sizeof(buffer), PHYS_ADDRESS_FMT, ifname);
    res=yang_db_runtime_get_oneline(ydrd, buffer, &value, &vsize);
    if (res < 0)
    {
        DPRINT("Faild to get mac address from DB"LINE_FEED);
    }
    memcpy(macaddr, value, sizeof(macaddr));
    Z_FREE(value);

    snprintf(buffer, sizeof(buffer), SPEED_FMT, ifname);
    res=yang_db_runtime_get_oneline(ydrd, buffer, &value, &vsize);
    if (res < 0)
    {
        DPRINT("Faild to get speed from DB"LINE_FEED);
    }
    speed = *(uint64_t *)value;
    Z_FREE(value);

    DPRINT("Link: %s, if-idx: %d, speed: %lld, MAC:%02x %02x %02x %02x %02x %02x "LINE_FEED,
           oper_status==1?"UP": "DOWN", index, speed, macaddr[0],
           macaddr[1], macaddr[2], macaddr[3], macaddr[4], macaddr[5]);
}

// return 1: no wait item, 0: got the wait item, -1: can't get the wait item
int main_loop(ucclient_opt_t *optd)
{
    xl4_data_data_t *xdd = NULL;
    uc_dbald *dbald = NULL;
    uc_notice_data_t *ucntd = NULL;
    yang_db_runtime_dataq_t *ydrd = NULL;
    int res = -1;

    do
    {
        if (uniconf_ready(optd->dbname, UC_CALLMODE_THREAD, optd->waitdb))
        {
            DPRINT("Uniconf has not been ready"LINE_FEED);
            break;
        }

        dbald = uc_dbal_open(optd->dbname, "w", UC_CALLMODE_THREAD);
        if (!dbald)
        {
            break;
        }
        xdd = xl4_data_init(dbald);
        if (!xdd)
        {
            break;
        }
        ucntd = uc_notice_init(UC_CALLMODE_THREAD);
        if (!ucntd)
        {
            DPRINT("Failed to intialize uc notice"LINE_FEED);
            break;
        }
        ydrd = yang_db_runtime_init(xdd, dbald, NULL);
        if (!ydrd)
        {
            break;
        }

        int i;
        uint32_t intervals[CMD_LIST_LENGTH] =
            {125200, 300000, 200000, 100000, 200000, 300000, 400000, 1000000};
        uint8_t gates[CMD_LIST_LENGTH] =
            {0xFF, 0x0F, 0xF0, 0xFF, 0x0F, 0xF0, 0xFF, 0x0F};
        struct tas_config_param cfg_prm =
        {
            .ifname = *(optd->netdevs),
            .base_time_seconds = 0,
            .base_time_nanoseconds = 0,
            .gate_enabled = true,
            .cycle_time_numerator = 1,
            .cycle_time_denominator = 500,
            .cmd_entry_len = CMD_LIST_LENGTH,
            .num_of_tc = 2,
        };

        for (i = 0; i < cfg_prm.cmd_entry_len; i++)
        {
            cfg_prm.cmd_entry[i].time_interval = intervals[i];
            cfg_prm.cmd_entry[i].gate_state_mask = gates[i];
        }
        set_tas_config_parameter(ydrd, &cfg_prm);

        // Since the gate is enabled, ask the uniconf to configure HW for TAS.
        if (cfg_prm.gate_enabled)
        {
            (void)yang_db_runtime_askaction(ydrd, ucntd);
        }

        // Set config for frame preemtion
        struct frame_preempt_config_param preempt_prm =
        {
            .ifname = *(optd->netdevs),
            .prioiry_preempt = {1, 1, 1, 1, 2, 2, 2, 2}, //1: express; 2: preemptable.
        };
        set_frame_preempt_config_parameter(ydrd, &preempt_prm);
        (void)yang_db_runtime_askaction(ydrd, ucntd);

        while (true)
        {
            CB_USLEEP(5000000UL);
            show_link_status(ydrd, *(optd->netdevs));
            show_tas_config_parameter(ydrd, *(optd->netdevs));
        }
    } while (0);
    if(ucntd)
    {
        uc_notice_close(ucntd, 0);
    }
    if(ydrd)
    {
        yang_db_runtime_close(ydrd);
    }
    if(xdd)
    {
        xl4_data_close(xdd);
    }
    uc_dbal_close(dbald, 0);
    return res;
}

void* uc_client_main(void *arg)
{
    char **netdevs = (char **)arg;
    ucclient_opt_t optd;
    memset(&optd, 0, sizeof(optd));
    optd.waitdb = WAIT_DB_TIME_MS;
    optd.netdevs = netdevs;
    main_loop(&optd);

    return NULL;
}
