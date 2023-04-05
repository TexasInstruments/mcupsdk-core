/*
 * Copyright (C) 2023 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "lwip/apps/mqtt.h"
#include "mqtt.h"
#include <kernel/dpl/ClockP.h>

#if MQTT_HAVE_TLS
#include "client_info.h"
#endif
#include "lwip/altcp_tls.h"

#if MQTT_HAVE_TLS
#define MQTT_USE_PORT   8883U
#else
#define MQTT_USE_PORT   1883U
#endif

#if LWIP_TCP

/* IP address for the MQTT Mosquitto broker (192.168.1.2) in this case*/
#define LWIP_MQTT_EXAMPLE_IPADDR_INIT = IPADDR4_INIT((0x0201a8c0UL))
#ifndef LWIP_MQTT_EXAMPLE_IPADDR_INIT
#if LWIP_IPV4
#define LWIP_MQTT_EXAMPLE_IPADDR_INIT = IPADDR4_INIT(PP_HTONL(IPADDR_LOOPBACK))
#else
#define LWIP_MQTT_EXAMPLE_IPADDR_INIT
#endif
#endif

/* Print to UART */
extern void EnetAppUtils_print(const char *pcString, ...);
#ifdef printf
#undef printf
#endif
#define printf EnetAppUtils_print

#define MQTT_PUBLISHER      0
#define PUBLISH_LENGTH      30

static ip_addr_t mqtt_ip LWIP_MQTT_EXAMPLE_IPADDR_INIT;
static mqtt_client_t* mqtt_client;

static struct mqtt_connect_client_info_t mqtt_client_info =
{
  "test",
  "am243x_evm", /* user */
  "1234", /* pass */
  100,  /* keep alive */
  NULL, /* will_topic */
  NULL, /* will_msg */
  0,    /* will_qos */
  0     /* will_retain */
#if LWIP_ALTCP && LWIP_ALTCP_TLS
  , NULL
#endif
};


static void
mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
  const struct mqtt_connect_client_info_t* client_info = (const struct mqtt_connect_client_info_t*)arg;

  LWIP_PLATFORM_DIAG(("MQTT client \"%s\" data received: %s, data len: %d bytes\r\n", client_info->client_id, data,(int)len));
}

static void
mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
  const struct mqtt_connect_client_info_t* client_info = (const struct mqtt_connect_client_info_t*)arg;

  LWIP_PLATFORM_DIAG(("MQTT client \"%s\" subscribed to topic: %s\r\n", client_info->client_id, topic));
}

static void
mqtt_request_cb(void *arg, err_t err)
{
  const struct mqtt_connect_client_info_t* client_info = (const struct mqtt_connect_client_info_t*)arg;
  switch(err)
  {
    case ERR_OK:
        LWIP_PLATFORM_DIAG(("MQTT Client \"%s\" connection accepted\r\n", client_info->client_id));
        break;
    case ERR_ABRT:
        LWIP_PLATFORM_DIAG(("MQTT Client \"%s\" connection failed (Error abort)\r\n", client_info->client_id));
        break;
    case ERR_TIMEOUT:
        LWIP_PLATFORM_DIAG(("MQTT Client \"%s\" connection failed (Connection timed out)\r\n", client_info->client_id));
        break;
    default:
        break;
  }
}

static void
mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
  const struct mqtt_connect_client_info_t* client_info = (const struct mqtt_connect_client_info_t*)arg;

  switch(status)
  {
    case 1:
        LWIP_PLATFORM_DIAG(("MQTT Client connection failed (Refused Protocol Version)\r\n"));
        break;
    case 2:
        LWIP_PLATFORM_DIAG(("MQTT Client connection failed (Refused Identifier)\r\n"));
        break;
    case 3:
        LWIP_PLATFORM_DIAG(("MQTT Client connection failed (Refused server)\r\n"));
        break;
    case 4:
        LWIP_PLATFORM_DIAG(("MQTT Client connection failed (Refused username or password)\r\n"));
        break;
    case 5:
        LWIP_PLATFORM_DIAG(("MQTT Client connection failed (not authorized)\r\n"));
        break;
    case 256:
        LWIP_PLATFORM_DIAG(("MQTT Client connection failed (Connect Disconnected)\r\n"));
        break;
    case 257:
        LWIP_PLATFORM_DIAG(("MQTT Client connection failed (Connection timed out)\r\n"));
        break;
    default:
        break;
  }

  if (status == MQTT_CONNECT_ACCEPTED) {

    #if MQTT_PUBLISHER
        static char publishMessage[PUBLISH_LENGTH] = "";
        DebugP_log("Enter the message to publish)\r\n");
        DebugP_scanf("%s", &publishMessage[0]);
        mqtt_publish(client, "topic_qos1", &publishMessage[0], PUBLISH_LENGTH, 0, 1, mqtt_request_cb, LWIP_CONST_CAST(void*, client_info));
    #else
        mqtt_sub_unsub(client,
                "topic_qos1",1,
                mqtt_request_cb, LWIP_CONST_CAST(void*, client_info),
                1);
    #endif
  }
}
#endif /* LWIP_TCP */

void mqtt_example_init(void)
{
#if LWIP_TCP
    mqtt_client = mqtt_client_new();


#if MQTT_HAVE_TLS
    struct altcp_tls_config *tls_config_mqtt;
    tls_config_mqtt = altcp_tls_create_config_client_2wayauth(caCrt, caLen, PrivateKey, PrivateKey_Size, Pass, passLen, Certificate, Certificate_Size);

    if(tls_config_mqtt==NULL)
    {
        DebugP_log("Failed to create TLS config");
    }

    mqtt_client_info.tls_config = tls_config_mqtt;
#endif

    mqtt_client_connect(mqtt_client,
          &mqtt_ip, MQTT_USE_PORT,
          mqtt_connection_cb, LWIP_CONST_CAST(void*, &mqtt_client_info),
          &mqtt_client_info);

    mqtt_set_inpub_callback(mqtt_client,
        mqtt_incoming_publish_cb,
        mqtt_incoming_data_cb,
        LWIP_CONST_CAST(void*, &mqtt_client_info));

#endif /* LWIP_TCP */
}
