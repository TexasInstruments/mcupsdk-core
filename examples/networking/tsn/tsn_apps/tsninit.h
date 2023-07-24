#ifndef __TSNINIT_H__
#define __TSNINIT_H__

typedef struct tsn_app_cfg {
	on_console_out console_out; //<! A callback function for log output on console.
	char *netdevs[MAX_NUMBER_ENET_DEVS+1]; //!< A list of network interfaces each is a string, terminated by NULL;
} tsn_app_cfg_t;

int tsn_app_init(tsn_app_cfg_t *cfg);

void tsn_app_deinit(void);

int tsn_app_start(void);
void tsn_app_stop(void);

#endif
