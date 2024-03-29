#ifndef __MAIN__H__
#define __MAIN__H__

#define TRANSFERSIZE (4*1024)

extern uint8_t __attribute__((aligned(4*1024), section(".data.flashSrcBuffer"))) sourceBuffer[TRANSFERSIZE];
extern uint8_t __attribute__((aligned(4*1024), section(".data.flashDestBuffer"))) destBuffer[TRANSFERSIZE];

void *test_flc_configuration(void*);
void *test_flc_runtimeconfig(void*);
void *test_flc_interrupt(void*);
void *test_flc_enable_disable(void*);
void *test_rl2_config(void *);
void *test_rat_config(void *);

#endif