#ifndef __MAIN__H__
#define __MAIN__H__

#define TRANSFERSIZE (4*1024)

extern uint8_t __attribute__((aligned(4*1024), section(".data.flashSrcBuffer"))) sourceBuffer[TRANSFERSIZE];
extern uint8_t __attribute__((aligned(4*1024), section(".data.flashDestBuffer"))) destBuffer[TRANSFERSIZE];

#endif