#ifndef __LOAD_H__
#define __LOAD_H__



#define MMR_READ_0(pDest) do { (void)pDest; } while(0)
#define MMR_READ_1(pDest) do { *(pDest) = *(volatile uint16_t*)0x53808000; } while(0)
#define MMR_READ_2(pDest) do {MMR_READ_1(pDest); MMR_READ_1(pDest);} while(0)
#define MMR_READ_4(pDest) do {MMR_READ_2(pDest); MMR_READ_2(pDest);} while(0)
#define MMR_READ_8(pDest) do {MMR_READ_4(pDest); MMR_READ_4(pDest);} while(0)
#define MMR_READ_16(pDest) do {MMR_READ_8(pDest); MMR_READ_8(pDest);} while(0)

#define MMR_WRITE_0(pVal) do { (void)pDest; } while(0)
#define MMR_WRITE_1(pVal) do { *(volatile uint32_t*)0 = pVal; } while(0)
#define MMR_WRITE_2(pVal) do {MMR_WRITE_1(pVal); MMR_WRITE_1(pVal);} while(0)
#define MMR_WRITE_4(pVal) do {MMR_WRITE_2(pVal); MMR_WRITE_2(pVal);} while(0)
#define MMR_WRITE_8(pVal) do {MMR_WRITE_4(pVal); MMR_WRITE_4(pVal);} while(0)
#define MMR_WRITE_16(pVal) do {MMR_WRITE_8(pVal); MMR_WRITE_8(pVal);} while(0)

#define MMR_COMP1_0(pSrc, pDest) do { (void)pSrc;(void)pDest; } while(0)
#define MMR_COMP1_1(pSrc, pDest) do { *(pDest) = *(pSrc) / 4096; } while(0)
#define MMR_COMP1_2(pSrc, pDest) do {MMR_COMP1_1(pSrc, pDest); MMR_COMP1_1(pSrc, pDest);} while(0)
#define MMR_COMP1_4(pSrc, pDest) do {MMR_COMP1_2(pSrc, pDest); MMR_COMP1_2(pSrc, pDest);} while(0)
#define MMR_COMP1_8(pSrc, pDest) do {MMR_COMP1_4(pSrc, pDest); MMR_COMP1_4(pSrc, pDest);} while(0)
#define MMR_COMP1_16(pSrc, pDest) do {MMR_COMP1_8(pSrc, pDest); MMR_COMP1_8(pSrc, pDest);} while(0)

#define MMR_COMP2_0(pSrc, pDest) do { (void)pSrc; (void)pDest; } while(0)
#define MMR_COMP2_1(pSrc, pDest) do { *(pDest) = *(pSrc) * 2; } while(0)
#define MMR_COMP2_2(pSrc, pDest) do {MMR_COMP2_1(pSrc, pDest); MMR_COMP2_1(pSrc, pDest);} while(0)
#define MMR_COMP2_4(pSrc, pDest) do {MMR_COMP2_2(pSrc, pDest); MMR_COMP2_2(pSrc, pDest);} while(0)
#define MMR_COMP2_8(pSrc, pDest) do {MMR_COMP2_4(pSrc, pDest); MMR_COMP2_4(pSrc, pDest);} while(0)
#define MMR_COMP2_16(pSrc, pDet) do {MMR_COMP2_8(pSrc, pDest); MMR_COMP2_8(pSrc, pDest);} while(0)


#define COMP_BLOCK_001(READCNT, WRCNT, COMP1CNT, COMP2CNT) do{\
                                                                volatile uint16_t readData = 0;\
                                                                (void)readData;\
                                                                MMR_READ_##READCNT (&readData); \
                                                                MMR_COMP1_##COMP1CNT (&readData, &readData); \
                                                                MMR_COMP2_##COMP2CNT (&readData, &readData); \
                                                                MMR_WRITE_##WRCNT (readData); \
                                                            } while(0);

#define COMP_BLOCK_002(READCNT, WRCNT, COMP1CNT, COMP2CNT) do{COMP_BLOCK_001(READCNT, WRCNT, COMP1CNT, COMP2CNT); COMP_BLOCK_001(READCNT, WRCNT, COMP1CNT, COMP2CNT);} while(0);
#define COMP_BLOCK_004(READCNT, WRCNT, COMP1CNT, COMP2CNT) do{COMP_BLOCK_002(READCNT, WRCNT, COMP1CNT, COMP2CNT); COMP_BLOCK_002(READCNT, WRCNT, COMP1CNT, COMP2CNT);} while(0);
#define COMP_BLOCK_008(READCNT, WRCNT, COMP1CNT, COMP2CNT) do{COMP_BLOCK_004(READCNT, WRCNT, COMP1CNT, COMP2CNT); COMP_BLOCK_004(READCNT, WRCNT, COMP1CNT, COMP2CNT);} while(0);
#define COMP_BLOCK_016(READCNT, WRCNT, COMP1CNT, COMP2CNT) do{COMP_BLOCK_008(READCNT, WRCNT, COMP1CNT, COMP2CNT); COMP_BLOCK_008(READCNT, WRCNT, COMP1CNT, COMP2CNT);} while(0);
#define COMP_BLOCK_032(READCNT, WRCNT, COMP1CNT, COMP2CNT) do{COMP_BLOCK_016(READCNT, WRCNT, COMP1CNT, COMP2CNT); COMP_BLOCK_016(READCNT, WRCNT, COMP1CNT, COMP2CNT);} while(0);
#define COMP_BLOCK_064(READCNT, WRCNT, COMP1CNT, COMP2CNT) do{COMP_BLOCK_032(READCNT, WRCNT, COMP1CNT, COMP2CNT); COMP_BLOCK_032(READCNT, WRCNT, COMP1CNT, COMP2CNT);} while(0);
#define COMP_BLOCK_128(READCNT, WRCNT, COMP1CNT, COMP2CNT) do{COMP_BLOCK_064(READCNT, WRCNT, COMP1CNT, COMP2CNT); COMP_BLOCK_064(READCNT, WRCNT, COMP1CNT, COMP2CNT);} while(0);
#define COMP_BLOCK_256(READCNT, WRCNT, COMP1CNT, COMP2CNT) do{COMP_BLOCK_128(READCNT, WRCNT, COMP1CNT, COMP2CNT); COMP_BLOCK_128(READCNT, WRCNT, COMP1CNT, COMP2CNT);} while(0);
#define COMP_BLOCK_512(READCNT, WRCNT, COMP1CNT, COMP2CNT) do{COMP_BLOCK_256(READCNT, WRCNT, COMP1CNT, COMP2CNT); COMP_BLOCK_256(READCNT, WRCNT, COMP1CNT, COMP2CNT);} while(0);

__attribute__((aligned(4096))) void load_function_t1_sram(void);
__attribute__((aligned(4096), section(".xip"))) void load_function_t1_xip(void);

__attribute__((aligned(4096))) void load_function_t2_sram(void);
__attribute__((aligned(4096), section(".xip"))) void load_function_t2_xip(void);

__attribute__((aligned(4096))) void load_function_t3_sram(void);
__attribute__((aligned(4096), section(".xip"))) void load_function_t3_xip(void);

__attribute__((aligned(4096))) void load_function_t4_sram(void);
__attribute__((aligned(4096), section(".xip"))) void load_function_t4_xip(void);

#endif ///__LOAD_H__