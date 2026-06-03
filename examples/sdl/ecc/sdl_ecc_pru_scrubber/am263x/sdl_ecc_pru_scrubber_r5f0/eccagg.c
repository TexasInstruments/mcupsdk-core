
#include <drivers/soc.h>

#define ECC_VECTOR_REG_ADDR             (CSL_ECC_AGG_TOP_U_BASE + CSL_MSS_ECC_AGG_MSS_ECC_VECTOR)

uint32_t ECCAGG_readRegister(uint8_t endpointId, uint16_t registerOffset)
{
    // Read mask includes register offset to read from, setting read bit to true, and endpoint ID
    uint32_t read_mask = ((uint32_t)registerOffset << CSL_MSS_ECC_AGG_MSS_ECC_VECTOR_RD_SVBUS_ADDRESS_SHIFT) | 
                                                       (1UL << CSL_MSS_ECC_AGG_MSS_ECC_VECTOR_RD_SVBUS_SHIFT) | 
                                              (endpointId << CSL_MSS_ECC_AGG_MSS_ECC_VECTOR_ECC_VECTOR_SHIFT);
    HW_WR_REG32(ECC_VECTOR_REG_ADDR, read_mask);

    // measured worst-case for this serial read is 35ms based on GPIO toggle
    volatile uint32_t counter = 0x800000; //roughly 70 ms wait cycle assuming below loop takes 2 cycles
    do
    {
        __asm("NOP");
    } while ((counter-- != 0U) && (((HW_RD_REG32(ECC_VECTOR_REG_ADDR) >> CSL_MSS_ECC_AGG_MSS_ECC_VECTOR_RD_SVBUS_DONE_SHIFT) & 1UL) == 0UL)); // TRM says to poll this bit until its set, indicating successful read

    return HW_RD_REG32(CSL_ECC_AGG_TOP_U_BASE + registerOffset);
}

bool ECCAGG_writeRegister(uint8_t endpointId, uint16_t registerOffset, uint32_t val)
{
    // Write mask explicitly clears read bit, sets endpoint ID
    uint32_t write_mask = (0UL << CSL_MSS_ECC_AGG_MSS_ECC_VECTOR_RD_SVBUS_SHIFT) | (endpointId << CSL_MSS_ECC_AGG_MSS_ECC_VECTOR_ECC_VECTOR_SHIFT);
    HW_WR_REG32(ECC_VECTOR_REG_ADDR, write_mask);
    HW_WR_REG32((CSL_ECC_AGG_TOP_U_BASE + registerOffset), val);

    // Ensure write went through
    uint32_t read_result = ECCAGG_readRegister(endpointId, registerOffset);
    return (read_result == val);
}
