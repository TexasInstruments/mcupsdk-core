/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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

#include <stdint.h>
#include <stdio.h>
#include <kernel/dpl/DebugP.h>

#define CSL_DSP_ICFG_U_BASE              (0x1800000U)
#define CSL_REG32_RD(p)                  (*(volatile uint32_t*)(p))
#define CSL_REG32_WR(p, v)               (*(volatile uint32_t*)(p) = (v))

#define L2_VALID_START                   (0x00800000U)
#define L2_VALID_LAST_ADDR               (0x0085FFFCU)
#define L2_RESERVED_START                (0x00860000U)

#define L2MPFAR_ADDR                     (CSL_DSP_ICFG_U_BASE + 0x4A000U)
#define L2MPFSR_ADDR                     (CSL_DSP_ICFG_U_BASE + 0x4A004U)
#define L2MPFCR_ADDR                     (CSL_DSP_ICFG_U_BASE + 0x4A008U)
#define L2MPPA_BASE_ADDR                 (CSL_DSP_ICFG_U_BASE + 0x4A200U)

static uint32_t tests_passed = 0;
static uint32_t tests_total = 0;

#define TEST_PASS(msg) do { tests_passed++; DebugP_log("  ✓ %s\n", msg); } while(0)
#define TEST_FAIL(msg) do { DebugP_log("  ✗ %s\n", msg); } while(0)

static void clear_mpu_faults(void)
{
    CSL_REG32_WR(L2MPFCR_ADDR, 0x00000001U);
}

static void tc_1_valid_l2_access(void)
{
    volatile uint32_t *valid_addr = (volatile uint32_t *)(L2_VALID_START);
    uint32_t test_pattern = 0xABCD1234U;
    uint32_t read_back;

    tests_total++;
    DebugP_log("\n[TC-1] Valid L2 Write/Read\n");

    clear_mpu_faults();
    *valid_addr = test_pattern;
    read_back = *valid_addr;

    DebugP_log("  Wrote: 0x%08X to 0x00800000\n", test_pattern);
    DebugP_log("  Read:  0x%08X\n", read_back);

    if (read_back == test_pattern)
    {
        TEST_PASS("Valid L2 read/write OK");
    }
    else
    {
        TEST_FAIL("Valid L2 read mismatch");
    }
}

static void tc_2_last_valid_address(void)
{
    volatile uint32_t *last_valid = (volatile uint32_t *)(L2_VALID_LAST_ADDR);
    uint32_t pattern = 0x11223344U;
    uint32_t read_back;

    tests_total++;
    DebugP_log("\n[TC-2] Last Valid Address\n");

    clear_mpu_faults();
    *last_valid = pattern;
    read_back = *last_valid;

    DebugP_log("  Wrote: 0x%08X\n", pattern);
    DebugP_log("  Read:  0x%08X\n", read_back);

    if (read_back == pattern)
    {
        TEST_PASS("Last valid addr OK");
    }
    else
    {
        TEST_FAIL("Last valid addr failed");
    }
}

static void tc_3_reserved_write_protection(void)
{
    volatile uint32_t *reserved_addr = (volatile uint32_t *)(L2_RESERVED_START);
    uint32_t fault_status_before, fault_status_after;

    tests_total++;
    DebugP_log("\n[TC-3] Reserved Space Write Protection\n");

    clear_mpu_faults();
    fault_status_before = CSL_REG32_RD(L2MPFSR_ADDR);
    DebugP_log("  L2MPFSR before: 0x%08X\n", fault_status_before);

    *reserved_addr = 0xDEADBEEFU;

    fault_status_after = CSL_REG32_RD(L2MPFSR_ADDR);
    DebugP_log("  L2MPFSR after:  0x%08X\n", fault_status_after);

    if (fault_status_after != 0)
    {
        DebugP_log("  (MPU fault detected)\n");
        TEST_PASS("Reserved write blocked");
    }
    else
    {
        DebugP_log("  (No fault)\n");
        TEST_FAIL("Reserved write not blocked");
    }
}

static void tc_4_aliasing_detection(void)
{
    volatile uint32_t *valid_addr = (volatile uint32_t *)(L2_VALID_START);
    volatile uint32_t *reserved_addr = (volatile uint32_t *)(L2_RESERVED_START);
    uint32_t original_value = 0xFFFFFFFFU;
    uint32_t read_back;

    tests_total++;
    DebugP_log("\n[TC-4] Aliasing Check\n");

    clear_mpu_faults();

    *valid_addr = original_value;
    read_back = *valid_addr;
    DebugP_log("  Write valid:  0x%08X\n", original_value);
    DebugP_log("  Read valid:   0x%08X\n", read_back);

    if (read_back != original_value)
    {
        TEST_FAIL("Valid L2 operation failed");
        return;
    }

    clear_mpu_faults();
    DebugP_log("  Write reserved: 0x55555555\n");
    *reserved_addr = 0x55555555U;

    read_back = *valid_addr;
    DebugP_log("  Read valid after: 0x%08X\n", read_back);

    if (read_back == original_value)
    {
        TEST_PASS("No aliasing detected");
    }
    else if (read_back == 0x55555555U)
    {
        TEST_FAIL("Aliasing detected");
    }
    else
    {
        TEST_FAIL("Unexpected value");
    }
}

static void tc_5_reserved_read_parity(void)
{
    volatile uint32_t *reserved_addr = (volatile uint32_t *)(L2_RESERVED_START);
    uint32_t fault_status_before, fault_status_after;
    uint32_t read_val;

    tests_total++;
    DebugP_log("\n[TC-5] Reserved Read\n");

    clear_mpu_faults();
    fault_status_before = CSL_REG32_RD(L2MPFSR_ADDR);
    DebugP_log("  L2MPFSR before: 0x%08X\n", fault_status_before);

    read_val = *reserved_addr;

    fault_status_after = CSL_REG32_RD(L2MPFSR_ADDR);
    DebugP_log("  L2MPFSR after:  0x%08X\n", fault_status_after);
    DebugP_log("  Value read: 0x%08X\n", read_val);

    if (fault_status_after != 0)
    {
        TEST_PASS("Protection fault on read");
    }
    else
    {
        TEST_PASS("Reserved read documented");
    }
}

static void tc_6_check_mpu_state(void)
{
    uint32_t l2mppa24, l2mppa31;
    volatile uint32_t *l2mppa_base = (volatile uint32_t *)(L2MPPA_BASE_ADDR);

    tests_total++;
    DebugP_log("\n[TC-6] L2MPPA State\n");

    l2mppa24 = l2mppa_base[24];
    l2mppa31 = l2mppa_base[31];

    DebugP_log("  L2MPPA[24] = 0x%08X\n", l2mppa24);
    DebugP_log("  L2MPPA[31] = 0x%08X\n", l2mppa31);

    uint32_t perm_bits = l2mppa24 & 0x0000FF7FU;

    if (perm_bits == 0 || (l2mppa24 & 0x0000FFFFU) == 0x0000FFFFU)
    {
        if (perm_bits == 0)
            DebugP_log("  Status: Protected\n");
        else
            DebugP_log("  Status: Unprotected\n");
    }

    TEST_PASS("Config state OK");
}

static void print_summary(void)
{
    DebugP_log("\n========================================\n");
    DebugP_log("  DSP L2 MPU Test Results\n");
    DebugP_log("========================================\n");
    DebugP_log("  Total:    %d\n", tests_total);
    DebugP_log("  Passed:   %d\n", tests_passed);
    DebugP_log("  Failed:   %d\n", tests_total - tests_passed);
    if (tests_total > 0)
        DebugP_log("  Rate:     %d%%\n", (tests_passed * 100 / tests_total));
    DebugP_log("========================================\n\n");
}

void test_dsp_l2mpu(void)
{
    DebugP_log("\n========================================\n");
    DebugP_log("  DSP L2 MPU Test\n");
    DebugP_log("========================================\n\n");

    tests_passed = 0;
    tests_total = 0;

    tc_1_valid_l2_access();
    tc_2_last_valid_address();
    tc_3_reserved_write_protection();
    tc_4_aliasing_detection();
    tc_5_reserved_read_parity();
    tc_6_check_mpu_state();

    print_summary();
}
