/*
 *  Copyright (C) 2023-2024 Texas Instruments Incorporated
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
#include <errno.h>
#include <libgen.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include <inttypes.h>
#include <linux/vfio.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/param.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/eventfd.h>

/* maximum number of distinct PCIe MSI IRQ vectors */
#define PCIE_MSI_IRQ_MAX        32

/* offset into EP's config space to trigger downstream IRQs via HTI bit */
#define PCIE_EP_VNDR_SPC_CNTRL_REG_OFFSET   0x408
#define PCIE_EP_VNDR_SPC_CNTRL_REG_HTI_SET  0x00000100

/* Dma map size */
#define PCIE_SMPL_DMA_MAP_SIZE  64 * 1024

#define SMPL_BAR0_DATA_SIZE                     (16 * 1024)
#define SMPL_BAR1_DATA_SIZE                     (1 * 1024 * 1024)
#define SMPL_BAR2_DATA_SIZE                     (1 * 1024 * 1024)

#define SMPL_CTRL_INIT_DONE     0x1
#define SMPL_CTRL_RESET         0x2
#define SMPL_CTRL_COPY          0x4
#define SMPL_CTRL_TEST_MSI      0x8
#define SMPL_CTRL_TEST_BARs     0x10

/**
 * \brief structure of our sample EP's Bar0 memory
 */
struct bar0
{
    volatile struct config {
        uint32_t ctrl;
        uint32_t stat;
        uint64_t dma_addr;
        uint32_t dma_len;
    } config;
    uint8_t  pad_config[128 - sizeof (struct config)];
    uint32_t data[SMPL_BAR0_DATA_SIZE / sizeof (uint32_t)];
};

volatile struct bar0 *bar0_mem;

/* data buffer to test BAR1 & BAR2 functionality */
volatile uint32_t *bar1_data;
volatile uint32_t *bar2_data;

enum test_mode {
    TEST_MODE_DEFAULT,
    TEST_MODE_BARS,
};

/* 1: wait for user input before continuing */
int wait_user_enabled = 0;

/*****************************************************************************
 * Wait for user input (if desired)
 ****************************************************************************/
void wait_user()
{
    if (!wait_user_enabled)
        return;

    printf("CONTINUE WITH ENTER\n");
    while (getchar() != '\n')
        ;
}

/*****************************************************************************
 * Initialize VFIO
 ****************************************************************************/
int initVFIO(int bus, int device, int function, int groupID,
             int *container, int *deviceFD, uint64_t *config_region_offset)
{
    int ret = 0;
    int group;
    char path[PATH_MAX];

    struct vfio_group_status group_status = {.argsz = sizeof(group_status)};
    struct vfio_device_info  device_info  = {.argsz = sizeof(device_info)};
    struct vfio_region_info  config_region_info = {.argsz = sizeof(config_region_info)};

    printf("Using PCI device 0000:%02x:%02x.%d in IOMMU group %d\n", bus, device, function, groupID);

    /* boilerplate VFIO setup */
    {
        *container = open("/dev/vfio/vfio", O_RDWR);

        if (*container < 0)
        {
            printf("Failed to open /dev/vfio/vfio, %d (%s)\n", *container, strerror(errno));
            return *container;
        }

        snprintf(path, sizeof(path), "/dev/vfio/%d", groupID);

        group = open(path, O_RDWR);

        if (group < 0)
        {
            printf("Failed to open %s, %d (%s)\n", path, group, strerror(errno));
            return group;
        }

        ret = ioctl(group, VFIO_GROUP_GET_STATUS, &group_status);

        if (ret)
        {
            printf("ioctl(VFIO_GROUP_GET_STATUS) failed\n");
            return ret;
        }

        if (!(group_status.flags & VFIO_GROUP_FLAGS_VIABLE))
        {
            printf("Group not viable, are all devices attached to vfio?\n");
            return -1;
        }

       ret = ioctl(group, VFIO_GROUP_SET_CONTAINER, container);

        if (ret)
        {
            printf("Failed to set group container\n");
            return ret;
        }

        printf("VFIO_CHECK_EXTENSION VFIO_TYPE1_IOMMU: %sPresent\n", ioctl(*container, VFIO_CHECK_EXTENSION, VFIO_TYPE1_IOMMU) ? "" : "Not ");
        printf("VFIO_CHECK_EXTENSION VFIO_NOIOMMU_IOMMU: %sPresent\n", ioctl(*container, VFIO_CHECK_EXTENSION, VFIO_NOIOMMU_IOMMU) ? "" : "Not ");

        ret = ioctl(*container, VFIO_SET_IOMMU, VFIO_NOIOMMU_IOMMU);

        if (!ret)
        {
            printf("Incorrectly allowed no-IOMMU usage!\n");
            return -1;
        }

        ret = ioctl(*container, VFIO_SET_IOMMU, VFIO_TYPE1_IOMMU);

        if (ret)
        {
            printf("Failed to set IOMMU\n");
            return ret;
        }
    } /* boilerplate VFIO setup */

    /* get device file descriptor */
    snprintf(path, sizeof(path), "0000:%02x:%02x.%d", bus, device, function);

    *deviceFD = ioctl(group, VFIO_GROUP_GET_DEVICE_FD, path);

    if (*deviceFD < 0)
    {
        printf("Failed to get device %s\n", path);
        return -1;
    }

    /* get device info */
    ret = ioctl(*deviceFD, VFIO_DEVICE_GET_INFO, &device_info);

    if (ret)
    {
        printf("Failed to get device info\n");
        return -1;
    }

    /* get device region info for config space */
    config_region_info.index = VFIO_PCI_CONFIG_REGION_INDEX;

    ret = ioctl(*deviceFD, VFIO_DEVICE_GET_REGION_INFO, &config_region_info);

    if (ret)
    {
        printf("Failed to get config region info\n");
        return -1;
    }

    *config_region_offset = config_region_info.offset;

    printf("Config region info: region index 0x%x, size 0x%lx, offset 0x%lx, cap_offset 0x%x, flags 0x%x\n",
                                           config_region_info.index,
                            (unsigned long)config_region_info.size,
                            (unsigned long)config_region_info.offset,
                                           config_region_info.cap_offset,
                                           config_region_info.flags);

    return ret;
}

/*****************************************************************************
 * Allocate DMA buffer and map into EP's PCIe address space
 ****************************************************************************/
int mapDMA(int container, uint64_t *iova, uint32_t *size, void **vaddr)
{
    int ret;

    struct vfio_iommu_type1_dma_map dma_map = {.argsz = sizeof(dma_map)};

    /* Allocate DMA buffer */
    dma_map.vaddr = (uint64_t) mmap(0, PCIE_SMPL_DMA_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, 0, 0);
    dma_map.size  = PCIE_SMPL_DMA_MAP_SIZE;

    /* DMA buffer should be mapped to 0x0 from device view */
    dma_map.iova  = 0;
    dma_map.flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE;

    if (!dma_map.vaddr)
    {
        printf("Failed to allocate DMAable buffer\n");
        return -1;
    }

    /* setup a DMA mapping to (virtual) PCI address 0 */
    ret = ioctl(container, VFIO_IOMMU_MAP_DMA, &dma_map);

    if (ret)
    {
        printf("Failed to map DMAable buffer\n");
        return -1;
    }

    *iova  = dma_map.iova;
    *size  = dma_map.size;
    *vaddr = (void *) dma_map.vaddr;

    return ret;
}

/*****************************************************************************
 * Map device BAR0 to application memory
 ****************************************************************************/
int mapBAR0(int deviceFD)
{
    int ret;

    struct vfio_region_info  bar0_region_info = {.argsz = sizeof(bar0_region_info)};

    bar0_region_info.index = VFIO_PCI_BAR0_REGION_INDEX;

    ret = ioctl(deviceFD, VFIO_DEVICE_GET_REGION_INFO, &bar0_region_info);

    if (ret)
    {
        printf("Failed to get info\n");
        return -1;
    }

    printf("BAR0 Info: size 0x%lx, offset 0x%lx, flags 0x%x\n",
                            (unsigned long)bar0_region_info.size,
                            (unsigned long)bar0_region_info.offset,
                                           bar0_region_info.flags);

    bar0_mem = (struct bar0*) mmap(NULL, (size_t)bar0_region_info.size, PROT_READ | PROT_WRITE, MAP_SHARED, deviceFD, (off_t)bar0_region_info.offset);

    if (bar0_mem == MAP_FAILED)
    {
        printf("mmap failed\n");
        return -1;
    }

    return ret;
}

/*****************************************************************************
 * Map device BAR1 to data buffer
 ****************************************************************************/
int mapBAR1(int deviceFD, uint64_t *barSize)
{
    int ret;

    struct vfio_region_info  bar1_region_info = {.argsz = sizeof(bar1_region_info)};

    bar1_region_info.index = VFIO_PCI_BAR1_REGION_INDEX;

    ret = ioctl(deviceFD, VFIO_DEVICE_GET_REGION_INFO, &bar1_region_info);

    if (ret)
    {
        printf("Failed to get info\n");
        return -1;
    }

    printf("BAR1 Info: size 0x%lx, offset 0x%lx, flags 0x%x\n",
                            (unsigned long)bar1_region_info.size,
                            (unsigned long)bar1_region_info.offset,
                                           bar1_region_info.flags);

    bar1_data = (uint32_t *) mmap(NULL, (size_t)bar1_region_info.size, PROT_READ | PROT_WRITE, MAP_SHARED, deviceFD, (off_t)bar1_region_info.offset);

    if (bar1_data == MAP_FAILED)
    {
        printf("mmap failed\n");
        return -1;
    }

    *barSize = bar1_region_info.size;

    return ret;
}

/*****************************************************************************
 * Map device BAR2 to data buffer
 ****************************************************************************/
int mapBAR2(int deviceFD, uint64_t *barSize)
{
    int ret;

    struct vfio_region_info  bar2_region_info = {.argsz = sizeof(bar2_region_info)};

    bar2_region_info.index = VFIO_PCI_BAR2_REGION_INDEX;

    ret = ioctl(deviceFD, VFIO_DEVICE_GET_REGION_INFO, &bar2_region_info);

    if (ret)
    {
        printf("Failed to get info\n");
        return -1;
    }

    printf("BAR2 Info: size 0x%lx, offset 0x%lx, flags 0x%x\n",
                            (unsigned long)bar2_region_info.size,
                            (unsigned long)bar2_region_info.offset,
                                           bar2_region_info.flags);

    bar2_data = (uint32_t *) mmap(NULL, (size_t)bar2_region_info.size, PROT_READ | PROT_WRITE, MAP_SHARED, deviceFD, (off_t)bar2_region_info.offset);

    if (bar2_data == MAP_FAILED)
    {
        printf("mmap failed\n");
        return -1;
    }

    *barSize = bar2_region_info.size;

    return ret;
}

/*****************************************************************************
 * Initialize MSI IRQ
 ****************************************************************************/
int initMsiIrq(int deviceFD, int *intx, int numMsiIrqs)
{
    int ret;
    int32_t *pfd;
    int i;

    struct vfio_irq_info  msi_irq_info = {.argsz = sizeof(msi_irq_info),};
    struct vfio_irq_set  *irq_set;

    msi_irq_info.index = VFIO_PCI_MSI_IRQ_INDEX;

    ret = ioctl(deviceFD, VFIO_DEVICE_GET_IRQ_INFO, &msi_irq_info);

    if (ret)
    {
        printf("Failed to get MSI IRQ info\n");
        return -1;
    }

    printf("MSI IRQ Info: index: %d, count: %d, flags: %d\n", msi_irq_info.index, msi_irq_info.count, msi_irq_info.flags);

    if (msi_irq_info.count > PCIE_MSI_IRQ_MAX || !(msi_irq_info.flags & VFIO_IRQ_INFO_EVENTFD))
    {
        printf("Unexpected IRQ info properties\n");
        return -1;
    }

    if (numMsiIrqs > msi_irq_info.count)
    {
        printf("EP does not request %d MSI IRQ vectors\n", numMsiIrqs);
        printf("The maximum number of distinct MSI IRQ vectors requested by the EP is %d\n", msi_irq_info.count);
        return -1;
    }

    irq_set = malloc(sizeof(*irq_set) + sizeof(*pfd) * numMsiIrqs);

    if (!irq_set)
    {
        printf("Failed to malloc irq_set\n");
        return -1;
    }

    irq_set->argsz = sizeof(*irq_set) + sizeof(*pfd) * numMsiIrqs;
    irq_set->count = numMsiIrqs;
    irq_set->index = VFIO_PCI_MSI_IRQ_INDEX;
    irq_set->start = 0;

    pfd = (int32_t *)&irq_set->data;

    for (i = 0; i < numMsiIrqs; i++)
    {
        pfd[i] = eventfd(0, EFD_CLOEXEC);

        if (pfd[i] < 0)
        {
            printf("Failed to get intx eventfd\n");
            return -1;
        }

        intx[i] = pfd[i];
    }

    irq_set->flags = VFIO_IRQ_SET_DATA_EVENTFD | VFIO_IRQ_SET_ACTION_TRIGGER;

    ret = ioctl(deviceFD, VFIO_DEVICE_SET_IRQS, irq_set);

    if (ret)
    {
        printf("Failed to set MSI IRQ\n");
    }

    return ret;
}

/*****************************************************************************
 * Send Downstream IRQ to EP
 ****************************************************************************/
int sendDwnIrq(int deviceFD, uint64_t config_region_offset)
{
    int ret;
    off_t    offset    = PCIE_EP_VNDR_SPC_CNTRL_REG_OFFSET;
    uint32_t buf_write = PCIE_EP_VNDR_SPC_CNTRL_REG_HTI_SET;
    size_t   len_write = sizeof(buf_write);

    /* in orer to trigger a downstream IRQ in the EP the host needs to
       write the HTI bit in the EP's vendor sepcific control register */
    ret = pwrite(deviceFD, &buf_write, len_write, (off_t) config_region_offset + offset);

    if (ret == -1)
    {
        printf("error on writing config space\n");
        perror("pwrite64");
        return -1;
    }

    return 0;
}

/*****************************************************************************
 * Compare sent and received application data
 ****************************************************************************/
int checkData(void *vaddr, uint64_t len, uint32_t misIrqItrNum)
{
    int ret = 0;
    uint32_t *buf = (uint32_t *) vaddr;

    while (len)
    {
        int j;
        for (j = 0; (j < len / sizeof (uint32_t)) && (j < SMPL_BAR0_DATA_SIZE / sizeof (uint32_t)); j++)
        {
            if (buf[j] != ((misIrqItrNum << 28) | (uint32_t) j) )
                return -1;
        }

        len -= j * 4;
        buf += j;
    }

    return ret;
}

/*****************************************************************************
 * Apply Test on distinct End Point MSI IRQs
 ****************************************************************************/
int testMSI(int deviceFD, uint64_t config_region_offset, int num_msi_irq, int *intx)
{
    int i;
    int ret;
    uint64_t buf_read;

    /* test multiple MSI vectors */
    printf("Initialize MSI test. Expect %d distinct MSI IRQs\n", num_msi_irq);

    bar0_mem->config.ctrl = SMPL_CTRL_TEST_MSI;

    wait_user();

    /* send Downstream IRQ */
    ret = sendDwnIrq(deviceFD, config_region_offset);

    wait_user();

    for (i = 0; i < num_msi_irq; i++)
    {
        printf("Expect MSI IRQ nr. %d\n", i);

        ret = read(intx[i], &buf_read, sizeof(buf_read));

        if (ret == -1)
        {
            printf("MSI test failed\n");
            break;
        }
    }

    return ret;
}

/*****************************************************************************
 * Apply Test on default PCIe End Point BARs 1 & 2
 ****************************************************************************/
int testBARs(int deviceFD, uint64_t config_region_offset, int *intx)
{
    int ret;
    int i;
    uint64_t buf_read;
    uint64_t bar1Size;
    uint64_t bar2Size;

    /* test BARs */
    printf("Initialize BARs test\n");

    ret = mapBAR1(deviceFD, &bar1Size);
    ret = mapBAR2(deviceFD, &bar2Size);

    wait_user();

    /* copy data to bar1 */
    for (i = 0; i < bar1Size / sizeof (uint32_t); i++)
    {
        bar1_data[i] = i;
    }

    /* copy data to bar2 */
    for (i = 0; i < bar2Size / sizeof (uint32_t); i++)
    {
        bar2_data[i] = ~i;
    }

    bar0_mem->config.ctrl = SMPL_CTRL_TEST_BARs;

    /* send Downstream IRQ */
    ret = sendDwnIrq(deviceFD, config_region_offset);

    /* wait for EP MSI IRQ*/
    ret = read(intx[0], &buf_read, sizeof(buf_read));

    if (ret == -1)
    {
        printf("Error on wating for EP MSI IRQ\n");
        printf("MSI IRQ read Buffer value: 0x%x\n", buf_read);
        perror("read");
        printf("BAR tests failed\n");
    }

    return ret;
}

/*****************************************************************************
 * Apply Test on extended PCIe End Point BAR configuration
 ****************************************************************************/
int testExtendBARs(int deviceFD)
{
    int ret;

    struct vfio_region_info  bar0_region_info = {.argsz = sizeof(bar0_region_info)};
    struct vfio_region_info  bar1_region_info = {.argsz = sizeof(bar1_region_info)};
    struct vfio_region_info  bar2_region_info = {.argsz = sizeof(bar2_region_info)};
    struct vfio_region_info  bar3_region_info = {.argsz = sizeof(bar3_region_info)};
    struct vfio_region_info  bar4_region_info = {.argsz = sizeof(bar4_region_info)};
    struct vfio_region_info  bar5_region_info = {.argsz = sizeof(bar5_region_info)};

    bar0_region_info.index = VFIO_PCI_BAR0_REGION_INDEX;
    bar1_region_info.index = VFIO_PCI_BAR1_REGION_INDEX;
    bar2_region_info.index = VFIO_PCI_BAR2_REGION_INDEX;
    bar3_region_info.index = VFIO_PCI_BAR3_REGION_INDEX;
    bar4_region_info.index = VFIO_PCI_BAR4_REGION_INDEX;
    bar5_region_info.index = VFIO_PCI_BAR5_REGION_INDEX;

    /* get region info of BAR0 */
    ret = ioctl(deviceFD, VFIO_DEVICE_GET_REGION_INFO, &bar0_region_info);

    wait_user();

    if (ret)
    {
        printf("Failed to get info\n");
        return -1;
    }

    printf("BAR0 Info: size 0x%lx, offset 0x%lx, flags 0x%x\n",
                            (unsigned long)bar0_region_info.size,
                            (unsigned long)bar0_region_info.offset,
                                           bar0_region_info.flags);

    /* get region info of BAR1 */
    ret = ioctl(deviceFD, VFIO_DEVICE_GET_REGION_INFO, &bar1_region_info);

    if (ret)
    {
        printf("Failed to get info\n");
        return -1;
    }

    printf("BAR1 Info: size 0x%lx, offset 0x%lx, flags 0x%x\n",
                            (unsigned long)bar1_region_info.size,
                            (unsigned long)bar1_region_info.offset,
                                           bar1_region_info.flags);

    /* get region info of BAR2 */
    ret = ioctl(deviceFD, VFIO_DEVICE_GET_REGION_INFO, &bar2_region_info);

    if (ret)
    {
        printf("Failed to get info\n");
        return -1;
    }

    printf("BAR2 Info: size 0x%lx, offset 0x%lx, flags 0x%x\n",
                            (unsigned long)bar2_region_info.size,
                            (unsigned long)bar2_region_info.offset,
                                           bar2_region_info.flags);

    /* get region info of BAR3 */
    ret = ioctl(deviceFD, VFIO_DEVICE_GET_REGION_INFO, &bar3_region_info);

    if (ret)
    {
        printf("Failed to get info\n");
        return -1;
    }

    printf("BAR3 Info: size 0x%lx, offset 0x%lx, flags 0x%x\n",
                            (unsigned long)bar3_region_info.size,
                            (unsigned long)bar3_region_info.offset,
                                           bar3_region_info.flags);

    /* get region info of BAR4 */
    ret = ioctl(deviceFD, VFIO_DEVICE_GET_REGION_INFO, &bar4_region_info);

    if (ret)
    {
        printf("Failed to get info\n");
        return -1;
    }

    printf("BAR4 Info: size 0x%lx, offset 0x%lx, flags 0x%x\n",
                            (unsigned long)bar4_region_info.size,
                            (unsigned long)bar4_region_info.offset,
                                           bar4_region_info.flags);

    /* get region info of BAR5 */
    ret = ioctl(deviceFD, VFIO_DEVICE_GET_REGION_INFO, &bar5_region_info);

    if (ret)
    {
        printf("Failed to get info\n");
        return -1;
    }

    printf("BAR5 Info: size 0x%lx, offset 0x%lx, flags 0x%x\n",
                            (unsigned long)bar5_region_info.size,
                            (unsigned long)bar5_region_info.offset,
                                           bar5_region_info.flags);

    return ret;
}

/*****************************************************************************
 * Main
 ****************************************************************************/
int main(int argc, char *argv[])
{
    int ret;
    int container;
    int deviceFD;
    int intx[PCIE_MSI_IRQ_MAX];
    void *dma_vaddr;
    uint32_t dma_len;
    uint64_t dma_iova;
    uint64_t buf_read;
    uint64_t config_region_offset;
    int bus, dev, func, group;
    int num_iter;
    int num_msi_irq;
    int i;
    enum test_mode test_mode;

    if (argc < 5 || argc > 8)
    {
        /* ti-sample-vfio <bus> <dev> <func> <iommu-group> [num msi irqs] [num loops] [*/
        printf("usage: %s <bus> <dev> <func> <iommu-group> ['testbars'] [num msi irqs] [num loops] ['wait']\n", argv[0]);
        return EXIT_FAILURE;
    }

    bus = strtol(argv[1], NULL, 0);
    dev = strtol(argv[2], NULL, 0);
    func = strtol(argv[3], NULL, 0);
    group = strtol(argv[4], NULL, 0);

    if (argc > 5)
    {
        if (strcmp(argv[5], "testbars") == 0)
        {
            test_mode = TEST_MODE_BARS;

            if (argc >= 7)
                if (strcmp(argv[6], "wait") == 0)
                    wait_user_enabled = 1;
        }
        else
        {
            test_mode = TEST_MODE_DEFAULT;
            num_msi_irq = strtol(argv[5], NULL, 0);

            if (argc >= 7)
                num_iter = strtol(argv[6], NULL, 0);
            else
                num_iter = 10;

            if (argc >= 8)
                if (strcmp(argv[7], "wait") == 0)
                    wait_user_enabled = 1;
        }
    }
    else
    {
        num_msi_irq = 1;
        num_iter = 10;
    }

    if (num_msi_irq > PCIE_MSI_IRQ_MAX)
    {
        printf("Assignment for number of MSI IRQ vectors failed. The maximum number is %d\n", PCIE_MSI_IRQ_MAX);
        return EXIT_FAILURE;
    }

    printf("--------------------------------------------------------------------------------\n");

    printf("Starting PCIe RC VFIO test application with parameters:\n"
            "PCIe Bus Number: %d\n"
            "PCIe Device Number: %d\n"
            "PCIe Function Number: %d\n"
            "PCIe IOMMU Number: %d\n",
             bus, dev, func, group);

    if (test_mode == TEST_MODE_BARS)
    {
        printf("Test mode: extended BAR test\n");
    }
    else if (test_mode == TEST_MODE_DEFAULT)
    {
        printf("Test mode: default test\n"
               "MSI IRQ number: %d\n"
               "Iteration number: %d\n",
               num_msi_irq, num_iter);
    }

    printf("--------------------------------------------------------------------------------\n");

    ret = initVFIO(bus, dev, func, group,
                   &container, &deviceFD, &config_region_offset);

    if (ret)
    {
        printf("VFIO initialization failed\n");
        return 0;
    }

    wait_user();

    /* extended bar testing and "default" test mode are mutually exclusive */
    if (test_mode == TEST_MODE_BARS)
    {
        ret = testExtendBARs(deviceFD);

        /* when testing BARs we're done */
        if (ret)
        {
            printf("BAR test failed\n");
            return EXIT_FAILURE;
        }

        printf("Extended BAR test passed\n");

        wait_user();

        return EXIT_SUCCESS;
    }

    ret = mapDMA(container, &dma_iova, &dma_len, &dma_vaddr);

    if (ret)
    {
        printf("DMA map failed\n");
        return 0;
    }

    ret = mapBAR0(deviceFD);

    if (ret)
    {
        printf("BAR0 map failed\n");
        return 0;
    }

    /* tell EP which dma address & size to use */
    bar0_mem->config.dma_addr = dma_iova;
    bar0_mem->config.dma_len  = dma_len;

    ret = initMsiIrq(deviceFD, intx, num_msi_irq);

    if (ret)
    {
        printf("MSI IRQ initialization failed\n");
        return 0;
    }

    printf("RC completed EP initialization\n");

    printf("--------------------------------------------------------------------------------\n");

    wait_user();

    printf("Start COPY test\n");

    /* signal EP that initialization is finished */
    bar0_mem->config.ctrl = SMPL_CTRL_INIT_DONE;

    /* send downstream IRQ */
    ret = sendDwnIrq(deviceFD, config_region_offset);

    if (ret)
        printf("Error on sending downstream IRQ\n");

    for(int i = 0; i < num_iter; i++)
    {
        /* copy data to bar0 */
        for (int j = 0; j < SMPL_BAR0_DATA_SIZE / sizeof (uint32_t); j++)
        {
            /* save current MSI IRQ iteration number to last four bit
               of data and fill rest of data with current index */
            bar0_mem->data[j] = ( (uint32_t) i << 28 ) | (uint32_t) j;
        }

        /* request copy, send Downstream IRQ */
        bar0_mem->config.ctrl |= SMPL_CTRL_COPY;
        ret = sendDwnIrq(deviceFD, config_region_offset);

        if (ret)
            printf("Error on sending DWN IRQ\n");

        /* wait for EP MSI IRQ*/
        ret = read(intx[0], &buf_read, sizeof(buf_read));

        if (ret == -1)
        {
            printf("Error on wating for EP MSI IRQ\n");
            printf("MSI IRQ read buffer value: 0x%x\n", buf_read);
            perror("read");
        }

        /* clear copy request */
        bar0_mem->config.ctrl &= ~SMPL_CTRL_COPY;

        /* verify data in DMA buffer */
        ret = checkData(dma_vaddr, dma_len, i);

        if (ret)
        {
            printf("Application data verification failed\n");
	         wait_user();
        }
    }

    printf("COPY test passed with %d loops\n", num_iter);

    printf("--------------------------------------------------------------------------------\n");

    ret = testMSI(deviceFD, config_region_offset, num_msi_irq, intx);

    if (ret != -1)
        printf("MSI test passed\n");

    printf("--------------------------------------------------------------------------------\n");

    ret = testBARs(deviceFD, config_region_offset, intx);

    if (ret != -1)
        printf("BAR test passed\n");

    printf("--------------------------------------------------------------------------------\n");

    printf("RC resets EP\n");

    /* now signal application reset to EP */
    bar0_mem->config.ctrl = SMPL_CTRL_RESET;

    /* send Downstream IRQ */
    ret = sendDwnIrq(deviceFD, config_region_offset);

    if (ret)
        printf("Error on sending downstream IRQ\n");

    return 0;
}

