"use strict";

let common = system.getScript("/common");
let device = common.getDeviceName();

function enet_pkt_pool_validate(instance, report) {
    if (instance.PktPoolEnable === true)
    {
        let largePoolPktSize  = (instance.LargePoolPktCount > 0)   ? instance.LargePoolPktSize  : 0;
        let mediumPoolPktSize = (instance.MediumPoolPktCount > 0)  ? instance.MediumPoolPktSize : 0;
        let smallPoolPktSize  = (instance.SmallPoolPktCount > 0)   ? instance.SmallPoolPktSize  : 0;
        let totalPktPoolCount = instance.LargePoolPktCount + instance.MediumPoolPktCount + instance.SmallPoolPktCount;
        let totalDmaChannelPkts;
        if (instance.PktInfoOnlyEnable === true)
        {
            totalDmaChannelPkts = instance.$module.getRxPacketsCount(instance);
        }
        else
        {
            totalDmaChannelPkts = instance.$module.getTxPacketsCount(instance) + instance.$module.getRxPacketsCount(instance);
        }


        if (instance.LargePoolPktCount > 0)
        {
            if ((largePoolPktSize <= mediumPoolPktSize) || (largePoolPktSize <= smallPoolPktSize))
            {
                report.logError(`Large pool size must be greater than Medium and Small pool size`, instance, "LargePoolPktSize");
            }
        }
        if (instance.MediumPoolPktCount > 0)
        {
            if (mediumPoolPktSize <= smallPoolPktSize)
            {
                report.logError(`Medium pool size must be greater than Small pool size`, instance, "MediumPoolPktSize");
            }
        }
        if (totalPktPoolCount < totalDmaChannelPkts)
        {
            report.logError(`Number packets in pool ${totalPktPoolCount} does not match sum of tx and rx channel packet count ${totalDmaChannelPkts}`, instance, ["LargePoolPktCount", "MediumPoolPktCount","SmallPoolPktCount"]);
        }

    }
}

const enet_pkt_pool_config = {
    name: "enetPktPoolConfig",
    displayName: "Packet Pool Config",
	longDescription: "Configuration of packet pool",
    config: [
        {
            name: "PktPoolEnable",
            description: "Flag to enable packet allocation from enet utils library. It should be disabled to avoid utils memory wastage, in case aplication allots packet via other mechanism. (Ex- Lwip pools)",
            displayName: "Enable Packet Pool Allocation",
            default: true,
            onChange: function (inst, ui) {
                /* Init delay applicable only for single master mode */
                if(inst.PktPoolEnable == false) {
                    ui.LargePoolPktSize.hidden = true;
                    ui.LargePoolPktCount.hidden = true;
                    ui.MediumPoolPktSize.hidden = true;
                    ui.MediumPoolPktCount.hidden = true;
                    ui.SmallPoolPktSize.hidden = true;
                    ui.SmallPoolPktCount.hidden = true;
                }
                else {
                    ui.LargePoolPktSize.hidden = false;
                    ui.LargePoolPktCount.hidden = false;
                    ui.MediumPoolPktSize.hidden = false;
                    ui.MediumPoolPktCount.hidden = false;
                    ui.SmallPoolPktSize.hidden = false;
                    ui.SmallPoolPktCount.hidden = false;
                }
            },
        },
        {
            name: "LargePoolPktSize",
            displayName: "Large Pool Packet Size",
            default: 1536,
            isInteger: true,
            range: [128, 1536]

        },
        {
            name: "LargePoolPktCount",
            displayName: "Large Pool Packet Count",
            default: 48,
            isInteger: true,
            range: [0, 192]

        },
        {
            name: "MediumPoolPktSize",
            displayName: "Medium Pool Packet Size",
            default: 512,
            isInteger: true,
            range: [128, 512]
        },
        {
            name: "MediumPoolPktCount",
            displayName: "Medium Pool Packet Count",
            default: 0,
            isInteger: true,
            range: [0, 192]
        },
        {
            name: "SmallPoolPktSize",
            displayName: "Small Pool Packet Size",
            default: 128,
            isInteger: true,
            range: [128, 512]

        },
        {
            name: "SmallPoolPktCount",
            displayName: "Small Pool Packet Count",
            default: 0,
            isInteger: true,
            range: [0, 192]
        },
        {
            name: "PktInfoOnlyEnable",
            description: "Flag to allocate only the DMA Packet Info structures, this does not include the buffer memory. This is useful when the buffer memory is internally allocated by the application. (Ex- Lwip pools)",
            displayName: "Only Enable Packet Info Allocation",
            default: false,
            onChange: function (inst, ui) {
                /* Init delay applicable only for single master mode */
                if(inst.PktInfoOnlyEnable == true) {
                    ui.PktInfoOnlyCount.hidden = false;
                }
                else {
                    ui.PktInfoOnlyCount.hidden = true;
                }
            },
        },
        {
            name: "PktInfoOnlyCount",
            displayName: "PktInfoMem Only Count",
            description: "DMA Pkt Info structures are only allocated, the buffer memory is not allocated here.",
            default: 16,
            hidden: true,
            isInteger: true,
            range: [8, 192]
        },
    ],
    collapsed:true,
};

exports = {
    config: enet_pkt_pool_config,
    validate: enet_pkt_pool_validate,
};
