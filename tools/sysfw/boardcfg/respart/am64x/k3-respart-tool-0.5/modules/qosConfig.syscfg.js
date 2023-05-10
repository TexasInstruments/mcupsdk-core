const utils = system.getScript("/scripts/utils.js");

const deviceSelected = utils.socName;
const devData = utils.devData;
const socName = utils.socName;
const qos = utils.qos;

var properties = ["atype", "virtId", "orderId", "qos", "epriority", "asel"];

function getDeviceNameOptions() {
	var deviceNames = [];
	_.each(qos, (d) => {
		deviceNames.push(d.deviceName);
	});
	deviceNames = _.uniq(deviceNames);

	var deviceOpt = _.map(deviceNames, (d) => {
		return {
			name: d,
			displayName: d,
		};
	});
	deviceOpt.unshift({
		name: "unknown",
		displayName: "Select",
	});

	return deviceOpt;
}

var deviceEndPoints = _.groupBy(qos, (d) => {
	return d.deviceName;
});

function defaultOptionValues(max) {
	var options = [];

	for (var i = 0; max < 10 && i < max; i++) {
		options.push(i);
	}
	return options;
}

function optionValues(max, stringifyName = false) {
	var options = [];
	var val;

	for (var i = 0; i < max; i++) {
		if (stringifyName)
			val = i.toString();
		else
			val = i;
		options.push({
			name: val,
			displayName: i.toString(),
		});
	}
	return options;
}

function showValidParameters(inst, ui) {
	var maxINT = 100000;
	var parametersCount = {
		channels: maxINT,
	};

	if (inst.deviceName === "unknown")
		return;

	_.each(properties, (p) => {
		parametersCount[p] = 0;
	});

	_.each(inst.qosdev, (e) => {
		var endPointName = inst.deviceName + "_" + e;

		_.each(properties, (p) => {
			parametersCount[p] |= qos[endPointName][p];
		});
		parametersCount.channels = Math.min(parametersCount.channels, qos[endPointName].channelCount);
	});
	inst.numChan = parametersCount.channels === maxINT ? 0 : parametersCount.channels;
	ui.chan.hidden = false;

	_.each(properties, (p) => {
		ui[p].hidden = !parametersCount[p];
	});
}

var documentation = `
**QoS configuration**

---

This module allows you to set various Qoality of Service (QoS) parameters
for different devices in the SoC. QoS for most of the devices is programmed
at the interconnect QoS endpoints. QoS parameters for devices using UDMA
are programmed dynamically. Here you will only find devices which has CBASS
interconnect QoS endpoints.

Following steps allow you to achieve this:

*	Click on ADD button to program a device QoS endpoint
*	Select the device from the drop down list
*	Select a list of endpoints to be programmed. By default, all the
endpoints are selected. You can add more instances with same device
and select different set of endpoints to program the QoS differently.
*	Select the channel number for programming the exact QoS register.
*	For the selected channel, tool will present a set of parameters which
are applicable for it. Select the appropriate value that you wish
to program.

**Endpoint and Channel ID selection**

Every device may have multiple DMA ports to the interconnect. There is a
dedicated QoS endpoint per DMA port. Also, many devices differentiate the
type of DMA traffic by driving the channel_id value. e.g. DSS traffic for
VID1 pipeline and VIDL1 pipeline wil carry a different channel ID and the
QoS settings corresponding to this channel ID will be used. Note that, how
the channel_id is driven for a device is different for different device.

**Output files**

---

*	\`qos-config.c\` - This file should be copied to the bootloader for
configuring the QoS registers. This file has a simple address, value
pairs in an array. The bootloader is supposed to iterate over the
array and program the QoS registers one-time as part of the bootup.

`;

exports = {
	displayName: "Quality of Service",
	longDescription: documentation,
	config: [
		{
			name: "deviceName",
			displayName: "Device Name",
			options: getDeviceNameOptions(),
			default: "unknown",
			onChange: (inst, ui) => {
				if (inst.deviceName === "unknown") return;

				// Make all endpoints visible and select all by default

				ui.qosdev.hidden = false;
				inst.qosdev = _.map(deviceEndPoints[inst.deviceName], (d) => {
					return d.name;
				});

				showValidParameters(inst, ui);
				inst.chan = defaultOptionValues(inst.numChan);
			},
		},
		{
			name: "qosdev",
			displayName: "QoS endpoint",
			hidden: true,
			default: ["none"],
			options: (inst) => {
				var endPoints = _.map(deviceEndPoints[inst.deviceName], (d) => {
					return {
						name: d.name,
						displayName: d.name,
					};
				});

				return endPoints;
			},
			onChange: (inst, ui) => {
				showValidParameters(inst, ui);
			},
		},
		{
			name: "numChan",
			displayName: "Available number of channels",
			hidden: true,
			default: 0,
		},
		{
			name: "chan",
			displayName: "Channel ID",
			default: [],
			hidden: true,
			options: (inst) => {
				return optionValues(inst.numChan, true);
			},
		},
		{
			name: "atype",
			displayName: "Address type",
			default: 0,
			hidden: true,
			options: [
				{
					name: 0,
					displayName: "Physical address",
					description: "All transactions are routed directly to the target",
				},
				{
					name: 1,
					displayName: "Intermediate physical address",
					description: "All transactions are routed via PVU",
				},
				{
					name: 2,
					displayName: "Virtual address",
					description: "All transactions are routed via sMMU",
				},
				{
					name: 3,
					displayName: "Non coherent address",
					description: "Memory coherency is disabled for these transactions",
				},
			],
		},
		{
			name: "virtId",
			displayName: "virt_id",
			hidden: true,
			default: 0,
			options: optionValues(16),
		},
		{
			name: "orderId",
			displayName: "order_id",
			hidden: true,
			default: 0,
			options: optionValues(16),
		},
		{
			name: "asel",
			displayName: "Address selector",
			hidden: true,
			default: 0,
			options: optionValues(4),
		},
		{
			name: "qos",
			displayName: "Quality of Service",
			hidden: true,
			default: 0,
			options: optionValues(8),
		},
		{
			name: "epriority",
			displayName: "Escalated priority",
			hidden: true,
			default: 0,
			options: optionValues(8),
		},
	],
	validate: (inst, report) => {
		if (inst.deviceName == "unknown") {
			report.logError("Select a device from the list", inst, "deviceName");
		}

		showParameterInfo(inst, report);
		check_overlapped_ep_ch(inst, report);
	},
};

// functions for validation

function check_overlapped_ep_ch(inst, report) {
	var moduleInstance = inst.$module.$instances;

	_.each(moduleInstance, (i) => {
		if (i !== inst) {
			if (i.deviceName === inst.deviceName) {
				var common_eps = _.intersection(i.qosdev, inst.qosdev);
				var common_chans = _.intersection(i.chan, inst.chan);
				if (common_eps.length && common_eps[0] !== "none" &&
				    common_chans.length && common_chans[0] != "none") {
					report.logError("This endpoint(" + common_eps[0] +
					") + channel(" + common_chans[0] +
					") is used more than once", inst, "qosdev");
				}
			}
		}
	});
}

function showParameterInfo(inst, report) {
	if (inst.deviceName === "unknown") return;

	_.each(properties, (p) => {
		if (!inst[p].hidden) {
			var names = "";
			_.each(inst.qosdev, (e) => {
				var endPointName = inst.deviceName + "_" + e;
				if (!qos[endPointName][p]) {
					names += e;
					names += ", ";
				}
			});
			if (names.length) {
				report.logInfo("This parameter is not available for " + names, inst, p);
			}
		}
	});
}
