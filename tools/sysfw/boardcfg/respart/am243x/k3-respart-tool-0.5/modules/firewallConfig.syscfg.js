const utils = system.getScript("/scripts/utils.js");
const deviceSelected = utils.deviceSelected;
const devData = utils.devData;
const socName = utils.socName;
var devices = utils.FirewallDevices;

var start = "0";
var end = "0";
var regions = 1;
var customAlloc = false;

function getDeviceNameOptions() {
	var uniqDevices = _.map(devices, (d) => {
		return d.name;
	});

	var devOpt = _.map(uniqDevices, (d) => {
		return {
			name: d,
			displayName: d,
		};
	});

	devOpt.unshift({
		name: "unknown",
		displayName: "Select",
	});

	return devOpt;
}

var documentation = `
**Firewall configuration**

---

This module allows to program the firewalls for peripheral partitioning across
different CPU cores. A firewall checks if the given transaction is allowed or
not based on few parameters like priv_id, transaction type (read/write, etc),
security and privilege levels. It is possible to program firewalls such that
only certain CPUs can access the device and accesses from any other CPUs are
ignored. Thus ensuring peripheral partitioning.

Following steps allow to do this:

*	Click on ADD button and add an instance for firewall configuration.
*	Select a device to be firewalled or protected.
*	For slave firewalls, the tool automatically fills up a firewall
region with start and end addresses to cover all the slave interfaces
of the device.
*	For every region, upto 3 permission slots can be added. A permission
slot defines access permissions for a host_id. For simplicity, you
can select a host_id for automatically filling the priv_id or provide
a custom priv_id in the permission slot.
*	By default, for any slot, all permissions are enabled. Modify this as
required.


**Output files**

---

*	\`firewall-config.c\` -	This file describes the firewall configuration
for a bootloader. It contains an array of the struct
*TISCI_MSG_FWL_SET_REGION* which can be directly passed in TISCI calls.
Bootloader should simply iterate over this to program all the firewalls.

`;

exports = {
	displayName: "Firewall Configuration",
	longDescription: documentation,
	config: [
		{
			name: "device",
			displayName: "Device to be protected",
			options: getDeviceNameOptions(),
			default: "unknown",
			onChange: (inst, ui) => {
				(start = devices[inst.device].start_address), (end = devices[inst.device].end_address);

				if (devices[inst.device].memory) {
					regions = devices[inst.device].num_regions;
					customAlloc = true;
				} else {
					regions = 1;
					customAlloc = false;
				}
				var instances = devices[inst.device].protected_inst;
				instances = _.uniq(instances);

				inst.instanceName = _.map(instances, (i) => {
					return i;
				});
			},
		},
		{
			name: "instanceName",
			displayName: "Instance Name",
			default: ["none"],
			options: (inst) => {
				if (!devices[inst.device]) {
					return [];
				}
				var instances = devices[inst.device].protected_inst;
				instances = _.uniq(instances);
				return _.map(instances, (i) => {
					return {
						name: i,
						displayName: i,
					};
				});
			},
		},
	],
	moduleInstances: (inst) => {
		return [
			{
				name: "regions",
				displayName: "Firewall Regions",
				moduleName: "/modules/firewallRegion",
				minInstanceCount: 1,
				maxInstanceCount: regions,
				useArray: true,
				collapsed: false,
				args: {
					defaultStart: start,
					defaultEnd: end,
					regionAlloc: customAlloc,
					memory: customAlloc,
				},
			},
		];
	},
	validate: (inst, report) => {
		if (inst.device === "unknown") {
			report.logError("Select a Device", inst, "device");
		}
	},
};
