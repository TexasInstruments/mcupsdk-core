var utils = system.getScript("/scripts/utils.js");
const { checkOverlap, checkInvalidHwaRange, resourceAllocate } = system.getScript("/scripts/allocation.js");

const deviceSelected = utils.deviceSelected;
const devData = utils.devData;
const socName = utils.socName;
const resources = utils.resources;
var hosts = utils.hosts;
var resGroup = utils.resourcesByGroup;
var groupNames = utils.groupNames;
var hostNames =  utils.hostNames;
var moduleInstances = [];

_.each(resources, (resource) => {
	resource.copyToUtype = [];
	resource.blockCopyTo = [];
});

_.each(resources, (resource) => {
	if (resource.copyFromUtype) {
		resources[resource.copyFromUtype].copyToUtype.push(resource.utype);
	}
	if (resource.blockCopyFrom) {
		resources[resource.blockCopyFrom].blockCopyTo.push(resource.utype);
	}
});

var documentation = `
**SYSFW Resource Partitioning**

---

This module allows to assign resources managed by System firmware
for the selected host. Each host is a dedicated context for SYSFW.
Typically, resources related to DMA channels, rings, proxies, and
interrupts are managed by SYSFW.

Following steps allow you to achieve this:

*	Click on ADD button to add a new host in the tool
*	Note the CPU core and security level assosiated with selected host
*	Optionally, you can configure some capabilities of the host
*	For each host, a list of resources is presented in various groups.
You can specify the count of certain resource.

**Resource allocation**

Most of the resources are homogeneous. It should not matter as to which specific
range of resource has been allocated to the software. Therefore, the tool only
asks for the count of the resources required for each host, and then does
allocation of the exact range taking into consideration some constraints.

However, there are some resources (e.g. virtid) where user can choose the
start index and count specifically.

Note that after allocating resources for all the hosts, remaining count is
assigned to *HOST_ID_ALL*.

**Host capabilities**

Most of the parameters here are needed for some specialized use cases.
You should modify these only if you knwo what you are doing.

*	**Allowed values of atype** - SYSFW will ensure that a host can program
a certain value of atype (address type) only if it is enabled in this
list. There are few APIs (e.g. UDMA config) which accept atype
as part of the TISCI API. This is useful for hosts running inside
virtual machine where all the DMA accesses should must through an IOMMU.
*	**Allowed values of QoS, orderid, priority, schedpriority** - SYSFW will
ensure that the host can program a certain value of Quality of Service
parameters only if it is enabled in this list. This is useful to ensure
that a host cannot program higher QoS values for UDMA potentially
causing performance degradation to other masters.
*	**Supervisor host** - This parameter allows to declare another host
as the supervisor of the current host. Supervisor can claim all the
resources that his subordinate owns. This is useful for FFI recovery
scenario where a safety software would try to restart nonsafe software.

**Output files**

---

*	\`rm-cfg.c\` - This is the format used for k3-image-gen
RM board config files
*	\`sciclient_defaultBoardcfg_rm.c\` - This is the format used for Secondary
Boot Loader (SBL) RM board config files.

**Allocation visualization**

---

The tool shows the current allocation of each resource for each host in a table.
Click on the three dots in the top right corner to open the *Resource Allocation Table*
pane to view this table. This table gets updated automatically whenever some
allocation is changed. This is especially useful when trying to solve errors
due to overflow in allocation.

`;

function getHostConfigurables(hostName) {
	// Create configurable for each resource

	var configurables = [];

	_.each(groupNames, (gName) => {
		var groupResources = resGroup[gName];

		var def = {
			name: _.join(_.split(gName, " "), "_"),
			displayName: gName,
			config: [],
			collapsed: false,
		};

		_.each(groupResources, (r) => {
			var canBeRouted = checkRestrictHost(hostName, r);
			var displayPrefix = utils.getDisplayPrefix(gName, r.utype);
			def.config.push({
				name: _.join(_.split(r.utype, " "), "_") + "_start",
				displayName: r.extended ? displayPrefix + " DRU Start" : displayPrefix + " Start",
				default: 0,
				hidden: r.autoAlloc === false && !r.copyFromUtype ? false : true,
				onChange: (inst, ui) => {
					update_dependencies(inst, r);
				},
			});

			def.config.push({
				name: _.join(_.split(r.utype, " "), "_") + "_count",
				displayName: r.extended ? displayPrefix + " DRU Count" : displayPrefix + " Count",
				default: 0,
				readOnly: r.copyFromUtype ? true : false,
				hidden: !canBeRouted || r.copyFromUtype ? true : false,
				description: r.copyFromUtype
					? "Count of this resource is automatically matched with resource " + r.copyFromUtype
					: "",
				onChange: (inst, ui) => {
					update_dependencies(inst, r);
				},
			});

			if (r.blockCopy) {
				var blockCopyDisplayName = utils.getBlockCopyDisplayName(displayPrefix);
				def.config.push({
					name: _.join(_.split(r.utype, " "), "_") + "_blockCount",
					displayName: blockCopyDisplayName + "Block-copy count",
					default: 0,
					readOnly: r.copyFromUtype ? true : false,
					hidden: r.blockCopyFrom || r.copyFromUtype ? true : false,
					description: r.copyFromUtype
						? "Block Count of this resource is automatically matched with resource " + r.copyFromUtype
						: "",
					onChange: (inst, ui) => {
						update_dependencies(inst, r);
					},
				});
			}

			if (r.extended) {
				def.config.push({
					name: _.join(_.split(r.utype, " "), "_") + "_hwa_start",
					displayName: displayPrefix + " HWA Start",
					default: 0,
					hidden: !r.copyFromUtype ? false : true,
					readOnly: r.copyFromUtype ? true : false,
					description: r.copyFromUtype
						? "HWA Start of this resource is automatically matched with resource " + r.copyFromUtype
						: "",
					onChange: (inst, ui) => {
						update_dependencies(inst, r);
					},
				});
				
				def.config.push({
					name: _.join(_.split(r.utype, " "), "_") + "_hwa_count",
					displayName: displayPrefix + " HWA count",
					default: 0,
					readOnly: r.copyFromUtype ? true : false,
					hidden: !canBeRouted || r.copyFromUtype ? true : false,
					description: r.copyFromUtype
						? "HWA Count of this resource is automatically matched with resource " + r.copyFromUtype
						: "",
					onChange: (inst, ui) => {
						update_dependencies(inst, r);
					},
				});
			}
		});

		configurables.push(def);
	});

	return configurables;
}

function optionValues(val) {
	var option = [];
	for (var i = 0; i < 1 << val; i++) {
		option.push({
			name: i,
			displayName: i.toString(),
		});
	}
	return option;
}

// Check if host is present in restrictHost of resource

function checkRestrictHost(hostName, resource) {
	var restrictHostFound = 0,
		hostFound = 0;
	_.each(resource.resRange, (range) => {
		if (range.restrictHosts) {
			restrictHostFound = 1;
			_.each(range.restrictHosts, (res) => {
				if (res === hostName) {
					hostFound = 1;
				}
			});
		}
	});

	return !(restrictHostFound && !hostFound);
}

function createHostModule(hostInfo) {
	var configurables = getHostConfigurables(hostInfo.hostName);
	var hostDocumentation = `
**${hostInfo.displayName}**

---

*	Host Name - HOST_ID_${hostInfo.hostName}
*	Host ID - ${hostInfo.hostId}
*	Security - ${hostInfo.Security}
*	Description - ${hostInfo.Description}
	`;
	var def = {
		displayName: hostInfo.hostName,
		longDescription: hostDocumentation + documentation,
		moduleStatic: {
			config: [
				{
					name: "HostCapabilities",
					displayName: "Host Capabilities",
					config: [
						// Host Name
						{
							name: "hostName",
							default: hostInfo.hostName,
							hidden: true,
						},
						// Allowed atypes
						{
							name: "allowedAtype",
							displayName: "Allowed values of atype",
							options: [
								{
									name: 0,
									displayName: "Physical address",
								},
								{
									name: 1,
									displayName: "Intermediate Physical address",
								},
								{
									name: 2,
									displayName: "Virtual address",
								},
							],
							default: [0, 1, 2],
						},
						// Allowed values of qos
						{
							name: "allowedqos",
							displayName: "Allowed values of qos",
							default: "unknown",
							options: optionValues(3),
							default: [0, 1, 2, 3, 4, 5, 6, 7],
						},
						// Allowed values of orderid
						{
							name: "allowedorderid",
							displayName: "Allowed values of orderid",
							default: "unknown",
							options: optionValues(4),
							default: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15],
						},
						// Allowed values of priority
						{
							name: "allowedpriority",
							displayName: "Allowed values of priority",
							default: "unknown",
							options: optionValues(3),
							default: [0, 1, 2, 3, 4, 5, 6, 7],
						},
						// Allowed values of schedpriority
						{
							name: "allowedschedpriority",
							displayName: "Allowed values of schedpriority",
							default: "unknown",
							options: optionValues(2),
							default: [0, 1, 2, 3],
						},
						// Supervisor Host
						{
							name: "supervisorhost",
							displayName: "Supervisor Host",
							options: [
								{
									name: "none",
									displayName: "None",
								},
								...hostNames,
							],
							default: "none",
						},
						// Share Resource With Host
						{
							name: "shareResource",
							displayName: "Share Resources with",
							options: [
								{
									name: "none",
									displayName: "None",
								},
								...hostNames,
							],
							default: "none",
						},
						{
							name: "allocOrder",
							displayName: "Allocation order",
							longDescription: "This value is used to sort the hosts and allocate the resources in the ascending order of allocOrder. This ensures that the allocation is always predictable irrespective of the sequence of adding hosts or sequence of adding resource counts.",
							default: 0
						},
					],
				},
				...configurables,
			],
			validate: (instance, report) => {

				moduleInstances = utils.getModuleInstances();
				overlapAndOverflow(instance, report);
				duplicateShareResourceWithHost(instance, report);
				duplicateHostAndShareHost(instance, report);
				checkSupervisorTreeCyclic(instance, report);
				checkValidOrder(instance, report);
				updateSharedResource(instance, report);
			},
		},
	};

	return def;
}

function update_dependencies(inst, r) {
	var src, dest;

	src = _.join(_.split(r.utype, " "), "_");

	// Update block copy dependencies first
	for (var i = 0; i < r.blockCopyTo.length; i++) {
		dest = _.join(_.split(r.blockCopyTo[i], " "), "_");

		inst[dest + "_blockCount"] = inst[src + "_blockCount"];
		update_dependencies(inst, resources[r.blockCopyTo[i]]);
	}

	for (var i = 0; i < r.copyToUtype.length; i++) {
		dest = _.join(_.split(r.copyToUtype[i], " "), "_");

		if (r.autoAlloc === false) {
			inst[dest + "_start"] = inst[src + "_start"];
		}

		inst[dest + "_count"] = inst[src + "_count"];

		if (r.blockCopy) {
			inst[dest + "_blockCount"] = inst[src + "_blockCount"];
		}

		if(r.extended) {
			inst[dest + "_hwa_start"] = inst[src + "_hwa_start"];
			inst[dest + "_hwa_count"] = inst[src + "_hwa_count"];
		}
	}
}
// Functions for validation

// Check if same host is selected to share resource with different hosts
function duplicateShareResourceWithHost(instance, report) {
	for (var idx = 0; idx < moduleInstances.length; idx++) {
		if (
			instance.shareResource !== "none" &&
			instance.shareResource === moduleInstances[idx].shareResource &&
			instance != moduleInstances[idx]
		) {
			report.logError("Cannot Share resources with same host twice", instance, "shareResource");
		}
	}
}

// Check if same host is selected as Host as well as Share resource with
function duplicateHostAndShareHost(instance, report) {
	for (var idx = 0; idx < moduleInstances.length; idx++) {
		if (instance.shareResource === moduleInstances[idx].hostName) {
			report.logError("Resources are already assigned to host", instance, "shareResource");
		}
	}
}

// Check if the Supervisor tree is cyclic or not
function checkSupervisorTreeCyclic(instance, report) {
	var node = instance;
	var count = hostNames.length + 2;

	while(node.supervisorhost != "none" && count > 0) {
		var moduleName = "/modules/" + socName + "/" + node.supervisorhost
		if (!system.modules[moduleName])
			break;
		node = system.modules[moduleName].$static;
		if (node == instance) {
			report.logError("Supervisor tree is Tree is cyclic", instance, "supervisorhost");
			break;
		}
		count--;
	}
}

// Check for overlap and overflow
function overlapAndOverflow(instance, report) {
	_.each(resources, (resource) => {
		var name = _.join(_.split(resource.utype, " "), "_");

		if (instance[name + "_count"] == 0 && instance[name + "_blockCount"] == 0) {
			return;
		}
		if (resource.autoAlloc === false || resource.extended === true) {
			var utype = resource.utype;
			if(resource.extended === true) {
				var invalidRange = checkInvalidHwaRange(utype, instance);
				if(invalidRange === 0) {
					report.logWarning(`Warning : Invalid start/count for HWA, entry will be discarded`, instance);
				}
				utype += " hwa";
			}
			var overlapInstance = checkOverlap(utype, instance);
			if (overlapInstance.length) {
				const conflicting = _.join(
					_.map(overlapInstance, (inst) => system.getReference(inst)),
					", "
				);

				report.logWarning(`WARNING : Overlap with ${conflicting}`, instance, name + "_count");
			}
			
			return;
		}

		var over = resourceAllocate(resource.utype).overflowCount;

		var index = -1, id = 0;
		_.each(resource.resRange, (range) => {
			if (range.restrictHosts) {
				_.each(range.restrictHosts, (res) => {
					if (res === instance.hostName) {
						index = id;
					}
				});
			} else {
				index = id;
			}
			id++;
		});

		if (index !== -1 && over[index] > 0) {
			report.logError(
				"ERROR : Assigned resource count exceeds by " + over[index],
				instance,
				name + "_count"
			);
		}
	});
}

function checkValidOrder(inst, report) {
	for (var idx = 0; idx < moduleInstances.length; idx++) {
		var other = moduleInstances[idx];
		if (other == inst)
			continue;
		if (other.allocOrder == inst.allocOrder) {
			report.logError("Invalid Allocation order", inst, "allocOrder");
		}
	}
}

function updateSharedResource(inst, report) {
	var sharedResourceList = utils.getSharedResourceList();
	for (let res of sharedResourceList ){
		if(res.resourceName != 'none'){
			if(res.sharedFromHostName == inst.hostName){
				var name = _.join(_.split(res.resourceName , " "), "_");
				report.logInfo("This resource is shared with " + res.sharedToHostName + " host", inst, name + "_count");
			}
			else if (res.sharedToHostName == inst.hostName){
				var name = _.join(_.split(res.resourceName , " "), "_");
				report.logInfo("This resource is shared from " + res.sharedFromHostName + " host", inst, name + "_count");
			}
		}
	}
}
exports = {
	createHostModule,
};
