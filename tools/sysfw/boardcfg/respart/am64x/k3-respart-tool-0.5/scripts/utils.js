const deviceSelected = system.deviceData.device;
const devData = _.keyBy(system.getScript("/data/SOC.json"), (r) => r.soc);
const tifsResRem = _.keyBy(system.getScript("/data/tifsResRem.json"), (r) => r);
const socName = devData[deviceSelected].shortName;

const resources = _.keyBy(system.getScript("/data/" + socName + "/Resources.json"), (r) => r.utype);
var FirewallDevices = _.keyBy(system.getScript("/data/" + socName + "/Firewall.json"), (d) => d.name);
const qos = _.keyBy(system.getScript("/data/" + socName + "/Qos.json"), (r) => r.endpointName);
const hosts = _.keyBy(system.getScript("/data/" + socName + "/Hosts.json"), (r) => r.hostName);

// List of all hosts
var hostNames = [];
_.each(hosts, (h) => {
	hostNames.push({
		name: h.hostName,
		displayName: h.hostName,
	});
});

// Find all unique group names

var groupNames = [];

_.each(resources, (resource) => {
	groupNames.push(resource.groupName);
});
groupNames = _.uniq(groupNames);


// Create map of groupName to resources

var resourcesByGroup = _.groupBy(resources, (r) => {
	return r.groupName;
});

// list of all resoureces
var reslist = [];
_.each(groupNames, (gName) => {
	var groupResources = resourcesByGroup[gName];
	_.each(groupResources, (r) => {
		reslist.push({
			name: r.utype,
			displayName: r.utype,
		});
	});
});

// Set the bits to 10 if a option is selected else to 01

function setBit(Resource, maxValue) {
	// This array contain option that are not selected

	var notSelected = [];
	for (var i = 0; i < maxValue; i++) {
		var found = 0;
		for (var j = 0; j < Resource.length; j++) {
			if (Resource[j] === i) {
				found = 1;
			}
		}
		if (!found) {
			notSelected.push(i);
		}
	}
	// set the selected bits to 10

	var val = 0;
	for (var i = 0; i < Resource.length; i++) {
		var bit = Resource[i];
		val |= 1 << (2 * bit + 1);
	}

	// set the unselected bits to 01

	for (var i = 0; i < notSelected.length; i++) {
		var bit = notSelected[i];
		val |= 1 << (2 * bit);
	}
	return val;
}

function decimalToBinary(val) {
	return "0b" + val.toString(2);
}

function decimalToHexadecimal(val) {
	return "0x" + val.toString(16).toUpperCase();
}

function unsignedToBinary(Resource) {
	var notSelected = [];
	var maxValue = 16;
	for (var i = 0; i < maxValue; i++) {
		var found = 0;
		for (var j = 0; j < Resource.length; j++) {
			if (Resource[j] === i) {
				found = 1;
			}
		}
		if (!found) {
			notSelected.push(i);
		}
	}
	var val = [];
	for (var i = 0; i < Resource.length; i++) {
		var bit = Resource[i];
		val[2 * bit + 1] = 1;
	}
	for (var i = 0; i < notSelected.length; i++) {
		var bit = notSelected[i];
		val[2 * bit] = 1;
	}
	var str = "";
	for (var i = 31; i >= 0; i--) {
		if (val[i]) str += "1";
		else str += "0";
	}
	return str;
}

function toHexa(val) {
	var hex = "";
	for (var i = 7; i >= 0; i--) {
		var str = val[4 * i] + val[4 * i + 1] + val[4 * i + 2] + val[4 * i + 3];
		hex += parseInt(str, 2).toString(16).toUpperCase();
	}
	return "0x" + hex;
}

function removePrefix(str) {
	var pieces = _.split(str, "_");
	pieces.shift();
	return _.join(pieces, "_");
}

function addPrefix(str) {
	return devData[deviceSelected].sciClientPrefixReplacement + str;
}

// return mask for qos parameters

function getQosValue(inst) {
	var value = "";

	if (inst.atype)
		value += " | ATYPE_" + inst.atype;
	if (inst.virtId)
		value += " | VIRTID_" + inst.virtId;
	if (inst.epriority)
		value += " | EPRIORITY_" + inst.epriority;
	if (inst.asel)
		value += " | ASEL_" + inst.asel;
	if (inst.orderId)
		value += " | ORDERID_" + inst.orderId;
	if (inst.qos)
		value += " | QOS_" + inst.qos;

	if (value == "")
		return "0;"
	value = value.replace(" | ", "");
	return value;
}

// Return all the selected endpoints

function endPoint() {
	var endPoint = [];

	if (system.modules["/modules/qosConfig"]) {
		for (let inst of system.modules["/modules/qosConfig"].$instances) {
			_.each(inst.qosdev, (e) => {
				endPoint.push(inst.deviceName + "_" + e);
			});
		}
	}

	return endPoint;
}

// Return permission mask for firewall entries

function getPermissionMask(p) {
	var val = 0;
	var order = [
		"s_supervisor_wr",
		"s_supervisor_rd",
		"s_supervisor_cache",
		"s_supervisor_debug",
		"s_user_wr",
		"s_user_rd",
		"s_user_cache",
		"s_user_debug",
		"ns_supervisor_wr",
		"ns_supervisor_rd",
		"ns_supervisor_cache",
		"ns_supervisor_debug",
		"ns_user_wr",
		"ns_user_rd",
		"ns_user_cache",
		"ns_user_deb",
	];

	for (var idx = 0; idx < order.length; idx++) {
		if (p[order[idx]]) {
			val |= p[order[idx]] << idx;
		}
	}

	var pid = p.privid;
	for (var idx = 0; idx < 8; idx++) {
		if (pid & (1 << idx)) {
			val |= 1 << (16 + idx);
		}
	}
	return "0x" + val.toString(16).toUpperCase();
}

// return control mask for firewall entries

function getControlMask(r) {
	var val = 10;
	if (r.lock) {
		val |= 1 << 4;
	}

	if (r.background) {
		val |= 1 << 8;
	}
	val |= 1 << 9;

	return "0x" + val.toString(16).toUpperCase();
}

function getIdsOfSelectedInstances(selectedDeviceInstances, device) {
	var selectedIds = [];

	var deviceInstances = device.protected_inst;
	var deviceIds = device.ids;

	_.each(deviceInstances, (i, idx) => {
		_.each(selectedDeviceInstances, (d) => {
			if (i === d) {
				selectedIds.push(deviceIds[idx]);
			}
		});
	});

	return selectedIds;
}

function getSharedResourceList(){
	var entries = [];
	if (system.modules["/modules/resourceSharing"]) {
		for (let inst of system.modules["/modules/resourceSharing"].$instances) {
			entries.push(inst)
		}
	}
	return entries;
}
function getSharedResourceMap(){
	var entries = [];
	if (system.modules["/modules/resourceSharing"]) {
		for (let inst of system.modules["/modules/resourceSharing"].$instances) {
			if(inst.resourceName != "none" && inst.sharedFromHostName != "none" && inst.sharedToHostName != "none"){
				var entry ={
					from:inst.sharedFromHostName,
					to:inst.sharedToHostName
				}
				if(!entries[inst.resourceName]) {entries[inst.resourceName] = []}
				entries[inst.resourceName].push(entry)
			}
		}
	}
	return entries;
}

function generateFirewallEntries() {
	var entries = [];
	if (system.modules["/modules/firewallConfig"]) {
		for (let inst of system.modules["/modules/firewallConfig"].$instances) {
			var device = FirewallDevices[inst.device];
			var ids = getIdsOfSelectedInstances(inst.instanceName, device);

			var comment = `${inst.$name} - ${ids.length} firewalls with ${inst.regions.length} regions each`;
			_.each(ids, (id) => {
				var regions = inst.regions;

				_.each(regions, (r, idx) => {
					var params = r.perms;
					var premissionArray = [];

					_.each(params, (p) => {
						premissionArray.push(getPermissionMask(p));
					});

					entries.push({
						fwl_id: id,
						start: r.addrStart,
						end: r.addrEnd,
						region: idx,
						permissions: premissionArray,
						n_permission_regs: params.length,
						control: getControlMask(r),
						comment: comment,
					});
					comment = "";
				});
			});
		}
	}

	return entries;
}

// Get value for the selected msmc cache size

function getNumber(val) {
	var TotalSize = devData[deviceSelected].sRAMSize;
	var cacheSize = TotalSize - val;

	val = (32 * cacheSize) / TotalSize;

	return "0x" + val.toString(16).toUpperCase();
}

function getDisplayPrefix(toBeRemoved, removeFrom) {
	var finalPrefix = "",
		idx = 0;

	var split1 = _.split(toBeRemoved, " ");
	var split2 = _.split(removeFrom, " ");

	for (var i = 0; i < split2.length; i++) {
		if (split1[i] !== split2[i]) {
			idx = i;
			break;
		}
	}

	for (var i = idx; i < split2.length; i++) {
		finalPrefix += split2[i];
		finalPrefix += " ";
	}

	return finalPrefix;
}

function getBlockCopyDisplayName(name) {
	var chunk = _.split(name, " ");
	name = "";
	_.each(chunk, (c) => {
		if (c === "Rx" || c === "Tx");
		else {
			name += c;
			name += " ";
		}
	});

	return name;
}

function getSelectedHostInstances() {
	var allModules = system.modules;
	var instances = [];

	_.each(allModules, (m) => {
		// Filter Out those modules having static instance
		var staticInstance = m.$static;
		if (staticInstance && staticInstance.hostName) {
			instances.push(staticInstance);
		}
	});

	instances.sort(function (a, b) {
		return a.allocOrder - b.allocOrder;
	})

	return instances;
}

function getModuleInstances() {
	var moduleInstances = [];
	_.each(hosts, (host) => {
		var moduleName = "/modules/" + socName + "/" + host.hostName;
		if (system.modules[moduleName]) {
			moduleInstances.push(system.modules[moduleName].$static);
		}
	});
	return moduleInstances;
}

exports = {
	setBit,
	decimalToBinary,
	decimalToHexadecimal,
	unsignedToBinary,
	toHexa,
	removePrefix,
	addPrefix,
	getQosValue,
	endPoint,
	generateFirewallEntries,
	getNumber,
	getDisplayPrefix,
	getBlockCopyDisplayName,
	getSelectedHostInstances,
	getSharedResourceList,
	getSharedResourceMap,
	getModuleInstances,
	deviceSelected,
	devData,
	socName,
	tifsResRem,
	resources,
	reslist,
	FirewallDevices,
	hosts,
	hostNames,
	qos,
	resourcesByGroup,
	groupNames,
};
