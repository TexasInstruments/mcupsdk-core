var utils = system.getScript("/scripts/utils.js");

const resources = utils.resources;
const socName = utils.socName;
const hosts = utils.hosts;

function checkOverlap(utype, inst1) {
	var hostsInstances = utils.getSelectedHostInstances();
	var overlap = [];
	var name = _.join(_.split(utype, " "), "_");
	var start1 = inst1[name + "_start"];
	var last1 = start1 + inst1[name + "_count"];

	_.each(hostsInstances, (host) => {
		var inst2 = host;
		if (inst1 !== inst2) {
			var start2 = inst2[name + "_start"],
				last2 = start2 + inst2[name + "_count"];
			if (Math.max(start1, start2) < Math.min(last1, last2)) {
				overlap.push(inst2);
			}
		}
	});

	return overlap;
}

function checkInvalidHwaRange(utype, inst) {
	var name = _.join(_.split(utype, " "), "_");
	var resStart = inst[name + "_hwa_start"];
	var resCount = inst[name + "_hwa_count"];
	var resdata = resources[utype];
	var startFlag = 0;
	var validRange = 0;
	var count = 0;
	for(var i=0; i < resdata.hwaRange.length ; i++)
	{
		var hwaRange = resdata.hwaRange[i];
		if(startFlag === 0){
			if(resStart === hwaRange.start) {
				startFlag = 1;
				count += hwaRange.count;
				if(resCount === count){
					validRange = 1;
					break;
				}
			}
		} else {
			count += hwaRange.count;
			if(resCount === count){
				validRange = 1;
				break;
			}
		}
	}
	if(resCount === 0)
	{
		validRange = 1;
	}
	return validRange;
}

function add_allocation(allocation, entry, sharedHost=undefined) {
	allocation.push(entry);
	if (sharedHost && sharedHost != "none") {
		var copy = {
			utype: entry.utype,
			hostName: sharedHost,
			start: entry.start,
			count: entry.count,
		};
		allocation.push(copy);
	}
}

function get_avail_ranges(utype) {
	var resdata = resources[utype];
	var avail_range = [];

	for (var i = 0; i < resdata.resRange.length; i++) {
		var range = resdata.resRange[i];

		avail_range.push({
			start: range.resStart,
			count: range.resCount,
			restrictHosts: range.restrictHosts,
			used: 0,
		});
	}
	return avail_range;
}

function get_idx_for_host(avail_ranges, hostName) {
	var idx = 0;
	if (avail_ranges.length > 1){
		for(var i = 0; i < avail_ranges.length; i++) {
			if (avail_ranges[i].restrictHosts.includes(hostName)) {
				idx = i;
				break;
			}
		}
	}
	return idx;
}

function getSharedHost(utype, hostsInstances){
	var sharedResourceList = utils.getSharedResourceList();
	for (let sharedResource of sharedResourceList ){
		if(sharedResource.resourceName == utype && hostsInstances.hostName == sharedResource.sharedFromHostName ){
			return sharedResource.sharedToHostName;
		}
	}
	return hostsInstances.shareResource;
}

function resourceAllocate(utype, addShared) {
	var hostsInstances = utils.getSelectedHostInstances();
	var avail_ranges = get_avail_ranges(utype);
	var prefix = _.join(_.split(utype, " "), "_");
	var allocation = [], overflow = [];
	var start, total, used, idx;
	var sharedHost;

	_.each(hostsInstances, (inst) => {

		if (addShared) {
			sharedHost = getSharedHost(utype, inst);
		} else {
			sharedHost = undefined;
		}

		var entry = {
			utype: utype,
			hostName: inst.hostName,
			start: inst[prefix + "_start"],
			count: inst[prefix + "_count"],
		};

		// Use block copy count for allocation first
		// Generate an entry even if either count or blockCount is non zero
		if (resources[utype].blockCopy) {
			var block_count = inst[prefix + "_blockCount"];
			if (entry.count == 0 && block_count == 0) {
				return;
			}
			entry.count = block_count;
		} else if (resources[utype].extended) {
			// Generate an entry even if either count or hwa count is non zero
			var ext_count = inst[prefix + "_hwa_count"];
			if (entry.count == 0 && ext_count == 0) {
				return;
			}
		} else if (entry.count == 0) {
			return;
		}

		idx = get_idx_for_host(avail_ranges, inst.hostName);
		total = avail_ranges[idx].count;
		start = avail_ranges[idx].start;
		start += avail_ranges[idx].used;

		if (resources[utype].autoAlloc === false) {
			//Nothing to be done, start/count is already populated
		} else {
			entry.start = start;
		}

		avail_ranges[idx].used += entry.count;
		add_allocation(allocation, entry, sharedHost);
	});

	// Add second entry after block copy allocation
	_.each(hostsInstances, (inst) => {

		if (addShared) {
			sharedHost = getSharedHost(utype, inst);
		} else {
			sharedHost = undefined;
		}

		var entry = {
			utype: utype,
			hostName: inst.hostName,
			start: inst[prefix + "_start"],
			count: inst[prefix + "_count"],
		};

		idx = get_idx_for_host(avail_ranges, inst.hostName);
		total = avail_ranges[idx].count;
		start = avail_ranges[idx].start;
		start += avail_ranges[idx].used;
		entry.start = start;
		if (resources[utype].blockCopy) {
			if (inst[prefix + "_count"]) {
				avail_ranges[idx].used += entry.count;
				start += entry.count;
				add_allocation(allocation, entry, sharedHost);
			}
		} else if (resources[utype].extended) {
			var resdata = resources[utype];
			entry.start = inst[prefix + "_hwa_start"];
			entry.count = inst[prefix + "_hwa_count"];
			var startFlag = 0;
			var validRange = 0;
			var count = 0;
			var startHwa = resdata.hwaStart;
			for(var i=0; i < resdata.hwaRange.length ; i++)
			{
				var hwaRange = resdata.hwaRange[i];
				if(startFlag === 0){
					if(entry.start === hwaRange.start) {
						startFlag = 1;
						count += hwaRange.count;
						if(entry.count === count){
							validRange = 1;
							break;
						}
					}
				} else {
					count += hwaRange.count;
					if(entry.count === count){
						validRange = 1;
						break;
					}
				}
			}

			if(validRange === 0){
				return;
			}

			if (inst[prefix + "_hwa_count"]) {
				start += entry.count;
				add_allocation(allocation, entry, sharedHost);
			}
		}
	});

	// Add remaining resources to HOST_ID_ALL
	idx = get_idx_for_host(avail_ranges, "ALL");
	total = avail_ranges[idx].count;
	start = avail_ranges[idx].start;
	start += avail_ranges[idx].used;
	if(resources[utype].extended) {	
		var resdata = resources[utype];
		for(var i=0; i < resdata.hwaRange.length ; i++)
		{
			var hwaRange = resdata.hwaRange[i];
			avail_ranges[idx].used += hwaRange.count;
		}
	}

	used = avail_ranges[idx].used;
	if (total - used > 0 && resources[utype].autoAlloc != false) {

		// Add HOST_ID_ALL if (a) there is single range OR (b) there is dedicated range
		if (avail_ranges.length == 1 || avail_ranges[idx].restrictHosts.includes("ALL")) {
			allocation.push({
				utype: utype,
				hostName: "ALL",
				start: start,
				count: total - used,
			});
		}
	}

	for (var i = 0; i < avail_ranges.length; i++) {
		overflow.push(Math.max(0, avail_ranges[i].used - avail_ranges[i].count));
	}
	return {
		allocation: allocation,
		overflowCount: overflow,
	};
}

function allocateAndSort(skipZeroEntries, addShareResourceEntries) {
	var allocation = [];

	_.each(resources, (resource) => {
		var res = resourceAllocate(resource.utype, addShareResourceEntries).allocation;
		res.sort(function (a, b) {
			if (a.start < b.start) {
				return -1;
			} else if (a.start > b.start) {
				return 1;
			} else {
				var h1 = a.hostName,
					h2 = b.hostName;
				if (h1 === "ALL") return 1;
				if (h2 === "ALL") return -1;
				if (h1 == h2)
					return a.count - b.count;
				else
					return hosts[h1].hostId - hosts[h2].hostId;
			}
		});
		if (res.length) allocation.push(res);
	});
	allocation.sort(function (a, b) {
		var u1 = a[0].utype,
			u2 = b[0].utype;
		return resources[u1].uniqueId - resources[u2].uniqueId;
	});

	return allocation;
}

function mapByResources(skipZeroEntries, addShareResourceEntries) {
	var allocation = allocateAndSort(skipZeroEntries, addShareResourceEntries);
	var resourcesMap;
	if (allocation.length) {
		resourcesMap = _.keyBy(allocation, (all) => all[0].utype);
	}
	return resourcesMap;
}

exports = {
	allocateAndSort,
	checkOverlap,
	checkInvalidHwaRange,
	resourceAllocate,
	mapByResources,
};
