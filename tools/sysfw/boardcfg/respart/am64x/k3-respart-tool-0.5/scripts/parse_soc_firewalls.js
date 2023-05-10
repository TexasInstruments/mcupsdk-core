const args = require('yargs')
	.options({
		"doc": {
			alias: "document",
			describe: "Path to SOC JSON file",
			demandOption: true,
			type: "string"
		},
		"soc": {
			describe: "Soc name",
			demandOption: true,
			type: "string"
		},
		"dname": {
			describe: "Path to DeviceName.json file",
			demandOption: true,
			type: "string"
		},
		"firewall": {
			describe: "Path to firewall.rst file",
			demandOption: true,
			type: "string"
		}
	})
	.help()
	.alias('help', 'h')
	.argv;

function onlyUnique(value, index, self) {
	return self.indexOf(value) === index;
}

function getInHexa(val) {
	var hex = val.toString(16).toUpperCase();

	var prefix = "0x";
	for (var idx = 0; idx < (12 - hex.length); idx++) {
		prefix += "0";
	}

	return prefix + hex;
}

function MinMaxOfRange(array) {
	var start = parseInt(array[0].start_address);
	var end = parseInt(array[0].end_address);

	array.forEach(r => {
		start = Math.min(start, parseInt(r.start_address));
		end = Math.max(end, parseInt(r.end_address));
	});

	return {
		start: getInHexa(start),
		end: getInHexa(end)
	}
}

function parseAndMergeFirewallData() {

	var fs = require('fs');

	var firewall = fs.readFileSync(args.doc).toString();

	var names = fs.readFileSync(args.dname).toString();

	names = JSON.parse(names);
	firewall = JSON.parse(firewall);

	firewall = firewall.security.std_slave_firewalls;

	var deviceNames = [];

	names.forEach(n => {
		deviceNames.push(n.name);

		if (!n.memory) {
			n.memory = false;
		}
	})

	var deviceNames = deviceNames.filter(onlyUnique)

	// Create map of device names

	var namesMap = new Map();

	for (var idx = 0; idx < names.length; idx++) {

		var inst = names[idx].protected_inst;

		inst.forEach(i => {
			namesMap[i] = {
				name: names[idx].name,
				memory: names[idx].memory
			}
		})
	}
	var finalData = [];

	var notFoundCount = 0;

	firewall.forEach(item => {

		if (item.protected_regions.length > 0) {

			var values = MinMaxOfRange(item.protected_regions);
			var start = values.start;
			var end = values.end;

			var devName = "",
				memory = false;

			if (namesMap[item.protected_inst]) {
				devName = namesMap[item.protected_inst].name;
				memory = namesMap[item.protected_inst].memory;
			} else {
				devName = "AAAAAA_" + notFoundCount;
				notFoundCount++;
			}

			finalData.push({
				id: item.id,
				num_regions: item.num_regions,
				protected_inst: item.protected_inst,
				name: devName,
				start_address: start,
				end_address: end,
				memory: memory
			})
		}
	})

	// Merge instances having same device name

	var instancesAfterMerging = [];

	deviceNames.forEach(n => {
		var interface = [];
		finalData.forEach(f => {
			if (n === f.name) {
				interface.push(f);
				f.found = 1;
			}
		})

		if (interface.length) {
			var ids = [];
			var region = interface[0].num_regions;
			var protected_inst = [];

			interface.forEach(i => {
				ids.push(i.id);
				region = Math.min(region, i.num_regions);
				protected_inst.push(i.protected_inst);
			})

			var values = MinMaxOfRange(interface);
			instancesAfterMerging.push({
				ids: ids,
				num_regions: region,
				name: n,
				start_address: values.start,
				end_address: values.end,
				memory: interface[0].memory,
				protected_inst: protected_inst
			})
		}
	})

	// Handle the case where device name is not found

	finalData.forEach(f => {
		if (!f.found) {
			instancesAfterMerging.push({
				ids: [f.id],
				num_regions: f.num_regions,
				name: f.name,
				start_address: f.start_address,
				end_address: f.end_address,
				memory: f.memory,
				protected_inst: [f.protected_inst]
			})
		}
	})

	finalData = instancesAfterMerging;

	return finalData;
}




function createOutputFile(data, soc) {

	var fs = require('fs');

	// Make json string from object
	var jsonString = JSON.stringify(data);

	// write the data to file
	var dir = process.argv[1].substring(0, process.argv[1].lastIndexOf('/'));

	var path = dir + "/../data/" + soc + "/Firewall.json";

	fs.writeFile(path, jsonString, (err) => {
		if (err) throw err;
	})
}
// Get firewalls used by dmsc and rm

function getUsedFirewalls() {

	var fs = require("fs");
	var textByLine = fs.readFileSync(args.firewall)
		.toString().split("\n");

	var startIndex = 0,
		endIndex = 0;

	for (var idx = 0; idx < textByLine.length; idx++) {
		if (textByLine[idx].trim() === "List of Region Based Firewalls") {
			startIndex = idx;
		}
		if (textByLine[idx].trim() === "List of Channelized Firewalls") {
			endIndex = idx;
		}
	}

	var table = [];

	for (var idx = startIndex; idx < endIndex; idx++) {
		if (textByLine[idx][0] === "|") {
			table.push(textByLine[idx]);
		}
	}

	textByLine = table;

	var firewallId = [];

	textByLine.forEach(t => {
		var arr = t.split("|");
		var fId = parseInt(arr[1].trim());

		if (fId) {
			if (arr[2].trim() !== "none") {
				firewallId.push(fId);
			}
		}
	})

	return firewallId;

}

// Remove firewalls used by dmsc

function removeUsedFirewalls(firewallData, usedFirewalls) {

	var afterRemoving = [];

	firewallData.forEach(f => {
		var ids = f.ids;
		var inst = f.protected_inst;

		var nonUsedIds = [];
		var nonUsedInst = [];

		for (var idx = 0; idx < ids.length; idx++) {
			var found = 0,
				i = ids[idx];

			usedFirewalls.forEach(u => {
				if (u === i) {
					found = 1;
				}
			})

			if (!found) {
				nonUsedIds.push(i);
				nonUsedInst.push(inst[idx]);
			}
		}

		if (nonUsedIds.length) {
			f.ids = nonUsedIds;
			f.protected_inst = nonUsedInst;
			afterRemoving.push(f);
		}
	})

	return afterRemoving;
}

// Merge instances having similar instances names

function mergeInterfaces(firewallData) {
	var devicesWithoutName = [];
	var dataAfterMerging = [];

	firewallData.forEach(f => {
		var t = f.name.split("_");

		if (t[0] === "AAAAAA") {
			devicesWithoutName.push(f);
		} else {
			dataAfterMerging.push(f);
		}
	})

	var uniqueInterfaceName = [];
	devicesWithoutName.forEach(d => {
		var n = d.protected_inst[0];
		n = n.split("_");
		n.pop();
		d.tempName = n.join("_");
		uniqueInterfaceName.push(d.tempName);
	})

	uniqueInterfaceName = uniqueInterfaceName.filter(onlyUnique);

	var index = 0;
	uniqueInterfaceName.forEach(i => {
		var sameDevice = [];
		devicesWithoutName.forEach(d => {
			if (i === d.tempName) {
				sameDevice.push(d);
			}
		})

		if (sameDevice.length) {

			var values = MinMaxOfRange(sameDevice);
			var start = values.start;
			var end = values.end;
			var r = sameDevice[0].num_regions;

			var ids = [];
			var inst = [];
			sameDevice.forEach(s => {
				ids.push(s.ids[0]);
				inst.push(s.protected_inst[0]);
				r = Math.min(r, s.num_regions);
			})
			dataAfterMerging.push({
				name: "ZZZZ_" + index,
				ids: ids,
				protected_inst: inst,
				num_regions: r,
				start_address: start,
				end_address: end,
				memory: sameDevice[0].memory
			})
			index++;
		}
	})

	return dataAfterMerging;
}

var firewallData = parseAndMergeFirewallData();

var usedFirewalls = getUsedFirewalls();

firewallData = removeUsedFirewalls(firewallData, usedFirewalls);

firewallData = mergeInterfaces(firewallData);


createOutputFile(firewallData, args.soc);