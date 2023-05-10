const args = require("yargs")
	.options({
		doc: {
			alias: "document",
			describe: "Path to resasg_types.rst file",
			demandOption: true,
			type: "string",
		},
		soc: {
			describe: "Soc name",
			demandOption: true,
			type: "string",
		},
		dep: {
			alias: "dependency",
			describe: "Path to dependency file",
			type: "string",
		},
	})
	.help()
	.alias("help", "h").argv;

function createResources(path) {
	var resources = [];

	// Read the file about resource information and split line by line
	var fs = require("fs");
	var textByLine = fs.readFileSync(path).toString().split("\n");

	// Extract resource data
	for (var line = 0; line < textByLine.length; line++) {
		if (textByLine[line][0] != "|") continue;

		var newText = textByLine[line].split("|");

		// Remove extra spaces
		for (var data = 0; data < newText.length; data++) {
			newText[data] = newText[data].trim();
		}

		// Handle cases where entry is empty by
		// replacing with previous row values
		var sz = resources.length;
		var newResource = {
			deviceName: newText[1] === "" ? resources[sz - 1].deviceName : newText[1],

			deviceId: newText[2] === "" ? resources[sz - 1].deviceId : parseInt(Number(newText[2]), 10),

			subtypeName: newText[3] === "" ? resources[sz - 1].subtypeName : newText[3],

			subtypeId: newText[4] === "" ? resources[sz - 1].subtypeId : parseInt(Number(newText[4]), 10),

			uniqueId: newText[5] === "" ? resources[sz - 1].uniqueId : parseInt(Number(newText[5]), 10),

			resStart: parseInt(newText[6]),

			resCount: parseInt(newText[7]),

			utype: "",
		};
		resources.push(newResource);
	}

	// Remove invalid entries
	while (Number.isNaN(resources[0].deviceId)) resources.shift();

	return resources;
}

function createResourceRange(resources) {
	var newResources = [];

	// For resources where deviceName and subtypename are same create a single entry

	for (var idx = 0; idx < resources.length;) {
		var deviceName = resources[idx].deviceName;
		var subtypeName = resources[idx].subtypeName;
		var deviceId = resources[idx].deviceId;
		var subtypeId = resources[idx].subtypeId;
		var uniqueId = resources[idx].uniqueId;

		var resArray = [];
		var count  = 0;

		while (idx < resources.length && resources[idx].deviceName === deviceName && resources[idx].subtypeName === subtypeName) {
			resArray.push({
				resStart: resources[idx].resStart,
				resCount: resources[idx].resCount,
			});
			count += resources[idx].resCount;

			idx++;
		}

		newResources.push({
			deviceName: deviceName,
			deviceId: deviceId,
			subtypeName: subtypeName,
			subtypeId: subtypeId,
			uniqueId: uniqueId,
			resRange: resArray,
			utype: "",
		});
	}

	return newResources;
}

function addDependencies(resources) {
	var process = require("process");
	var fs = require("fs");

	// If a argument is given then use it as a path and read file to assign attributes if possible
	if (args.dep && fs.existsSync(args.dep)) {
		var path = args.dep;
		var str = fs.readFileSync(path).toString();

		var dependency = JSON.parse(str);

		var newResources = [];

		for (var dep = 0; dep < dependency.length; dep++) {
			var gName = dependency[dep].groupName;
			var depResource = dependency[dep].resources;

			for (var res = 0; res < depResource.length; res++) {
				for (parseRes = 0; parseRes < resources.length; parseRes++) {
					if (
						resources[parseRes].deviceName === depResource[res].deviceName &&
						resources[parseRes].subtypeName === depResource[res].subtypeName
					) {
						var foundResource = Object.assign({}, resources[parseRes]);
						resources[parseRes].found = 1;
						foundResource.utype = depResource[res].utype;
						foundResource.groupName = gName;
						if (depResource[res].copyFromUtype) {
							foundResource.copyFromUtype = depResource[res].copyFromUtype;
						}
						if (depResource[res].blockCopyFrom) {
							foundResource.blockCopyFrom = depResource[res].blockCopyFrom;
						}
						if (depResource[res].autoAlloc === false) {
							foundResource.autoAlloc = depResource[res].autoAlloc;
						}
						if (depResource[res].blockCopy) {
							foundResource.blockCopy = depResource[res].blockCopy;
						}
						if (depResource[res].extended) {
							foundResource.extended = depResource[res].extended;
						}
						if (depResource[res].hwaRange) {
							foundResource.hwaRange = depResource[res].hwaRange;
						}
						if (depResource[res].restrictHosts) {
							for (var idx = 0; idx < foundResource.resRange.length; idx++) {
								foundResource.resRange[idx].restrictHosts = depResource[res].restrictHosts;
							}
						}
						if (depResource[res].resRange) {
							foundResource.resRange = depResource[res].resRange;
						}
						newResources.push(foundResource);
					}
				}
			}
		}

		var resourcesNotFound = [];
		resources.forEach((r) => {
			if (!r.found) {
				resourcesNotFound.push(r);
			}
		});

		for (var idx = 0; idx < resourcesNotFound.length; idx++) {
			resourcesNotFound[idx].utype = "u_type_" + idx;
			resourcesNotFound[idx].groupName = "Others";

			newResources.push(resourcesNotFound[idx]);
		}
		resources = newResources;

		resources.forEach((r) => {
			delete r.found;
		});
	} else {
		for (var idx = 0; idx < resources.length; idx++) {
			resources[idx].utype = "u_type_" + idx;
		}
	}

	return resources;
}

function createOutputFile(resources, soc) {
	var fs = require("fs");
	var process = require("process");

	// Make json string from object
	var jsonString = JSON.stringify(resources);

	// write the data to file
	var dir = process.argv[1].substring(0, process.argv[1].lastIndexOf("/"));

	var path = dir + "/../data/" + soc + "/Resources.json";

	fs.writeFile(path, jsonString, (err) => {
		if (err) throw err;
	});
}

// Call each function defined above

var finalOutput = createResources(args.doc);

finalOutput = createResourceRange(finalOutput);

finalOutput = addDependencies(finalOutput);

createOutputFile(finalOutput, args.soc);
