const args = require("yargs")
.options({
	doc: {
		alias: "document",
		describe: "Path to hostName.rst file",
		demandOption: true,
		type: "string",
	},
	soc: {
		describe: "Soc name",
		demandOption: true,
		type: "string",
	},
	names: {
		describe: "Host names",
		type: "string",
	},
	firewall: {
		describe: "Path to firewall.rst file",
		demandOption: true,
		type: "string",
	},
})
.help()
.alias("help", "h").argv;

// Parse the host rst file

function createHostArray(path) {
	var hostArray = [];
	var fs = require("fs");
	var textByLine = fs.readFileSync(path).toString().split("\n");

	// Split each line using | character and read data

	for (var line = 0; line < textByLine.length; line++) {
		if (textByLine[line][0] != "|") continue;

		var newText = textByLine[line].split("|");

		for (var data = 0; data < newText.length; data++) {
			newText[data] = newText[data].trim();
		}

		var newhost = {
			hostId: parseInt(newText[1]),
			hostName: newText[2],
			Security: newText[3],
			Description: newText[4],
		};

		hostArray.push(newhost);
	}

	// Remove invalid entries at begining

	while (Number.isNaN(hostArray[0].hostId)) hostArray.shift();

	// Merge if there are multiline description of hosts

	var afterMerging = [];
	for (var idx = 0; idx < hostArray.length; idx++) {
		if (Number.isNaN(hostArray[idx].hostId)) {
			afterMerging[afterMerging.length - 1].Description += " ";
			afterMerging[afterMerging.length - 1].Description += hostArray[idx].Description;
		} else {
			afterMerging.push(hostArray[idx]);
		}
	}

	hostArray = afterMerging;

	return hostArray;
}

// Parse firewall file to get priv-Ids of hosts

function addPrivIds(hostArray, path) {
	var fs = require("fs");
	var textByLine = fs.readFileSync(path).toString().split("\n");

	var privEntries = [];
	var privTableStart = 0;

	textByLine.forEach((line) => {
		if (privTableStart) {
			privEntries.push(line);
		} else {
			if (line.trim() === "List of priv-ids") privTableStart = 1;
		}
	});

	textByLine = privEntries;

	textByLine.forEach((line) => {
		if (line[0] === "|") {
			var newText = line.split("|");
			var privId = parseInt(newText[2], 10);
			if (privId) {
				var hostsId = newText[7];
				var t = hostsId.split(",").map((id) => {
					return parseInt(id, 10);
				});
				hostsId = t;
				hostsId.forEach((id) => {
					hostArray.forEach((h) => {
						if (h.hostId === id) {
							h.privId = privId;
						}
					});
				});
			}
		}
	});

	return hostArray;
}

function addDisplayName(hostArray, path) {
	var fs = require("fs");
	if (fs.existsSync(path)) {
		var data = fs.readFileSync(path);
		var names = JSON.parse(data);

		hostArray.forEach((h) => {
			h.displayName = h.hostName;
			names.forEach((n) => {
				if (h.hostName == n.hostName) {
					h.displayName = n.displayName
				}
			})
		});
	}
	return hostArray;
}

function createOutputFile(hosts, soc) {
	var fs = require("fs");

	// Make json string from object
	var jsonString = JSON.stringify(hosts);

	// write the data to file
	var dir = process.argv[1].substring(0, process.argv[1].lastIndexOf("/"));

	var path = dir + "/../data/" + soc + "/Hosts.json";

	fs.writeFile(path, jsonString, (err) => {
		if (err) throw err;
	});
}

var hostArray = createHostArray(args.doc);

hostArray = addPrivIds(hostArray, args.firewall);

if (args.names) {
	hostArray = addDisplayName(hostArray, args.names);
}

createOutputFile(hostArray, args.soc);
