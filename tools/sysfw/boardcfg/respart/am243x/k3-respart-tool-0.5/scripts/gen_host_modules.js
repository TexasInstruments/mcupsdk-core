const args = require("yargs")
.options({
	soc: {
		describe: "Soc name",
		demandOption: true,
		type: "string",
	},
})
.help()
.alias("help", "h").argv;

var soc = args.soc;

var fs = require("fs");
var dir = process.argv[1].substring(0, process.argv[1].lastIndexOf("/"));
var path = dir + "/../data/" + soc + "/Hosts.json";
var hosts = fs.readFileSync(path);

var modulePath = [];

hosts = JSON.parse(hosts);

hosts.forEach((host) => {
	const def = `
const {createHostModule} = system.getScript("/modules/sysfwResPart.js");
const hostInfo = ${JSON.stringify(host, null, 2)};
const modDef = createHostModule(hostInfo);
exports = modDef;
`;
	var path = dir + "/../modules/" + soc + "/" + host.hostName + ".syscfg.js";
	fs.writeFileSync(path, def);

	modulePath.push("/modules/" + soc + "/" + host.hostName);
});
