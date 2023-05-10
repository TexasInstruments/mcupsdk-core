var fs = require('fs');

function copy_attributes(inp, out, match, attrib) {

	console.log(`Copying attributes "${attrib}" from ${inp} to ${out} when "${match}" matches`);

	var input = JSON.parse(fs.readFileSync(inp).toString());
	var output = JSON.parse(fs.readFileSync(out).toString());

	if (input == undefined || output == undefined)
		return;

	processed = []
	output.forEach(out => {
		input.forEach(inp => {
			if (JSON.stringify(inp[match]) == JSON.stringify(out[match])) {
				out[attrib] = inp[attrib]
			}
		})
		processed.push(out)
	})
	output = processed

	fs.writeFile(out, JSON.stringify(output), (err) => {
		if (err) throw err;
	})

	console.log(`Prettify as below\n jq --tab --sort-keys . ${out} >pretty && mv pretty ${out}`)
}

function copyAll(soclist, soc, match, attrib) {

	soclist.forEach(s => {
		if (s == soc)
			return;

		var inp = `./data/${s}/HostNames.json`
		var out = `./data/${soc}/HostNames.json`
		copy_attributes(inp, out, match, attrib)
	});
}

//copyAll(["j721e"], "j7200", "protected_inst", "name");
//copyAll(["j721e"], "j7200", "hostName", "displayName");
