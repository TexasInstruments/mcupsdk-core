var utils = system.getScript("/scripts/utils.js");
var hosts = utils.hosts;
const socName = utils.socName;
var hostNames =  utils.hostNames;
var reslist = utils.reslist;
var moduleInstances = [];


exports = {
	displayName: "Resource Sharing",
	config: [
		{
			name: "resourceName",
			displayName: "Select Resource to be shared",
			options: [
				{
					name: "none",
					displayName: "None",
				},
				...reslist,
			],
			default: "none",
		},
		{
			name: "sharedFromHostName",
			displayName: "Share From Host",
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
			name: "sharedToHostName",
			displayName: "To The Host",
			options: [
				{
					name: "none",
					displayName: "None",
				},
				...hostNames,
			],
			default: "none",
		},
	],
	validate: (inst, report) => {
		moduleInstances = utils.getModuleInstances();
		duplicateHostAndShareHost(inst, report);
		checkResourceSharedMoreThanOnce(inst, report);
		checkShareFromHostResourceCount(inst, report);
		checkShareToHostResourceCount(inst, report);
		console.log(`Validating now.. `)
	},
};



// Check whether a resource belonging to one host is shared between more than one host
function checkResourceSharedMoreThanOnce(instance, report) {
	var sharedResourceList = utils.getSharedResourceList();
	for (let i of sharedResourceList ) {
		if(i != instance){
			if (instance.resourceName != "none" && i.sharedFromHostName == instance.sharedFromHostName &&  i.resourceName == instance.resourceName ) {
				report.logError(instance.resourceName + " of " +  instance.sharedFromHostName + " shared with more than one host", instance,"sharedFromHostName");
			}
		}
	}
}

// Check if same host is selected as shared from host and shared to host
function duplicateHostAndShareHost(instance, report) {
	if (instance.sharedToHostName != "none" && instance.sharedToHostName === instance.sharedFromHostName) {
		report.logError("Shared From and To host name cannot be the same ", instance,"sharedFromHostName");
	}
}

// check whether the share from host have a non zero resource count
function checkShareFromHostResourceCount(instance, report) {
	if(instance.resourceName != "none"){
		var name = _.join(_.split(instance.resourceName, " "), "_");
		var resourceCountZero = true
		for (let module of moduleInstances ) {
			if (instance.sharedFromHostName === module.hostName ) {
				if(module[name + '_count'] && module[name + '_count'] > 0){
					resourceCountZero = false;
				}
			}
		}
		if(instance.sharedFromHostName != "none" && resourceCountZero){
			report.logError(instance.resourceName + " count is zero for the host " + instance.sharedFromHostName, instance, "sharedFromHostName");
		}
	}
}

// check whether the share to host have the resource count zero
function checkShareToHostResourceCount(instance, report) {
	if(instance.resourceName != "none"){
		var name = _.join(_.split(instance.resourceName, " "), "_");
		for (let module of moduleInstances ) {
			if (instance.sharedToHostName === module.hostName ) {
				if(module[name + '_count'] != 0)
					report.logError(instance.resourceName + " count is not zero for the host " + module.hostName, instance, "sharedToHostName");
			}
		}
	}
}
