exports = {
	displayName: "Firewall Region",
	config: [
		{
			name: "regionAlloc",
			displayName: "Define custom region",
			default: false,
			onChange: (inst, ui) => {
				if (inst.regionAlloc) {
					ui.addrStart.readOnly = false;
					ui.addrEnd.readOnly = false;
				} else {
					inst.addrStart = inst.defaultStart;
					inst.addrEnd = inst.defaultEnd;
					ui.addrStart.readOnly = true;
					ui.addrEnd.readOnly = true;
				}
			},
		},
		{
			name: "addrStart",
			displayName: "Region start address",
			default: "0",
			readOnly: true,
		},
		{
			name: "addrEnd",
			displayName: "Region end address",
			default: "0",
			readOnly: true,
		},
		{
			name: "defaultStart",
			default: "0",
			hidden: true,
			onChange: (inst, ui) => {
				inst.addrStart = inst.defaultStart;
				ui.addrStart.readOnly = true;
			},
		},
		{
			name: "defaultEnd",
			default: "0",
			hidden: true,
			onChange: (inst, ui) => {
				inst.addrEnd = inst.defaultEnd;
				ui.addrEnd.readOnly = true;
			},
		},
		{
			name: "memory",
			default: false,
			hidden: true,
			onChange: (inst, ui) => {
				if (inst.memory) {
					ui.regionAlloc.hidden = true;
				} else {
					ui.regionAlloc.hidden = false;
				}
			},
		},
		{
			name: "lock",
			displayName: "Lock Region",
			default: false,
		},
		{
			name: "background",
			displayName: "Mark as Background Region",
			default: false,
		},
	],
	moduleInstances: (inst) => {
		return [
			{
				name: "perms",
				displayName: "Permission Slot",
				moduleName: "/modules/firewallPermissions",
				minInstanceCount: 1,
				maxInstanceCount: 3,
				useArray: true,
				collapsed: false,
			},
		];
	},
	validate: (inst, report) => {
		if (inst.regionAlloc) {
			var startAdd = inst.addrStart;
			var endAdd = inst.addrEnd;
			startAdd = parseInt(startAdd);
			endAdd = parseInt(endAdd);

			if (endAdd < startAdd) {
				report.logError("End Address must be greater than Start Address", inst, "addrEnd");
			}

			if (startAdd % 0x1000) {
				report.logError("Start Address is not aligned on 4K boundary", inst, "addrStart");
			}

			if (endAdd % 0x1000) {
				report.logError("End Address is not aligned on 4K boundary", inst, "addrEnd");
			}
		}
	},
};
