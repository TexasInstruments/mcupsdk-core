let common = system.getScript("/common");

let mmu_armv8_module = {
    displayName: "MMU ARMv8",
    maxInstances: 16,
	templates: {
        "/kernel/dpl/dpl_config.c.xdt": {
			dpl_config: "/kernel/dpl/mmu_armv8.c.xdt",
			dpl_init: "/kernel/dpl/mmu_armv8_init.c.xdt",
		},
		"/kernel/dpl/dpl_config.h.xdt": "/kernel/dpl/mmu_armv8.h.xdt"
	},
    defaultInstanceName: "CONFIG_MMU_REGION",
    config: [
		{
			name: "vAddr",
            displayName: "Region Start Virtual Address (hex)",
            description: "MUST be <= 48 bits and MUST be region size aligned",
            default: 0x0,
            displayFormat: "hex",
        },
		{
			name: "pAddr",
            displayName: "Region Start Physical Address (hex)",
            description: "MUST be <= 48 bits and MUST be region size aligned",
            default: 0x0,
            displayFormat: "hex",
        },
		{
			name: "size",
			displayName: "Region Size (bytes)",
            description: "MUST be 4KB aligned",
			default: 0x0,
			displayFormat: "hex",
		},
		{
            name: "accessPermissions",
            displayName: "Access Permissions",
            default: "Privilege RW, User None",
			options: [
				{
					"name": "Privilege RW, User None"
				},
				{
					"name": "Privilege RW, User RW"
				},
				{
					"name": "Privilege RO, User None"
				},
				{
					"name": "Privilege RO, User RO"
				},
			]
		},
		{
            name: "privExecute",
            displayName: "Allow Privileged Execution",
            default: true,
        },
		{
			name: "userExecute",
			displayName: "Allow User Execution",
			default: false,
		},
		{
			name: "shareable",
			displayName: "Sharable",
			default: "Outer Shareable",
			options: [
				{
					"name": "Non Shareable"
				},
				{
					"name": "Outer Shareable"
				},
				{
					"name": "Inner Shareable"
				},
			],
		},
		{
			name: "attribute",
			displayName: "Attribute",
			default: "MAIR0",
			options: [
				{
                    name: "MAIR0",
					displayName: "Attr Index 0: Non-gathering, non-reordering and no early write acknowledgement",
				},
				{
                    name: "MAIR1",
					displayName: "Attr Index 1: Non-gathering, non-reordering and early write acknowledgement"
				},
				{
                    name: "MAIR2",
					displayName: "Attr Index 2: Non-gathering, reordering and early write acknowledgement"
				},
				{
                    name: "MAIR3",
					displayName: "Attr Index 3: Gathering, reordering and early write acknowledgement"
				},
				{
                    name: "MAIR4",
					displayName: "Attr Index 4: Normal inner & outer non-cacheable"
				},
				{
                    name: "MAIR5",
					displayName: "Attr Index 5: Normal outer non-cacheable, inner write-back cacheable non-transient memory"
				},
				{
                    name: "MAIR6",
					displayName: "Attr Index 6: Normal outer & inner write-through cacheable non-transient memory"
				},
				{
                    name: "MAIR7",
					displayName: "Attr Index 7: Normal outer and inner write-back cacheable non-transient memory"
				},
			],
		},
		{
			name: "global",
			displayName: "Global",
			default: true
		}
    ],

	validate: function (instance, report) {
		let vAddr = instance.vAddr;
		let pAddr = instance.pAddr;
		let size = instance.size;
		let granule_size = 0x1000;
		let maxAddr = 0x0000FFFFFFFFFFFF;

		if ((vAddr % granule_size) != 0)
		{
			report.logError( `Virtual address must be aligned to 0x${granule_size.toString(16).toUpperCase()} B`,
                    instance, "vAddr");
		}
		if ((pAddr % granule_size) != 0)
		{
			report.logError( `Physical address must be aligned to 0x${granule_size.toString(16).toUpperCase()} B`,
                    instance, "pAddr");
		}
		if ((size % granule_size) != 0)
		{
			report.logError( `Size must be aligned to 0x${granule_size.toString(16).toUpperCase()} B`,
                    instance, "size");
		}
        common.validate.checkNumberRange(instance, report, "vAddr", 0, maxAddr, "hex");
        common.validate.checkNumberRange(instance, report, "pAddr", 0, maxAddr, "hex");
        common.validate.checkNumberRange(instance, report, "size", 0, maxAddr, "hex");

		if (( (vAddr+size) > (maxAddr+1)) || ((pAddr+size) > (maxAddr+1)))
		{
			report.logError( `Virtual and physical address range must be <= 0x${(maxAddr-1).toString(16).toUpperCase()}`,
                    instance, "size");
		}
	},
	moduleStatic: {
		modules: function(inst) {
			return [{
				name: "system_common",
				moduleName: "/system_common",
			}]
		},
    },
	getRegionAttributes(instance) {

		let attributes = {};

		switch(instance.accessPermissions) {
			default:
			case "Privilege RW, User None":
				attributes.accessPerm = "MMUP_ACCESS_PERM_PRIV_RW_USER_NONE";
				break;
			case "Privilege RW, User RW":
				attributes.accessPerm = "MMUP_ACCESS_PERM_PRIV_RW_USER_RW";
				break;
			case "Privilege RO, User None":
				attributes.accessPerm = "MMUP_ACCESS_PERM_PRIV_RO_USER_NONE";
				break;
			case "Privilege RO, User RO":
				attributes.accessPerm = "MMUP_ACCESS_PERM_PRIV_RO_USER_RO";
				break;
		}

		switch(instance.shareable) {
			default:
			case "Outer Shareable":
				attributes.shareable = "MMUP_SHARABLE_OUTER";
				break;
			case "Non Shareable":
				attributes.shareable = "MMUP_SHARABLE_NONE";
				break;
			case "Inner Shareable":
				attributes.shareable = "MMUP_SHARABLE_INNER";
				break;
		}

		switch (instance.attribute) {
			default:
			case "MAIR0":
				attributes.attrIndx = "MMUP_ATTRINDX_MAIR0";
				break;
			case "MAIR1":
				attributes.attrIndx = "MMUP_ATTRINDX_MAIR1";
				break;
			case "MAIR2":
				attributes.attrIndx = "MMUP_ATTRINDX_MAIR2";
				break;
			case "MAIR3":
				attributes.attrIndx = "MMUP_ATTRINDX_MAIR3";
				break;
			case "MAIR4":
				attributes.attrIndx = "MMUP_ATTRINDX_MAIR4";
				break;
			case "MAIR5":
				attributes.attrIndx = "MMUP_ATTRINDX_MAIR5";
				break;
			case "MAIR6":
				attributes.attrIndx = "MMUP_ATTRINDX_MAIR6";
				break;
			case "MAIR7":
				attributes.attrIndx = "MMUP_ATTRINDX_MAIR7";
				break;
		}
        attributes.privExecute = Number(instance.privExecute);
        attributes.userExecute = Number(instance.userExecute);
        attributes.global = Number(instance.global);

		return attributes;
	}

};

exports = mmu_armv8_module;