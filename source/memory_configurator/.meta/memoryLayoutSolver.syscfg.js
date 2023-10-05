const device = system.deviceData.device
const physicalLayout = system.getScript("/memory_configurator/physicalLayout.json")[device];
let common = system.getScript("/common");
let selfCoreName = common.getSelfSysCfgCoreName();

/**
 * Finds all memory regions across all contexts and returns an object where the keys are the type of memory region and
 * the values are a sorted array of the module instances.  The instances are sorted so that those with a manually
 * entered start address are first, and then sorted such that those requesting larger sizes are first.  The sorting is
 * done in this order to enable efficient placement when laying out regions:
 * 1) Manually entered start addresses must be first as they will block out those regions
 * 2) Larger regions must be first so that a smaller region doesn't use up a piece of memory inefficiently
 *
 * For example, the returned object will look something like this:
 * 	{
 * 		OCRAM: [{
 * 			context: "r5fss0-0",
 * 			auto: true,
 * 			start: 0x8000,
 * 			size: 0x50,
 * 			inst: <module instance>
 * 		}, {
 * 			context: "r5fss0-1",
 * 			auto: false,
 * 			start: -1,
 * 			size: 0x100,
 * 			inst: <module instance>
 * 		}, {
 * 			context: "r5fss0-1",
 * 			auto: false,
 * 			start: -1,
 * 			size: 0x50,
 * 			inst: <module instance>
 * 		}],
 * 		Flash: [] // etc
 */
let moduleName = ""

function isOptiShare(device){

    if(device == "AM263x_beta" || device == "AM263Px")
        return true;
    else if(device == "AM273x" || device == "AM243x" || device == "AM64x")
        return false;
}

function sortAndGroupRegions() {
    let region_module = system.modules['/memory_configurator/region'];

    if(region_module !== undefined) {
        if(region_module.$instances[0].mpu_setting)
            moduleName = "/memory_configurator/memory_region_mpu";
        else
            moduleName = "/memory_configurator/memory_region";
    }
	return _.chain(system.contexts)
		.flatMap((context) => {																			// For each context
			return _.map(context.system.modules?.[moduleName]?.$instances, (inst) => {  // Map every region instance as the following object

                let obj = {
                    context: "",
                    auto: "",
                    start: "",
                    size: "",
                    isShared: "",
                    shared_cores: "",
                }

                if ( physicalLayout[inst.type].access == "all" || (context.system.context).includes(selfCoreName)) {

                    let size = inst.size;

                    if(moduleName === "/memory_configurator/memory_region_mpu") {
                        size = Math.pow(2,inst.size);
                    }

                    obj.context = context.system.context
                    obj.auto = inst.auto
                    obj.start = inst.manualStartAddress
                    obj.size = size
                    obj.isShared = inst.isShared
                    obj.shared_cores = inst.shared_cores

                    obj = {
                        ...obj,
                        inst
                    }
                    return obj;
                }
			});
		})
        .filter((obj) => obj !== undefined) // Filter out null or undefined objects
		.sortBy((region) => `${region.auto}_${(0xFFFFFFFF - region.size).toString(16).padStart(8)}`)		// Sort first by manual then auto, then biggest to smallest
		.groupBy((region) => region.inst.type)
		.value();
}

/**
 * This function takes the output of sortAndGroupRegions() and places all the regions so they don't overlap.  If a
 * region cannot be placed, its start address is set to -1.  The returned object has keys for each memory type and
 * values being an array of the placed regions along with remaining holes in memory (indicated by a null inst value).
 * For example:
 * 	{
 * 		OCRAM: [{
 * 			context: "r5fss0-0",
 * 			auto: true,
 * 			start: 0x8000,
 * 			size: 0x50,
 * 			inst: <module instance>
 * 		}, {
 * 			start: 0x8050,		// hole
 * 			size: 0x100,
 * 		}, {
 * 			context: "r5fss0-1",
 * 			auto: false,
 * 			start: 8150,
 * 			size: 0x50,
 * 			inst: <module instance>
 * 		}],
 * 		Flash: [] // etc
 */
function layoutMemory(groupedRegions) {

	// physicalLayout consists of an object with the keys being the memory type and a value being start/size.  By
	// mapping just the values, we'll end up with an object containing assigned regions and holes for every memory type
	// even if there's no regions on that type yet.

	return _.mapValues(physicalLayout, (memoryInfo, type) => {  // for each (memoryInfo --> value for type --> key) pair

		// Initialize the layout with one unassigned hole, then iterate over the regions placing each one

		let layout = [memoryInfo];  //convert memoryInfo object into type array. memoryInfo = size: x start: y, layout = 0: [{size: x , start: y, access: "all"}]
        let regions = groupedRegions[type]

        if((type.toString()).includes("CUSTOM")) {
            //layout.splice(0,layout.length);
            let block_regions = JSON.parse(JSON.stringify(physicalLayout));

            _.chain(block_regions)
            .sortBy((memory_type) => memory_type.start)
            .flatMap(memory_type => {
                if((memory_type.toString()).includes("CUSTOM") == -1) {
                    memory_type.inst = true
                }
            })
            .value();

            delete block_regions.type;

            _.each(block_regions, (region, type) => {
                    const hole = _.find(layout, (h) => h.start <= region.start && h.start + h.size >= region.start + region.size && !h.inst)

                    if(hole) {
                        _.pull(layout, hole);
                        layout.push({ start: hole.start, size: region.start - hole.start });
					    layout.push({ start: region.start + region.size, size: hole.size - region.size - (region.start - hole.start) });
                    }
            })
            _.reject(layout, (entry) => entry.size === 0 && !entry.inst);
        }

		_.each(regions, (region) => {
			if (!region.auto) {

				// If the region is manually placed, there can only be one possible place to put it.  Look for a hole
				// that's big enough, and insert it there

				const hole = _.find(layout, (h) => h.start <= region.start && h.start + h.size >= region.start + region.size && !h.inst);
				if (hole) {

					// Found a hole to place it.  Remove that hole, create new holes on either side, then insert this
					// region between them

					_.pull(layout, hole);
					layout.push({ start: hole.start, size: region.start - hole.start });
					layout.push({ start: region.start + region.size, size: hole.size - region.size - (region.start - hole.start) });
				} else {
					// This region doesn't fit.  Set start to -1 to indicate that
					region.start = -1;
				}
			} else {

				// If the region is auto placed, find the smallest hole that it could fit, as we want to use the memory
				// as efficiently as possible

				// const hole = _.chain(layout)
				// 	.sortBy("size")									// Sort by size first, so we find the biggest first
				// 	.find((h) => h.size >= region.size && !h.inst)	// Then find the first one that works
				// 	.value();

                const hole = _.chain(layout)
                .sortBy("size")									// Sort by size first, so we find the biggest first
                .find((h) => {
                    if((!region.isShared && moduleName === "/memory_configurator/memory_region") || !isOptiShare(device)){
                        return (h.size >= region.size && !h.inst)
                    }
                    else {  // For opti-shared enabled devices, the start address of the shared regions should be multiple of region's size
                        let original_start = h.start;
                        let shifted_start = Math.ceil(h.start/region.size)*(region.size)
                        let shift = shifted_start - original_start
                        return (h.size >= (shift + region.size) && !h.inst)
                    }
                })	// Then find the first one that works
                .value();

				if (hole) {

					// Found a hole to place it.  Remove that hole, insert the region, and add a new hole for whatever
					// is left

					_.pull(layout, hole);
                    if((!region.isShared && moduleName === "/memory_configurator/memory_region") || !isOptiShare(device)){
                        region.start = hole.start;
                    }
                    else { // For opti-shared enabled devices, the start address of the shared regions should be multiple of region's size
                        region.start = Math.ceil(hole.start/region.size)*(region.size)
                    }
					layout.push({ start: region.start + region.size, size: hole.size - region.size });
				} else {

					// This region doesn't fit.  Set start to -1 to indicate that

					region.start = -1;
				}
			}

			// Finally, push the region, whatever was done with it

			layout.push(region);
		});

		// Return the layout with all zero sized holes removed

		return _.reject(layout, (entry) => entry.size === 0 && !entry.inst);
	});
}

/**
 * This is used by getValue() and validate() in order to determine calculated values and where errors should be.  It
 * takes the output from layoutMemory() and groups it into a map of context names to instance names to the result
 */
function getMemoryLayout() {
	const layout = layoutMemory(sortAndGroupRegions());

	const answer = {};
	_.each(layout, (regions) => {
		_.each(regions, (region) => {
			if (region.inst) {
				_.set(answer, [region.context, region.inst.$name], region.start);
			}
		});
	});
	return answer;
}

/**
 * This is used by the custom view in order to display regions in order.  It takes the output from layoutMemory() and
 * groups it into a map of memory types to sorted regions and holes in those regions
 */
function getMemorySummary() {
	const layout = layoutMemory(sortAndGroupRegions());

	return _.mapValues(layout, (regions) => _.sortBy(regions, "start"));
}

exports = {
	getMemoryLayout,
	getMemorySummary,
}