
/* This file exports modules that are mandatory and always needed in any application. 
 * All modules just need to include this module and it will pull in all mandatory modules.
 */
let common = system.getScript("/common");
let soc = system.getScript(`/system_${common.getSocName()}`);

exports = soc;