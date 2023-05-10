
const {createHostModule} = system.getScript("/modules/sysfwResPart.js");
const hostInfo = {
  "Description": "Cortex R5_0 context 1 on Main island",
  "Security": "Non Secure",
  "displayName": "Main R5F0 core0 NonSecure host",
  "hostId": 36,
  "hostName": "MAIN_0_R5_1",
  "privId": 212
};
const modDef = createHostModule(hostInfo);
exports = modDef;
