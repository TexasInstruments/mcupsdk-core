
const {createHostModule} = system.getScript("/modules/sysfwResPart.js");
const hostInfo = {
  "Description": "Cortex R5_1 context 1 on Main island",
  "Security": "Non Secure",
  "displayName": "Main R5F1 core0 NonSecure host",
  "hostId": 41,
  "hostName": "MAIN_1_R5_1",
  "privId": 214
};
const modDef = createHostModule(hostInfo);
exports = modDef;
