
const {createHostModule} = system.getScript("/modules/sysfwResPart.js");
const hostInfo = {
  "Description": "Cortex R5_0 context 3 on Main island",
  "Security": "Non Secure",
  "displayName": "Main R5F0 core1 NonSecure host",
  "hostId": 38,
  "hostName": "MAIN_0_R5_3",
  "privId": 213
};
const modDef = createHostModule(hostInfo);
exports = modDef;
