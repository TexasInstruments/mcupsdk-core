
const {createHostModule} = system.getScript("/modules/sysfwResPart.js");
const hostInfo = {
  "Description": "Cortex R5_1 context 3 on Main island",
  "Security": "Non Secure",
  "displayName": "Main R5F1 core1 NonSecure host",
  "hostId": 43,
  "hostName": "MAIN_1_R5_3",
  "privId": 215
};
const modDef = createHostModule(hostInfo);
exports = modDef;
