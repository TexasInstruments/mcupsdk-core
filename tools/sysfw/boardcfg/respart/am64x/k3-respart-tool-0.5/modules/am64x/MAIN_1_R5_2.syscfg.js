
const {createHostModule} = system.getScript("/modules/sysfwResPart.js");
const hostInfo = {
  "Description": "Cortex R5_1 context 2 on Main island",
  "Security": "Secure",
  "displayName": "Main R5F1 core1 Secure host",
  "hostId": 42,
  "hostName": "MAIN_1_R5_2",
  "privId": 215
};
const modDef = createHostModule(hostInfo);
exports = modDef;
