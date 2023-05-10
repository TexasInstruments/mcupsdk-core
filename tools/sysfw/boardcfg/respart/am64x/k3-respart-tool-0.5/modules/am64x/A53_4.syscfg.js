
const {createHostModule} = system.getScript("/modules/sysfwResPart.js");
const hostInfo = {
  "Description": "Cortex A53 context 1 on Main island",
  "Security": "Non Secure",
  "displayName": "A53_4 NonSecure host",
  "hostId": 14,
  "hostName": "A53_4",
  "privId": 1
};
const modDef = createHostModule(hostInfo);
exports = modDef;
