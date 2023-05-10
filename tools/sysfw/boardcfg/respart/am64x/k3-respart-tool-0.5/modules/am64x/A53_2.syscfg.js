
const {createHostModule} = system.getScript("/modules/sysfwResPart.js");
const hostInfo = {
  "Description": "Cortex A53 context 2 on Main island",
  "Security": "Non Secure",
  "displayName": "A53_2 NonSecure host",
  "hostId": 12,
  "hostName": "A53_2",
  "privId": 1
};
const modDef = createHostModule(hostInfo);
exports = modDef;
