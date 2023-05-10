
const {createHostModule} = system.getScript("/modules/sysfwResPart.js");
const hostInfo = {
  "Description": "Cortex A53 context 3 on Main island",
  "Security": "Non Secure",
  "displayName": "A53_3 NonSecure host",
  "hostId": 13,
  "hostName": "A53_3",
  "privId": 1
};
const modDef = createHostModule(hostInfo);
exports = modDef;
