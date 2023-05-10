
const {createHostModule} = system.getScript("/modules/sysfwResPart.js");
const hostInfo = {
  "Description": "ICSSG context 0 on Main island",
  "Security": "Non Secure",
  "displayName": "ICSSG_0",
  "hostId": 50,
  "hostName": "ICSSG_0",
  "privId": 136
};
const modDef = createHostModule(hostInfo);
exports = modDef;
