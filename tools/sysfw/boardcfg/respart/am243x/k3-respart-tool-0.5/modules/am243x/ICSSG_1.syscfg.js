
const {createHostModule} = system.getScript("/modules/sysfwResPart.js");
const hostInfo = {
  "Description": "ICSSG context 1 on Main island",
  "Security": "Non Secure",
  "displayName": "ICSSG_1",
  "hostId": 51,
  "hostName": "ICSSG_1",
  "privId": 137
};
const modDef = createHostModule(hostInfo);
exports = modDef;
