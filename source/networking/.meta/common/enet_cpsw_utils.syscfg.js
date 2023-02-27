"use strict";

/*
 *  ======== getGroupByName ========
 *  Get a list of groups and a group name.
 *  Returns the group config array.
 *
 *
 *  @param groupList   - List of groups
 *  @param groupName   - The name of the group to return
 */

function getGroupByName(groupList, groupName)
{
  for(let i = 0; i < groupList.length; i++)
  {
    if(groupList[i].name == groupName)
    {
      return groupList[i].config;
    }
  }
}

function getGroupHierarchyByName(groupList, groupHierarchy)
{
  let groupNameList = groupHierarchy.split("/");
  let groupHead = groupList;

  for(let i = 0; i < groupNameList.length; i++)
  {
      groupHead = getGroupByName(groupHead, groupNameList[i]);
  }
  return groupHead;
}


/*
 *  ======== hideGroup ========
 *  Hide or UnHide an entire group
 *
 *  @param group   - The group config
 *  @param toHide  - True(Hide)/false(UnHide)
 *  @param ui      - The User Interface object
 */
function hideGroup(group, toHide, ui)
{
  let namesArray = getNameListToHide(group);
  _.each(namesArray, (cfg) => { if (ui.hasOwnProperty(cfg)) {ui[cfg].hidden = toHide;}});
}


/*
 *  ======== getNameListToHide ========
 *  Hide or UnHide an entire group
 *
 *  @param group   - The group config
 *  @param toHide  - True(Hide)/false(UnHide)
 *  @param ui      - The User Interface object
 */
function getNameListToHide(group)
{
  let namesArray = _.map(group, function (n)
                         {
                             if (n.hasOwnProperty("config"))
                             {
                                 return (getNameListToHide(n.config));
                             }
                             else
                             {
                                 return n.name;
                             }
                         }
                        );
  return namesArray.flat(1);
}

function updatePortConfig(updatedConfig, fromProperty, toProperty) {
    for (var member in updatedConfig)
    {
        if (typeof updatedConfig[member] == "object" && updatedConfig[member] !== null)
        {
            updatePortConfig(updatedConfig[member], fromProperty, toProperty);
        }
        else
        {
            if (typeof(updatedConfig[member]) === 'string')
            {
                updatedConfig[member] = updatedConfig[member].replace(fromProperty, toProperty);
            }
        }
    }
    return updatedConfig;
}

function getPortSpecificConfig(configObj, fromProperty, toProperty) {
    let updatedConfig = _.cloneDeep(configObj)

    updatedConfig = updatePortConfig(updatedConfig, fromProperty,toProperty);
    return updatedConfig;
}

function getPhyStatePollFreqHz(mdioBusFreqHz, mdioIPGRatio) {

    return ((Math.floor)(mdioBusFreqHz/mdioIPGRatio));
}

function getArraytoBitMask(configObj) {
    let bitmask = 0;

    for (var elem in configObj)
    {
        bitmask = (bitmask | (1 << parseInt(elem)));
    }
    return bitmask.toString();
}

exports =
{
    getGroupByName: getGroupByName,
    getGroupHierarchyByName: getGroupHierarchyByName,
    hideGroup: hideGroup,
    getNameListToHide: getNameListToHide,
    updatePortConfig: updatePortConfig,
    getPortSpecificConfig: getPortSpecificConfig,
    getArraytoBitMask: getArraytoBitMask,
    getPhyStatePollFreqHz: getPhyStatePollFreqHz,
};
