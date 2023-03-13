function getComponentProperty(device)
{
    return require(`./project_${device}_contrib`).getComponentProperty();
};

function getComponentBuildProperty(buildOption)
{
    return require(`./project_${buildOption.device}_contrib`).getComponentBuildProperty(buildOption);
};

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
