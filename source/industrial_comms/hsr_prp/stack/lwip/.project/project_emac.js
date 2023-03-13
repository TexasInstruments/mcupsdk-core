function getComponentProperty(device)
{
    return require(`./project_${device}_emac`).getComponentProperty();
};

function getComponentBuildProperty(buildOption)
{
    return require(`./project_${buildOption.device}_emac`).getComponentBuildProperty(buildOption);
};

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
