function getComponentProperty(device)
{
    return require(`./project_${device}_emac_lwip_if`).getComponentProperty();
};

function getComponentBuildProperty(buildOption)
{
    return require(`./project_${buildOption.device}_emac_lwip_if`).getComponentBuildProperty(buildOption);
};

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
