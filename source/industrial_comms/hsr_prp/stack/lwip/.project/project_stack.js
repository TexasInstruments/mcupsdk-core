function getComponentProperty(device)
{
    return require(`./project_${device}_stack`).getComponentProperty();
};

function getComponentBuildProperty(buildOption)
{
    return require(`./project_${buildOption.device}_stack`).getComponentBuildProperty(buildOption);
};

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
