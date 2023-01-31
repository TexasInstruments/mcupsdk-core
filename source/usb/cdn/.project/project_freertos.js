function getComponentProperty(device)
{
    return require(`./project_freertos_${device}`).getComponentProperty();
};

function getComponentBuildProperty(buildOption)
{
    return require(`./project_freertos_${buildOption.device}`).getComponentBuildProperty(buildOption);
};

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
