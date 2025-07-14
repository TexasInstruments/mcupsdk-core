function getComponentProperty(device)
{
    return require(`./project_${device}`).getComponentProperty();
};

function getComponentBuildProperty(buildOption)
{
    return require(`./project_${buildOption.device}`).getComponentBuildProperty(buildOption);
};

function getProperty(device)
{
    return require(`./project_${device}`).getProperty();
};

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
    getProperty
};
