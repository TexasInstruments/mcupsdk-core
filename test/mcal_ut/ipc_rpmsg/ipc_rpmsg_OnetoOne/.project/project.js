function getComponentProperty(device)
{
    return require(`./project_${device}`).getComponentProperty();
};

function getComponentBuildProperty(buildOption)
{
    return require(`./project_${buildOption.device}`).getComponentBuildProperty(buildOption);
};

function getSystemProjects(device)
{
    return require(`./project_${device}`).getSystemProjects(device);
};

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
    getSystemProjects,
};
