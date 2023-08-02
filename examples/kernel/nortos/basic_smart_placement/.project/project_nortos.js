function getComponentProperty(device)
{
    return require(`./project_nortos_${device}`).getComponentProperty();
};

function getComponentBuildProperty(buildOption)
{
    return require(`./project_nortos_${buildOption.device}`).getComponentBuildProperty(buildOption);
};

function getSystemProjects(device)
{
    return require(`./project_nortos_${device}`).getSystemProjects(device);
};

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
    getSystemProjects,
};
