function getComponentProperty(device)
{
    return require(`./project_${device}_rt_mrp_rgmii`).getComponentProperty();
};

function getComponentBuildProperty(buildOption)
{
    return require(`./project_${buildOption.device}_rt_mrp_rgmii`).getComponentBuildProperty(buildOption);
};

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
