function getComponentProperty(device)
{
    return require(`./project_${device}_hsr_prp_icss_emac_mii`).getComponentProperty();
};

function getComponentBuildProperty(buildOption)
{
    return require(`./project_${buildOption.device}_hsr_prp_icss_emac_mii`).getComponentBuildProperty(buildOption);
};

module.exports = {
    getComponentProperty,
    getComponentBuildProperty,
};
