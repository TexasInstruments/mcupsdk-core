let common = system.getScript("/common");
let firewall_config = system.getScript(`/drivers/firewall/soc/firewall_am64x_am243x.json`);

const firewall_master_config = [
    {
        // master information for everyone
        name: "ALLOW_EVERYONE",
        privid: 195,
        secure: true,
        nsecure: true,
        priv: true,
        user: true,
    },
    {
        // master information for block_everyone
        name: "BLOCK_EVERYONE",
        privid: 197,
        secure: true,
        nsecure: true,
        priv: true,
        user: true,
    },
    {
        // master information for a53_non_secure_supervisor
        name: "A53_NON_SECURE",
        privid: 1,
        secure: false,
        nsecure: true,
        priv: true,
        user: false,
    },
    {
        // master information for a53_secure_supervisor
        name: "A53_SECURE",
        privid: 1,
        secure: true,
        nsecure: false,
        priv: true,
        user: false,
    },
    {
        // master information for m4_0
        name: "M4_0_NON_SECURE",
        privid: 100,
        secure: false,
        nsecure: true,
        priv: true,
        user: true,
    },
    {
        // master information for main_0_r5_0_nonsecure
        name: "MAIN_0_R5_0_NONSECURE",
        privid: 212,
        secure: false,
        nsecure: true,
        priv: true,
        user: true,
    },
    {
        // master information for main_0_r5_0_secure
        name: "MAIN_0_R5_0_SECURE",
        privid: 212,
        secure: true,
        nsecure: false,
        priv: true,
        user: true,
    },
    {
        // master information for main_0_r5_1_nonsecure
        name: "MAIN_0_R5_1_NONSECURE",
        privid: 213,
        secure: false,
        nsecure: true,
        priv: true,
        user: true,
    },
    {
        // master information for main_0_r5_1_secure
        name: "MAIN_0_R5_1_SECURE",
        privid: 213,
        secure: true,
        nsecure: false,
        priv: true,
        user: true,
    },
    {
        // master information for main_1_r5_0_nonsecure
        name: "MAIN_1_R5_0_NONSECURE",
        privid: 214,
        secure: false,
        nsecure: true,
        priv: true,
        user: true,
    },
    {
        // master information for main_1_r5_0_secure
        name: "MAIN_1_R5_0_SECURE",
        privid: 214,
        secure: true,
        nsecure: false,
        priv: true,
        user: true,
    },
    {
        // master information for main_1_r5_1_nonsecure
        name: "MAIN_1_R5_1_NONSECURE",
        privid: 215,
        secure: false,
        nsecure: true,
        priv: true,
        user: true,
    },
    {
        // master information for main_1_r5_1_secure
        name: "MAIN_1_R5_1_SECURE",
        privid: 215,
        secure: true,
        nsecure: false,
        priv: true,
        user: true,
    },
    {
        // master information for main_0_icssg_0
        name: "MAIN_0_ICSSG_0",
        privid: 136,
        secure: false,
        nsecure: true,
        priv: true,
        user: true,
    },
];

function getMasterConfigArr() {
    return firewall_master_config;
}

function getFirewallConfig(){
    return firewall_config;
}

exports = {
    getMasterConfigArr,
    getFirewallConfig,
};


