let common   = system.getScript("/common");
let device_peripheral = system.getScript(`/drivers/epwm/soc/epwm_${common.getSocName()}.syscfg.js`);


function calculatePeriodCoarseSteps(inst, ui)
{
    var clockDiv = parseInt(inst.epwmTimebase_clockDiv.replace("EPWM_CLOCK_DIVIDER_", ""))
    var HSPclockDiv = parseInt(inst.epwmTimebase_hsClockDiv.replace("EPWM_HSCLOCK_DIVIDER_", ""))
    var TTBCLK = 1/((inst.hrpwm_epwmclk * Math.pow(10,6))/(clockDiv * HSPclockDiv))
    var PWMPeriod = (inst.hrpwm_period* Math.pow(10,-9)) / TTBCLK;
    return PWMPeriod;
}

function calculateTBCLK(inst,ui)
{
    var clockDiv = parseInt(inst.epwmTimebase_clockDiv.replace("EPWM_CLOCK_DIVIDER_", ""))
    var HSPclockDiv = parseInt(inst.epwmTimebase_hsClockDiv.replace("EPWM_HSCLOCK_DIVIDER_", ""))
    var TBCLK = inst.hrpwm_epwmclk/(clockDiv * HSPclockDiv)
    return TBCLK
}

function calculateCoarseSteps(inst,ui)
{
    //Get clock dividers
    var clockDiv = parseInt(inst.epwmTimebase_clockDiv.replace("EPWM_CLOCK_DIVIDER_", ""))
    var HSPclockDiv = parseInt(inst.epwmTimebase_hsClockDiv.replace("EPWM_HSCLOCK_DIVIDER_", ""))
    var TTBCLK = 1/((inst.hrpwm_epwmclk * Math.pow(10,6))/(clockDiv * HSPclockDiv))
    var PWMPeriod = (inst.hrpwm_period* Math.pow(10,-9)) / TTBCLK;
    var coarseStep = parseInt(inst.hrpwm_duty/100 * PWMPeriod);
    return coarseStep;
}

function calculateMEPSteps(inst,ui)
{
    var clockDiv = parseInt(inst.epwmTimebase_clockDiv.replace("EPWM_CLOCK_DIVIDER_", ""))
    var HSPclockDiv = parseInt(inst.epwmTimebase_hsClockDiv.replace("EPWM_HSCLOCK_DIVIDER_", ""))
    var TTBCLK = 1/((inst.hrpwm_epwmclk * Math.pow(10,6))/(clockDiv * HSPclockDiv))
    var MEPStep = parseInt(TTBCLK/(inst.hrpwm_scaleFactor* Math.pow(10,-12)));
    return MEPStep;
}

function calculateCMPX(inst,ui)
{
    return inst.hrpwm_coarseSteps;
}

function calculateCMPXHR(inst,ui)
{
    var clockDiv = parseInt(inst.epwmTimebase_clockDiv.replace("EPWM_CLOCK_DIVIDER_", ""))
    var HSPclockDiv = parseInt(inst.epwmTimebase_hsClockDiv.replace("EPWM_HSCLOCK_DIVIDER_", ""))
    var TTBCLK = 1/((inst.hrpwm_epwmclk * Math.pow(10,6))/(clockDiv * HSPclockDiv))
    var PWMPeriod = (inst.hrpwm_period* Math.pow(10,-9)) / TTBCLK;
    var CMPAHRVal = parseInt((((inst.hrpwm_duty/100 * PWMPeriod) % 1) * inst.hrpwm_mepSteps + .5) * 256);
    return CMPAHRVal;
}

function calculateTBPRD(inst, ui)
{
    var clockDiv = parseInt(inst.epwmTimebase_clockDiv.replace("EPWM_CLOCK_DIVIDER_", ""))
    var HSPclockDiv = parseInt(inst.epwmTimebase_hsClockDiv.replace("EPWM_HSCLOCK_DIVIDER_", ""))
    var Tpwm = inst.hrpwm_period * Math.pow(10,-9)
    var TTBCLK = 1/((inst.hrpwm_epwmclk * Math.pow(10,6))/(clockDiv * HSPclockDiv));

    if (inst.epwmTimebase_counterMode == "EPWM_COUNTER_MODE_UP")
    {
        var tbprd= Math.trunc((Tpwm/TTBCLK) -1);
    }
    else if (inst.epwmTimebase_counterMode == "EPWM_COUNTER_MODE_UP_DOWN")
    {
        var tbprd = Math.trunc(Tpwm / (2* TTBCLK));
    }
    else
    {
         var tbprd = 0;
    }
    return tbprd;
}

function calculateTBPRDHR(inst,ui)
{
    var clockDiv = parseInt(inst.epwmTimebase_clockDiv.replace("EPWM_CLOCK_DIVIDER_", ""))
    var HSPclockDiv = parseInt(inst.epwmTimebase_hsClockDiv.replace("EPWM_HSCLOCK_DIVIDER_", ""))
    var Tpwm = inst.hrpwm_period * Math.pow(10,-9)
    var TTBCLK = 1/((inst.hrpwm_epwmclk * Math.pow(10,6))/(clockDiv * HSPclockDiv));

    if (inst.epwmTimebase_counterMode == "EPWM_COUNTER_MODE_UP")
    {
        var tbprd= (Tpwm/TTBCLK) -1;
        if (inst.hrpwm_autoConv == true)
        {
            var tbprdhr= Math.trunc(((tbprd-Math.floor(tbprd)) * inst.hrpwm_mepSteps + .5) *256)
        }
        else
        {
            var tbprdhr= Math.trunc((tbprd-Math.floor(tbprd))*256)
        }

    }
    else if (inst.epwmTimebase_counterMode == "EPWM_COUNTER_MODE_UP_DOWN")
    {
        var tbprd = Tpwm / (2* TTBCLK)
        if (inst.hrpwm_autoConv == true)
        {
            var tbprdhr= Math.trunc(((tbprd-Math.floor(tbprd)) * inst.hrpwm_mepSteps + .5) *256)
        }
        else
        {
            var tbprdhr= Math.trunc((tbprd-Math.floor(tbprd))*256)
        }
    }
    else
    {
         var tbprdhr = 0;
    }
    return tbprdhr;
}

function calculateRED(inst,ui)
{
    var clockDiv = parseInt(inst.epwmTimebase_clockDiv.replace("EPWM_CLOCK_DIVIDER_", ""))
    var HSPclockDiv = parseInt(inst.epwmTimebase_hsClockDiv.replace("EPWM_HSCLOCK_DIVIDER_", ""))
    var TTBCLK = 1/((inst.hrpwm_epwmclk * Math.pow(10,6))/(clockDiv * HSPclockDiv));
    var red_delay = inst.hrpwm_requiredRED * Math.pow(10, -9)
    var red = Math.trunc(red_delay/(TTBCLK/2))
    return red
}

function calculateREDHR(inst,ui)
{
    var clockDiv = parseInt(inst.epwmTimebase_clockDiv.replace("EPWM_CLOCK_DIVIDER_", ""))
    var HSPclockDiv = parseInt(inst.epwmTimebase_hsClockDiv.replace("EPWM_HSCLOCK_DIVIDER_", ""))
    var TTBCLK = 1/((inst.hrpwm_epwmclk * Math.pow(10,6))/(clockDiv * HSPclockDiv));
    var red_delay = inst.hrpwm_requiredRED * Math.pow(10, -9)
    var red = red_delay/(TTBCLK/2)
    var redhr = Math.trunc(((red - Math.floor(red)) * inst.hrpwm_mepSteps + .5) * 256)
    return redhr
}

function calculateFED(inst,ui)
{
    var clockDiv = parseInt(inst.epwmTimebase_clockDiv.replace("EPWM_CLOCK_DIVIDER_", ""))
    var HSPclockDiv = parseInt(inst.epwmTimebase_hsClockDiv.replace("EPWM_HSCLOCK_DIVIDER_", ""))
    var TTBCLK = 1/((inst.hrpwm_epwmclk * Math.pow(10,6))/(clockDiv * HSPclockDiv));
    var fed_delay = inst.hrpwm_requiredFED * Math.pow(10, -9)
    var fed = Math.trunc(fed_delay/(TTBCLK/2))
    return fed
}

function calculateFEDHR(inst,ui)
{
    var clockDiv = parseInt(inst.epwmTimebase_clockDiv.replace("EPWM_CLOCK_DIVIDER_", ""))
    var HSPclockDiv = parseInt(inst.epwmTimebase_hsClockDiv.replace("EPWM_HSCLOCK_DIVIDER_", ""))
    var TTBCLK = 1/((inst.hrpwm_epwmclk * Math.pow(10,6))/(clockDiv * HSPclockDiv));
    var fed_delay = inst.hrpwm_requiredFED * Math.pow(10, -9)
    var fed = fed_delay/(TTBCLK/2)
    var fedhr = Math.trunc(((fed - Math.floor(fed)) * inst.hrpwm_mepSteps + .5) * 256)
    return fedhr
}

function hrpwmEnableSettings(inst, ui)
{
    if(inst.hrpwm_enable)
    {
        ui.hrpwm_autoConv.hidden = false;
        ui.hrpwm_enableCalculator.hidden = false;
        ui.hrpwm_controlModeA.hidden = false;
        ui.hrpwm_controlModeB.hidden = false;
        ui.hrpwm_edgeModeDB.hidden = false;
    }
    else
    {
        // Change everything to the default state:
        inst.hrpwm_autoConv = false;
        inst.hrpwm_enableCalculator = false;

        ui.hrpwm_autoConv.hidden = true;
        ui.hrpwm_enableCalculator.hidden = true;
        ui.hrpwm_controlModeA.hidden = true;
        ui.hrpwm_controlModeB.hidden = true;
        ui.hrpwm_edgeModeDB.hidden = true;
    }
    checkSettings(inst,ui);
}

function checkSettings(inst,ui)
{
    // Check if Calculator is enabled
    if (inst.hrpwm_enableCalculator){
        ui.hrpwm_epwmclk.hidden = false;
        ui.hrpwm_tbclk.hidden = false;
        ui.hrpwm_period.hidden = false;
        ui.hrpwm_duty.hidden = false;
        ui.hrpwm_requiredRED.hidden = false;
        ui.hrpwm_requiredFED.hidden = false;
        ui.hrpwm_scaleFactor.hidden = false;
        ui.hrpwm_mepSteps.hidden = false;
        ui.hrpwm_coarseSteps.hidden = false;
        ui.hrpwm_periodCoarseSteps.hidden = false;
        ui.hrpwm_calculatedCMPX.hidden = false;
        ui.hrpwm_calculatedCMPXHR.hidden = false;
        ui.hrpwm_calculatedTBPRD.hidden = false;
        ui.hrpwm_calculatedTBPRDHR.hidden = false;
        ui.hrpwm_calculatedRED.hidden = false;
        ui.hrpwm_calculatedREDHR.hidden = false;
        ui.hrpwm_calculatedFED.hidden = false;
        ui.hrpwm_calculatedFEDHR.hidden = false;
        }
    else{
        ui.hrpwm_epwmclk.hidden = true;
        ui.hrpwm_tbclk.hidden = true;
        ui.hrpwm_period.hidden = true;
        ui.hrpwm_duty.hidden = true;
        ui.hrpwm_requiredRED.hidden = true;
        ui.hrpwm_requiredFED.hidden = true;
        ui.hrpwm_scaleFactor.hidden = true;
        ui.hrpwm_mepSteps.hidden = true;
        ui.hrpwm_coarseSteps.hidden = true;
        ui.hrpwm_periodCoarseSteps.hidden = true;
        ui.hrpwm_calculatedCMPX.hidden = true;
        ui.hrpwm_calculatedCMPXHR.hidden = true;
        ui.hrpwm_calculatedTBPRD.hidden = true;
        ui.hrpwm_calculatedTBPRDHR.hidden = true;
        ui.hrpwm_calculatedRED.hidden = true;
        ui.hrpwm_calculatedREDHR.hidden = true;
        ui.hrpwm_calculatedFED.hidden = true;
        ui.hrpwm_calculatedFEDHR.hidden = true;
    }

    //Check the control modes
    if (inst.hrpwm_controlModeA == 'HRPWM_MEP_DUTY_PERIOD_CTRL' || inst.hrpwm_controlModeB == 'HRPWM_MEP_DUTY_PERIOD_CTRL')
    {
        ui.hrpwm_edgeModeA.hidden = false;
        if (inst.hrpwm_edgeModeA == "HRPWM_MEP_CTRL_DISABLE")
        {
            ui.hrpwm_cmpaHR.hidden = true;
            ui.hrpwm_HRLoadA.hidden = true;
        }
        else{
            ui.hrpwm_cmpaHR.hidden = false;
            ui.hrpwm_HRLoadA.hidden = false;
        }
        ui.hrpwm_edgeModeB.hidden = false;
        if (inst.hrpwm_edgeModeB == "HRPWM_MEP_CTRL_DISABLE")
        {
            ui.hrpwm_cmpbHR.hidden = true;
            ui.hrpwm_HRLoadB.hidden = true;
        }
        else{
            ui.hrpwm_cmpbHR.hidden = false;
            ui.hrpwm_HRLoadB.hidden = false;
        }
        ui.hrpwm_periodEnable.hidden = false;
        if (inst.hrpwm_periodEnable)
        {
            ui.hrpwm_tbprdHR.hidden = false;
        }
        else
        {
            ui.hrpwm_tbprdHR.hidden = true;
        }

        if (inst.hrpwm_controlModeA == "HRPWM_MEP_DUTY_PERIOD_CTRL" && inst.hrpwm_controlModeB == "HRPWM_MEP_DUTY_PERIOD_CTRL")
        {
            ui.hrpwm_tbphsHR.hidden = true;
            ui.hrpwm_phaseLoadEnable.hidden = true;
        }
        else if (inst.hrpwm_controlModeA == "HRPWM_MEP_PHASE_CTRL")
        {
            ui.hrpwm_edgeModeA.hidden = true;
            ui.hrpwm_cmpaHR.hidden = true;
            ui.hrpwm_HRLoadA.hidden = true;

            ui.hrpwm_tbphsHR.hidden = false;
            ui.hrpwm_phaseLoadEnable.hidden = false;
        }
        else if (inst.hrpwm_controlModeB == "HRPWM_MEP_PHASE_CTRL")
        {
            ui.hrpwm_edgeModeB.hidden = true;
            ui.hrpwm_cmpbHR.hidden = true;
            ui.hrpwm_HRLoadB.hidden = true;

            ui.hrpwm_tbphsHR.hidden = false;
            ui.hrpwm_phaseLoadEnable.hidden = false;
        }
    }
    else
    {
        ui.hrpwm_edgeModeA.hidden = true;
        ui.hrpwm_cmpaHR.hidden = true;
        ui.hrpwm_HRLoadA.hidden = true;
        ui.hrpwm_edgeModeB.hidden = true;
        ui.hrpwm_cmpbHR.hidden = true;
        ui.hrpwm_HRLoadB.hidden = true;
        ui.hrpwm_periodEnable.hidden = true;
        ui.hrpwm_tbprdHR.hidden = true;

        ui.hrpwm_tbphsHR.hidden = false;
        ui.hrpwm_phaseLoadEnable.hidden = false;
    }

    // Check if deadband is utilized
    if (inst.hrpwm_edgeModeDB == "HRPWM_DB_MEP_CTRL_DISABLE")
    {
        ui.hrpwm_DBredHR.hidden = true;
        ui.hrpwm_controlModeDBA.hidden = true;
        ui.hrpwm_DBfedHR.hidden = true;
        ui.hrpwm_controlModeDBB.hidden = true;
        ui.hrpwm_swapDBOutputs.hidden = true;
        ui.hrpwm_DBBOutput.hidden = true;
    }
    else if (inst.hrpwm_edgeModeDB == "HRPWM_DB_MEP_CTRL_RED")
    {
        ui.hrpwm_DBredHR.hidden = false;
        ui.hrpwm_controlModeDBA.hidden = false;
        ui.hrpwm_DBfedHR.hidden = true;
        ui.hrpwm_controlModeDBB.hidden = true;
        ui.hrpwm_swapDBOutputs.hidden = false;
        ui.hrpwm_DBBOutput.hidden = false;
    }
    else if (inst.hrpwm_edgeModeDB =="HRPWM_DB_MEP_CTRL_FED")
    {
        ui.hrpwm_DBredHR.hidden = true;
        ui.hrpwm_controlModeDBA.hidden = true;
        ui.hrpwm_DBfedHR.hidden = false;
        ui.hrpwm_controlModeDBB.hidden = false;
        ui.hrpwm_swapDBOutputs.hidden = false;
        ui.hrpwm_DBBOutput.hidden = false;
    }
    else if (inst.hrpwm_edgeModeDB =="HRPWM_DB_MEP_CTRL_RED_FED")
    {
        ui.hrpwm_DBredHR.hidden = false;
        ui.hrpwm_controlModeDBA.hidden = false;
        ui.hrpwm_DBfedHR.hidden = false;
        ui.hrpwm_controlModeDBB.hidden = false;
        ui.hrpwm_swapDBOutputs.hidden = false;
        ui.hrpwm_DBBOutput.hidden = false;
    }
}

var config = [
    {
        name: "hrpwm_enable",
        displayName : "Enable HRPWM Support",
        description : 'Click the checkbox to enable hrpwm support',
        hidden      : false,
        default     : false,
        onChange    : hrpwmEnableSettings
    },
    {
        name: "hrpwm_autoConv",
        displayName : "Enable Automatic HRMSTEP Scaling",
        description : 'Selects whether the fractional duty cycle/period/phase is automatically scaled by the MEP scale factor',
        hidden      : true,
        default     : false,
        onChange    : checkSettings
    },
    {
        name: "hrpwm_enableCalculator",
        displayName : "Enable HRPWM Timing Calculator",
        description : 'Click the checkbox to enable hrpwm calculator',
        hidden      : true,
        default     : false,
        onChange    : checkSettings
    },
    {
        name: "hrpwm_controlModeA",
        displayName : "Control Mode For Channel A",
        description : 'Selects the register that controls the MEP',
        hidden      : true,
        default     : device_peripheral.HRPWM_MEPCtrlMode[0].name,
        options     : device_peripheral.HRPWM_MEPCtrlMode,
        onChange    : checkSettings
    },
    {
        name: "hrpwm_controlModeB",
        displayName : "Control Mode For Channel B",
        description : 'Selects the register that controls the MEP',
        hidden      : true,
        default     : device_peripheral.HRPWM_MEPCtrlMode[0].name,
        options     : device_peripheral.HRPWM_MEPCtrlMode,
        onChange    : checkSettings
    },
    {
        name: "hrpwm_SettingConfig",
        displayName: "HRPWM Timing Calculator",
        description: "",
        longDescription: "",
        config: [
                {
                    name: "hrpwm_epwmclk",
                    displayName : "EPWMCLK (MHz)",
                    description : 'Frequency of the EPWM Module (MHz)',
                    hidden      : true,
                    default     : 100
                },
                {
                    name: "hrpwm_tbclk",
                    displayName : "TBCLK(MHz)",
                    description : 'Frequency of the Time Base Counter',
                    hidden      : true,
                    getValue    : (inst) => calculateTBCLK(inst),
                    default     : 0
                },
                {
                    name: "hrpwm_period",
                    displayName : "Required Period (nsec)",
                    description : 'The period of the EPWM output',
                    hidden      : true,
                    default     : 800
                },
                {
                    name: "hrpwm_duty",
                    displayName : "Required Duty Cycle (%)",
                    description : 'The percentage of on time over off time',
                    hidden      : true,
                    default     : 50
                },
                {
                    name: "hrpwm_requiredRED",
                    displayName : "Required Rising Edge Delay (nsec)",
                    description : 'Deadband Rising Edge Delay Requirement',
                    hidden      : true,
                    default     : 0,
                },
                {
                    name: "hrpwm_requiredFED",
                    displayName : "Required Falling Edge Delay (nsec)",
                    description : 'Deadband Falling Edge Delay Requirement',
                    hidden      : true,
                    default     : 0,
                },
                {
                    name: "hrpwm_scaleFactor",
                    displayName : "Approximated MEP Step Size (psec)",
                    description : 'Micro-Edge Positioning Scale Factor',
                    hidden      : true,
                    default     : 180
                },
                {
                    name: "hrpwm_mepSteps",
                    displayName : "Approximated MAX MEP Steps",
                    description : 'Number of MEP Steps per coarse step',
                    hidden      : true,
                    default     : 0,
                    getValue    : (inst) => calculateMEPSteps(inst),
                },
                {
                    name: "hrpwm_coarseSteps",
                    displayName : "Coarse Steps",
                    description : 'Number of Coarse Steps',
                    hidden      : true,
                    getValue    : (inst) => calculateCoarseSteps(inst),
                    default     : 0
                },
                {
                    name: "hrpwm_periodCoarseSteps",
                    displayName : "Period In Coarse Steps",
                    description : 'Period Value represented in coarse steps',
                    hidden      : true,
                    default     : 0,
                    getValue    : (inst) => calculatePeriodCoarseSteps(inst),
                },
                {
                    name: "hrpwm_calculatedCMPX",
                    displayName : "Calulated CMPA/CMPB",
                    description : 'Calulated value for counter compare',
                    hidden      : true,
                    default     : 0,
                    getValue    : (inst) => calculateCMPX(inst),
                },
                {
                    name: "hrpwm_calculatedCMPXHR",
                    displayName : "Calculated CMPAHR/CMPBHR",
                    description : 'Calulated value for counter compare HR',
                    hidden      : true,
                    default     : 0,
                    getValue    : (inst) => calculateCMPXHR(inst),
                },
                {
                    name: "hrpwm_calculatedTBPRD",
                    displayName : "Calculated TBPRD",
                    description : 'Calulated value for TBPRDHR',
                    hidden      : true,
                    default     : 0,
                    getValue    : (inst) => calculateTBPRD(inst),
                },
                {
                    name: "hrpwm_calculatedTBPRDHR",
                    displayName : "Calculated TBPRDHR",
                    description : 'Calulated value for TBPRDHR',
                    hidden      : true,
                    default     : 0,
                    getValue    : (inst) => calculateTBPRDHR(inst),
                },
                {
                    name: "hrpwm_calculatedRED",
                    displayName : "Calculated RED",
                    description : 'Calulated value for Rising Edge Delay',
                    hidden      : true,
                    default     : 0,
                    getValue    : (inst) => calculateRED(inst),
                },
                {
                    name: "hrpwm_calculatedREDHR",
                    displayName : "Calculated DBREDHR",
                    description : 'Calulated value for Rising Edge Delay HR',
                    hidden      : true,
                    default     : 0,
                    getValue    : (inst) => calculateREDHR(inst),
                },
                {
                    name: "hrpwm_calculatedFED",
                    displayName : "Calculated FED",
                    description : 'Calulated value for Falling Edge Delay',
                    hidden      : true,
                    default     : 0,
                    getValue    : (inst) => calculateFED(inst),
                },
                {
                    name: "hrpwm_calculatedFEDHR",
                    displayName : "Calculated DBFEDHR",
                    description : 'Calulated value for Falling Edge Delay HR',
                    hidden      : true,
                    default     : 0,
                    getValue    : (inst) => calculateFEDHR(inst),
                },

            ]
    },
    {
        name: "hrpwm_phaseControlConfig",
        displayName: "HRPWM Phase Control",
        description: "",
        longDescription: "",
        config: [
                {
                    name: "hrpwm_tbphsHR",
                    displayName : "High Resolution Phase",
                    description : 'Phase High Resolution Value',
                    hidden      : true,
                    default     : 0,
                },
                {
                    name: "hrpwm_phaseLoadEnable",
                    displayName : "High Resolution Phase Load Enable",
                    description : 'Synchronize ePWM module with high resolution phase',
                    hidden      : true,
                    default     : false,
                },
            ]
    },
    {
        name: "hrpwm_dutyControlConfig",
        displayName: "HRPWM Duty And Period Control",
        description: "",
        longDescription: "",
        config: [
                {
                name: "hrpwm_channelAConfig",
                displayName: "HRPWM Channel A",
                description: "",
                longDescription: "",
                config: [
                    {
                        name: "hrpwm_edgeModeA",
                        displayName : "Edge Mode",
                        description : 'Selects the edge of the PWM that is controlled by the MEP logic',
                        hidden      : false,
                        default     : device_peripheral.HRPWM_MEPEdgeMode[0].name,
                        options     : device_peripheral.HRPWM_MEPEdgeMode,
                        onChange    : checkSettings
                    },
                    {
                        name: "hrpwm_cmpaHR",
                        displayName : "High Resolution CMPA (CMPAHR)",
                        description : 'Counter Compare A (CMPA) Hardware Extension',
                        hidden      : true,
                        default     : 1
                    },
                    {
                        name: "hrpwm_HRLoadA",
                        displayName : "CMPAHR Shadow To Active Load Event",
                        description : 'Selects the event that loads CMPAHR shadow value into the active register',
                        hidden      : true,
                        default     : device_peripheral.HRPWM_LoadMode[0].name,
                        options     : device_peripheral.HRPWM_LoadMode
                    }
                   ]
                },
                {
                name: "hrpwm_ChannelBConfig",
                displayName: "HRPWM Channel B",
                description: "",
                longDescription: "",
                config: [
                        {
                            name: "hrpwm_edgeModeB",
                            displayName : "Edge Mode",
                            description : 'Selects the edge of the PWM that is controlled by the MEP logic',
                            hidden      : false,
                            default     : device_peripheral.HRPWM_MEPEdgeMode[0].name,
                            options     : device_peripheral.HRPWM_MEPEdgeMode,
                            onChange    : checkSettings
                        },
                        {
                            name: "hrpwm_cmpbHR",
                            displayName : "High Resolution CMPB (CMPBHR)",
                            description : 'Counter Compare B (CMPB) Hardware Extension',
                            hidden      : true,
                            default     : 1,
                        },
                        {
                            name: "hrpwm_HRLoadB",
                            displayName : "CMPBHR Shadow To Active Load Event",
                            description : 'Selects the event that loads CMPBHR shadow value into the active register',
                            hidden      : true,
                            default     : device_peripheral.HRPWM_LoadMode[0].name,
                            options     : device_peripheral.HRPWM_LoadMode
                        },
                    ]
                },
                {
                name: "hrpwm_periodControlConfig",
                displayName: "HRPWM Period Control",
                description: "",
                longDescription: "",
                config: [
                        {
                            name: "hrpwm_periodEnable",
                            displayName : "High Resolution Period Enable",
                            description : 'Enables the hardware extension of the period register',
                            hidden      : false,
                            default     : false,
                            onChange    : checkSettings
                        },
                        {
                            name: "hrpwm_tbprdHR",
                            displayName : "High Resolution Period",
                            description : 'Time Base Period High Resolution Value',
                            hidden      : true,
                            default     : 0
                        },
                    ]
                },
        ]
    },
    {
        name: "hrpwm_DeadbandControl",
        displayName: "HRPWM Deadband Control",
        description: "",
        longDescription: "",
        config: [
            {
                name: "hrpwm_edgeModeDB",
                displayName : "Deadband Edge Mode",
                description : 'Select the edge or edges of the PWM that are controlled by the MEP logic',
                hidden      : true,
                default     : device_peripheral.HRPWM_MEPDeadBandEdgeMode[0].name,
                options     : device_peripheral.HRPWM_MEPDeadBandEdgeMode,
                onChange    : checkSettings
            },
            {
                name: "hrpwm_DBredHR",
                displayName : "High Resoltion RED",
                description : 'Deadband Rising Edge Delay Hardware Extension',
                hidden      : true,
                default     : 0,
            },
            {
                name: "hrpwm_controlModeDBA",
                displayName : "Shadow To Active Load Of DBREDHR",
                description : 'Selects the event that loads the DBREDHR value into the active register',
                hidden      : true,
                default     : device_peripheral.HRPWM_LoadMode[0].name,
                options     : device_peripheral.HRPWM_LoadMode
            },
            {
                name: "hrpwm_DBfedHR",
                displayName : "High Resolution FED",
                description : 'Deadband Falling Edge Delay Hardware Extension',
                hidden      : true,
                default     : 0,
            },
            {
                name: "hrpwm_controlModeDBB",
                displayName : "Shadow To Active Load Of DBFEDHR",
                description : 'Selects the event that loads the DBFEDHR value into the active register',
                hidden      : true,
                default     : device_peripheral.HRPWM_LoadMode[0].name,
                options     : device_peripheral.HRPWM_LoadMode
            },
            {
                name: "hrpwm_swapDBOutputs",
                displayName : "Swap A And B Outputs",
                description : 'Enable the swaping of the A & B outputs',
                hidden      : true,
                default     : false,
            },
            {
                name: "hrpwm_DBBOutput",
                displayName : "EPWMxB Output Select",
                description : 'Selects which signal is output on the EPWMxB channel',
                hidden      : true,
                default     : device_peripheral.HRPWM_ChannelBOutput[0].name,
                options     : device_peripheral.HRPWM_ChannelBOutput
            },
        ]
    }
];

var hrpwmExtension = {
    displayName: "HRPWM",
    // maxInstances: common.peripheralCount("EPWM"),
    defaultInstanceName: "HRPWM",
    description: "Enhanced Pulse Width Modulator High Resolution",
    config: config,
    templates: {
        boardc : "",
        boardh : ""
    },
};


exports = hrpwmExtension;