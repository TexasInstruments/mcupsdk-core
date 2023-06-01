let common   = system.getScript("/common");
let device_peripheral = system.getScript(`/drivers/epwm/soc/epwm_${common.getSocName()}.syscfg.js`);

/*
    Structure of the sysconfig features

    config [0]  --> "epwmXCMP_enableMode"
    config [1]  --> "epwmXCMP_SplitCheck"
    config [2]  --> "epwmXCMP_No_Split"
    config [3]  --> "epwmXCMP_Split_A"
    config [4]  --> "epwmXCMP_Split_B"
    config [5]  --> "GROUP_ACTIVE";  config[5].config[0] --> "epwmXCMP1_Active" .... config[5].config[7] --> "epwmXCMP8_Active"
    config [6]  --> "GROUP_SHADOW1";  config[6].config[0] --> "epwmXCMP1_Shdw1" .... config[6].config[7] --> "epwmXCMP8_Shdw1"
    config [7]  --> "GROUP_SHADOW2";  config[7].config[0] --> "epwmXCMP1_Shdw2" .... config[7].config[7] --> "epwmXCMP8_Shdw2"
    config [8]  --> "GROUP_SHADOW3";  config[8].config[0] --> "epwmXCMP1_Shdw3" .... config[8].config[7] --> "epwmXCMP8_Shdw3"
    config [9]  --> "GROUP_AQACTIVE";  config[9].config[0] --> "epwmXAQCTL_EPWM_AQ_OUTPUT_A_ON_TIMEBASE_XCMP1_ACTIVE"
                                  .... config[9].config[7] --> "epwmXAQCTL_EPWM_AQ_OUTPUT_A_ON_TIMEBASE_XCMP8_ACTIVE"
                                       config[9].config[8] --> "epwmXAQCTL_EPWM_AQ_OUTPUT_B_ON_TIMEBASE_XCMP5_ACTIVE"
                                       config[9].config[11] --> "epwmXAQCTL_EPWM_AQ_OUTPUT_B_ON_TIMEBASE_XCMP8_ACTIVE"

    config [10]  --> "GROUP_AQSHADOW_1";  config[10].config[0] --> "epwmXAQCTL_EPWM_AQ_OUTPUT_A_ON_TIMEBASE_XCMP1_SHADOW_1"
                                  .... config[10].config[7] --> "epwmXAQCTL_EPWM_AQ_OUTPUT_A_ON_TIMEBASE_XCMP8_SHADOW_1"
                                       config[10].config[8] --> "epwmXAQCTL_EPWM_AQ_OUTPUT_B_ON_TIMEBASE_XCMP5_SHADOW_1"
                                       config[10].config[11] --> "epwmXAQCTL_EPWM_AQ_OUTPUT_B_ON_TIMEBASE_XCMP8_SHADOW_1"

    config [11]  --> "GROUP_AQSHADOW_2";  config[11].config[0] --> "epwmXAQCTL_EPWM_AQ_OUTPUT_A_ON_TIMEBASE_XCMP1_SHADOW_2"
                                  .... config[11].config[7] --> "epwmXAQCTL_EPWM_AQ_OUTPUT_A_ON_TIMEBASE_XCMP8_SHADOW_2"
                                       config[11].config[8] --> "epwmXAQCTL_EPWM_AQ_OUTPUT_B_ON_TIMEBASE_XCMP5_SHADOW_2"
                                       config[11].config[11] --> "epwmXAQCTL_EPWM_AQ_OUTPUT_B_ON_TIMEBASE_XCMP8_SHADOW_2"

    config [12]  --> "GROUP_AQSHADOW_3";  config[12].config[0] --> "epwmXAQCTL_EPWM_AQ_OUTPUT_A_ON_TIMEBASE_XCMP1_SHADOW_3"
                                  .... config[12].config[7] --> "epwmXAQCTL_EPWM_AQ_OUTPUT_A_ON_TIMEBASE_XCMP8_SHADOW_3"
                                       config[12].config[8] --> "epwmXAQCTL_EPWM_AQ_OUTPUT_B_ON_TIMEBASE_XCMP5_SHADOW_3"
                                       config[12].config[11] --> "epwmXAQCTL_EPWM_AQ_OUTPUT_B_ON_TIMEBASE_XCMP8_SHADOW_3"

    config [13]  --> "GROUP_XMINMAX";  config[13].config[0] --> "epwmXMin_Active", config[13].config[1] --> "epwmXMax_Active"
                                       ..config[13].config[6] --> "epwmXMin_Shdw3", config[13].config[7] --> "epwmXMax_Shdw3"
    config [14]  --> "GROUP_XTBPRD";   config[14].config[0] --> "epwmXTBPRD_Active" ... config[14].config[4] --> "epwmXTBPRD_Shdw3"
    config [15]  --> "GROUP_XLOADCTL"  config[15].config[0] --> "epwmXLOADCTL_Loadmode",
                                       config[15].config[1] --> "epwmXLOADCTL_Shadowlevel",
                                       config[15].config[2] --> "epwmXLOADCTL_Loadonce",
                                       config[15].config[3] --> "epwmXLOADCTL_RepeatBuf2",
                                       config[15].config[04] --> "epwmXLOADCTL_RepeatBuf3",
*/

function onClickEnable(inst, ui)
{
    if (inst.epwmXCMP_enableMode)
    {
        ui["epwmXCMP_SplitCheck"].hidden = false;
        ui["epwmXCMP_No_Split"].hidden = false;
        ui["epwmXLOADCTL_Loadmode"].hidden = false;
        ui["epwmXLOADCTL_Shadowlevel"].hidden = false;
        ui["epwmXLOADCTL_Loadonce"].hidden = false;
        for(let uiConfigIndex = 1; uiConfigIndex < config.length; uiConfigIndex++)
        {
            let configName = config[uiConfigIndex].name;
            if (configName.startsWith("GROUP_XMINMAX")
                ||  configName.startsWith("GROUP_XTBPRD")
                || configName.startsWith("GROUP_XLOADCTL"))
            {
                let groupLen= config[uiConfigIndex].config.length;
                let active_configurables_count = 1;
                if(configName.startsWith("GROUP_XMINMAX"))
                    active_configurables_count =2;

                for(let i = 0; i < active_configurables_count; i++)  //Displaying only the ACTIVE set
                    {
                        let subConfigName = config[uiConfigIndex].config[i].name;
                        ui[subConfigName].hidden = false;
                    }
            }
        }
    }
    else if(inst.epwmXCMP_enableMode == false)
    {

        inst["epwmXCMP_No_Split"] = device_peripheral.EPWM_XCMP_ALLOC_CMPA[0].name; //Resetting to default value : "Allocate 0 XCMP registers to CMPA"
        inst["epwmXCMP_SplitCheck"] = false;//Resetting to default value : false
        inst["epwmXCMP_Split_A"] = device_peripheral.EPWM_XCMP_ALLOC_CMPA[0].name; //Resetting to default value : "Allocate 0 XCMP registers to CMPA"
        inst["epwmXCMP_Split_B"] = device_peripheral.EPWM_XCMP_ALLOC_CMPB[0].name; //Resetting to default value : "Allocate XCMP5 registers to CMPB"
        inst["epwmXLOADCTL_Loadmode"] = device_peripheral.EPWM_XCMPXloadCtlLoadMode[0].name; //Resetting to default value : "LOAD ONCE"
        inst["epwmXLOADCTL_Shadowlevel"] = device_peripheral.EPWM_XCMP_XLOADCTL_SHDWLEVEL[0].name; //Resetting to default value : "SHADOW LEVEL ZERO"
        inst["epwmXLOADCTL_Loadonce"] = device_peripheral.EPWM_XCMP_XLOADCTL_SHDWBUFPTR[0].name; //Resetting to default value : "EPWM_XCMP_XLOADCTL_SHDWBUFPTR_NULL"

        ui["epwmXCMP_SplitCheck"].hidden = true;
        ui["epwmXCMP_Split_A"].hidden = true;
        ui["epwmXCMP_Split_B"].hidden = true;
        ui["epwmXCMP_No_Split"].hidden = true;
        ui["epwmXLOADCTL_Loadmode"].hidden = true;
        ui["epwmXLOADCTL_Shadowlevel"].hidden = true;

        onSelectNoSplit(inst, ui);
        onSelectShadowLevel(inst, ui);

        for(let uiConfigIndex = 1; uiConfigIndex < config.length; uiConfigIndex++)
        {
            let configName = config[uiConfigIndex].name;
            if (configName.startsWith("GROUP_XMINMAX")
            ||  configName.startsWith("GROUP_XTBPRD"))
            {
                let groupLen= config[uiConfigIndex].config.length;
                let active_configurables_count = 1;
                if(configName.startsWith("GROUP_XMINMAX"))
                    active_configurables_count =2;

                for(let i = 0; i < active_configurables_count; i++)  //Hiding all the registers set
                    {
                        let subConfigName = config[uiConfigIndex].config[i].name;
                        ui[subConfigName].hidden = true;
                        inst[subConfigName] = 0;    //Resetting to default value : 0
                    }
            }
            else if(configName.startsWith("GROUP_XLOADCTL"))
            {
                let groupLen= config[uiConfigIndex].config.length;
                for(let i = 0; i < 5; i++)
                {
                    let subConfigName = config[uiConfigIndex].config[i].name;
                    ui[subConfigName].hidden = true;
                    if(subConfigName == "epwmXLOADCTL_Loadmode")
                        inst[subConfigName] = device_peripheral.EPWM_XCMPXloadCtlLoadMode[0].name;
                    else if(subConfigName == "epwmXLOADCTL_Shadowlevel")
                        inst[subConfigName] = device_peripheral.EPWM_XCMP_XLOADCTL_SHDWLEVEL[0].name;
                    else if(subConfigName == "epwmXLOADCTL_Loadonce")
                        inst[subConfigName] = device_peripheral.EPWM_XCMP_XLOADCTL_SHDWBUFPTR[0].name;
                    else if(subConfigName == "epwmXLOADCTL_RepeatBuf2")
                        inst[subConfigName] = 0;
                    else if(subConfigName == "epwmXLOADCTL_RepeatBuf3")
                        inst[subConfigName] = 0;
                }
            }
        }
    }


}

function onClickSplit(inst, ui)
{
    let level = 3;

    for(let i=5; i<13; i++)
        for(let j=0;j<8;j++)
            {
                ui[config[i].config[j].name].hidden = true;
                //Resetting to default value. For indices refer the comment at top of file. For indices refer the comment at top of file
                if( i >=5 && i <= 8) //Active and Shadow set 1-3 register default value is 0
                    inst[config[i].config[j].name] = 0;
                else //Active and Shadow set 1-3 Action Qualifier default value is "No change in the output pins"
                    inst[config[i].config[j].name] = device_peripheral.EPWM_ActionQualifierOutput[0].name;
            }

    if(inst.epwmXLOADCTL_Shadowlevel == device_peripheral.EPWM_XCMP_XLOADCTL_SHDWLEVEL[3].name)
        level = 0;
    else if(inst.epwmXLOADCTL_Shadowlevel == device_peripheral.EPWM_XCMP_XLOADCTL_SHDWLEVEL[2].name)
        level = 1;
    else if(inst.epwmXLOADCTL_Shadowlevel == device_peripheral.EPWM_XCMP_XLOADCTL_SHDWLEVEL[1].name)
        level = 2;

    if(inst.epwmXCMP_SplitCheck)  //Splitting b/w CMPA and CMPB
    {
        //Display the EPWM_XCMP_ALLOC_CMPA and EPWM_XCMP_ALLOC_CMPB options
        ui.epwmXCMP_Split_A.hidden = false;
        ui.epwmXCMP_Split_B.hidden = false;
        ui.epwmXCMP_No_Split.hidden = true;
        inst["epwmXCMP_No_Split"] = device_peripheral.EPWM_XCMP_ALLOC_CMPA[0].name;

        for(let i=5; i<13; i++)
            if( ( (i>=5 && i<=8-level) || (i>=9 && i<=12-level) ) )
                {
                    ui[config[i].config[4].name].hidden = false;
                    if(i == 9)
                    ui[config[i].config[8].name].hidden = false;
                }
    }
    else
    {
        //Display the EPWM_XCMP_ALLOC_CMPA options
        ui.epwmXCMP_Split_A.hidden = true;
        inst["epwmXCMP_Split_A"] = device_peripheral.EPWM_XCMP_ALLOC_CMPA[0].name; //Resetting to default value : "Allocate 0 XCMP registers to CMPA"
        ui.epwmXCMP_Split_B.hidden = true;
        inst["epwmXCMP_Split_B"] = device_peripheral.EPWM_XCMP_ALLOC_CMPB[0].name; //Resetting to default value : "Allocate XCMP5 register to CMPB"
        ui.epwmXCMP_No_Split.hidden = false;

        for(let i=5; i<13; i++)
            if( ( (i>=5 && i<=8-level) || (i>=9 && i<=12-level) ) )
            {
                ui[config[i].config[4].name].hidden = true;
                //Resetting to default value. For indices refer the comment at top of file
                if (i>=5 && i<=8-level) //Active and Shadow set 1-3 register default value is 0
                    inst[config[i].config[4].name] = 0;
                else if (i>=9 && i<=12-level) //Active and Shadow set 1-3 Action Qualifier Output A default value is "No change in the output pins"
                    inst[config[i].config[4].name] = device_peripheral.EPWM_ActionQualifierOutput[0].name;
                if(i == 9)
                {
                    ui[config[i].config[8].name].hidden = true;
                    //Resetting to default value. Active Action Qualifier Output B default value is "No change in the output pins"
                    inst[config[i].config[8].name] = device_peripheral.EPWM_ActionQualifierOutput[0].name;
                }
            }
    }
}

function utilityDisplayActiveRegisters(inst, ui, start, end, bool)
{
    for(let i=5; i<13; i++)
        {
            for(let j=start;j<end;j++)
            if(! ( (i>=6 && i<=8) || (i>=10 && i<=12) ) )
                {
                    ui[config[i].config[j].name].hidden = bool;

                    if(bool)    //Resetting to default value. For indices refer the comment at top of file
                        {
                            if( i == 5 )    // Active register default value is 0
                                inst[config[i].config[j].name] = 0;
                            if( i == 9 )   // Active Action Qualifier Output A default value is "No change in the output pins"
                                inst[config[i].config[j].name] = device_peripheral.EPWM_ActionQualifierOutput[0].name;
                        }

                    if( i == 9 && j  > 3)
                    {
                        ui[config[i].config[j + 4].name].hidden = bool;
                        if(bool)    //Resetting to default value. Active Action Qualifier Output B default value is "No change in the output pins"
                            inst[[config[i].config[j + 4].name]] = device_peripheral.EPWM_ActionQualifierOutput[0].name;
                    }
                }
        }
}

function utilityDisplayShadowRegisters(inst, ui,bool)
{
    let start = 0;
    let end = 0;
    let level = 3;

    ui["epwmXLOADCTL_RepeatBuf2"].hidden = true;
    ui["epwmXLOADCTL_RepeatBuf3"].hidden = true;

    for(let i=6; i<13; i++)
        for(let j=0;j<8;j++)
            if( ( (i>=6 && i<=8) || (i>=10 && i<=12) ) )
            {
                ui[config[i].config[j].name].hidden = true;

                //Resetting to default value. For indices refer the comment at top of file
                if(i >=6 && i<=8)   // Shadow set 1-3 register default value is 0
                    inst[config[i].config[j].name] = 0;
                else   // Shadow set 1-3 Action Qualifier Output A default value is "No change in the output pins"
                    inst[config[i].config[j].name]= device_peripheral.EPWM_ActionQualifierOutput[0].name;
                if( i > 9 && j > 3)
                    {
                        ui[config[i].config[j + 4].name].hidden = true;
                        //Resetting to default value. Shadow set 1-3 Action Qualifier Output B default value is "No change in the output pins"
                        inst[config[i].config[j + 4].name] = device_peripheral.EPWM_ActionQualifierOutput[0].name;
                    }
            }

    for(let i=13; i<15; i++)    //XMINMAX and XTBPRD
    {
        let shadow_configurables_start_ind = 1;
        let shadow_configurables_end_ind = 3;

        if(i == 13) // XMINMAX
            {
                shadow_configurables_start_ind = 2;
                shadow_configurables_end_ind = 7;
            }

        for(let j=shadow_configurables_start_ind; j<=shadow_configurables_end_ind; j++)
            {
                ui[config[i].config[j].name].hidden = true;
                //Resetting to default value. For indices refer the comment at top of file
                inst[config[i].config[j].name] = 0;
            }
     }

    if(inst.epwmXLOADCTL_Shadowlevel == device_peripheral.EPWM_XCMP_XLOADCTL_SHDWLEVEL[3].name)
        {
            level = 0;
            ui["epwmXTBPRD_Shdw1"].hidden = false;
            ui["epwmXTBPRD_Shdw2"].hidden = false;
            ui["epwmXTBPRD_Shdw3"].hidden = false;
            ui["epwmXMin_Shdw1"].hidden = false;
            ui["epwmXMin_Shdw2"].hidden = false;
            ui["epwmXMin_Shdw3"].hidden = false;
            ui["epwmXMax_Shdw1"].hidden = false;
            ui["epwmXMax_Shdw2"].hidden = false;
            ui["epwmXMax_Shdw3"].hidden = false;
            ui["epwmXLOADCTL_RepeatBuf2"].hidden = false;
            ui["epwmXLOADCTL_RepeatBuf3"].hidden = false;
        }
    else if(inst.epwmXLOADCTL_Shadowlevel == device_peripheral.EPWM_XCMP_XLOADCTL_SHDWLEVEL[2].name)
        {
            level = 1;
            ui["epwmXTBPRD_Shdw1"].hidden = false;
            ui["epwmXTBPRD_Shdw2"].hidden = false;
            ui["epwmXMin_Shdw1"].hidden = false;
            ui["epwmXMin_Shdw2"].hidden = false;
            ui["epwmXMax_Shdw1"].hidden = false;
            ui["epwmXMax_Shdw2"].hidden = false;
            ui["epwmXLOADCTL_RepeatBuf2"].hidden = false;
        }
    else if(inst.epwmXLOADCTL_Shadowlevel == device_peripheral.EPWM_XCMP_XLOADCTL_SHDWLEVEL[1].name)
        {
            level = 2;
            ui["epwmXTBPRD_Shdw1"].hidden = false;
            ui["epwmXMin_Shdw1"].hidden = false;
            ui["epwmXMax_Shdw1"].hidden = false;
        }

    if((!inst.epwmXCMP_SplitCheck && inst.epwmXCMP_No_Split == device_peripheral.EPWM_XCMP_ALLOC_CMPA[0].name) ||
        (inst.epwmXCMP_SplitCheck && inst.epwmXCMP_Split_A == device_peripheral.EPWM_XCMP_ALLOC_CMPA[0].name))
    {}
    else if( (!inst.epwmXCMP_SplitCheck && inst.epwmXCMP_No_Split == device_peripheral.EPWM_XCMP_ALLOC_CMPA[1].name) ||
        (inst.epwmXCMP_SplitCheck && inst.epwmXCMP_Split_A == device_peripheral.EPWM_XCMP_ALLOC_CMPA[1].name))
    {start = 0; end = 1;}
    else if( (!inst.epwmXCMP_SplitCheck && inst.epwmXCMP_No_Split == device_peripheral.EPWM_XCMP_ALLOC_CMPA[2].name) ||
        (inst.epwmXCMP_SplitCheck && inst.epwmXCMP_Split_A == device_peripheral.EPWM_XCMP_ALLOC_CMPA[2].name))
    {start = 0; end = 2;}
    else if( (!inst.epwmXCMP_SplitCheck && inst.epwmXCMP_No_Split == device_peripheral.EPWM_XCMP_ALLOC_CMPA[3].name) ||
        (inst.epwmXCMP_SplitCheck && inst.epwmXCMP_Split_A == device_peripheral.EPWM_XCMP_ALLOC_CMPA[3].name))
    {start = 0; end = 3;}
    else if( (!inst.epwmXCMP_SplitCheck && inst.epwmXCMP_No_Split == device_peripheral.EPWM_XCMP_ALLOC_CMPA[4].name) ||
        (inst.epwmXCMP_SplitCheck && inst.epwmXCMP_Split_A == device_peripheral.EPWM_XCMP_ALLOC_CMPA[4].name))
    {start = 0; end = 4;}
    else if(inst.epwmXCMP_No_Split == device_peripheral.EPWM_XCMP_ALLOC_CMPA[5].name)
    {start = 0; end = 5;}
    else if(inst.epwmXCMP_No_Split == device_peripheral.EPWM_XCMP_ALLOC_CMPA[6].name)
    {start = 0; end = 6;}
    else if(inst.epwmXCMP_No_Split == device_peripheral.EPWM_XCMP_ALLOC_CMPA[7].name)
    {start = 0; end = 7;}
    else if(inst.epwmXCMP_No_Split == device_peripheral.EPWM_XCMP_ALLOC_CMPA[8].name)
    {start = 0; end = 8;}

    for(let i=6; i<13; i++)
        for(let j=start;j<end;j++)
            if( ( (i>=6 && i<=8-level) || (i>=10 && i<=12-level) ) )
            {
                ui[config[i].config[j].name].hidden = bool;
                if(bool)  //Resetting to default value. For indices refer the comment at top of file
                {
                    if(i>=6 && i<=8-level)  // Shadow set 1-3 register default value is 0
                        inst[config[i].config[j].name] = 0;
                    else    // Shadow set 1-3 Action Qualifier Output A default value is "No change in the output pins"
                        inst[config[i].config[j].name]= device_peripheral.EPWM_ActionQualifierOutput[0].name;
                }
                if( i > 9 && j  > 3)
                    {
                        ui[config[i].config[j + 4].name].hidden = bool;
                        if(bool)    //Resetting to default value.  Shadow set 1-3 Action Qualifier Output B default value is "No change in the output pins"
                            inst[config[i].config[j + 4].name] = device_peripheral.EPWM_ActionQualifierOutput[0].name;
                    }
            }

    if(inst.epwmXCMP_SplitCheck)
   {
        if(inst.epwmXCMP_Split_B == device_peripheral.EPWM_XCMP_ALLOC_CMPB[0].name)
        {start = 4; end = 5;}
        else if(inst.epwmXCMP_Split_B == device_peripheral.EPWM_XCMP_ALLOC_CMPB[1].name)
        {start = 4; end = 6;}
        else if(inst.epwmXCMP_Split_B == device_peripheral.EPWM_XCMP_ALLOC_CMPB[2].name)
        {start = 4; end = 7;}
        else if(inst.epwmXCMP_Split_B == device_peripheral.EPWM_XCMP_ALLOC_CMPB[3].name)
        {start = 4; end = 8;}


        for(let i=6; i<13; i++)
            for(let j=start;j<end;j++)
                if( ( (i>=6 && i<=8-level) || (i>=10 && i<=12-level) ) )
                {
                    ui[config[i].config[j].name].hidden = bool;
                    if(bool)    //Resetting to default value. For indices refer the comment at top of file
                    {
                        if(i>=6 && i<=8-level) // Shadow set 1-3 register default value is 0
                            inst[config[i].config[j].name] = 0;
                        else                  // Shadow set 1-3 Action Qualifier Output A default value is "No change in the output pins"
                            inst[config[i].config[j].name]= device_peripheral.EPWM_ActionQualifierOutput[0].name;
                    }
                    if( i > 9 && j  > 3)
                        {
                            ui[config[i].config[j + 4].name].hidden = bool;
                            if(bool)    //Resetting to default value. Shadow set 1-3 Action Qualifier Output B default value is "No change in the output pins"
                                inst[config[i].config[j + 4].name] = device_peripheral.EPWM_ActionQualifierOutput[0].name;
                        }
                }
    }
}

function onSelectNoSplit(inst, ui)
{

    utilityDisplayActiveRegisters(inst, ui,0, 8, true);


    if(inst.epwmXCMP_No_Split == device_peripheral.EPWM_XCMP_ALLOC_CMPA[0].name) //No XCMP to CMPA
    {
        //Do Nothing
    }
    else  if(inst.epwmXCMP_No_Split == device_peripheral.EPWM_XCMP_ALLOC_CMPA[1].name) //XCMP1 to CMPA
    {
        utilityDisplayActiveRegisters(inst, ui,0, 1, false);
    }
    else  if(inst.epwmXCMP_No_Split == device_peripheral.EPWM_XCMP_ALLOC_CMPA[2].name) //XCMP1-2 to CMPA
    {
        utilityDisplayActiveRegisters(inst, ui,0, 2, false);
    }
    else  if(inst.epwmXCMP_No_Split == device_peripheral.EPWM_XCMP_ALLOC_CMPA[3].name) //XCMP1-3 to CMPA
    {
        utilityDisplayActiveRegisters(inst, ui,0, 3, false);
    }
    else  if(inst.epwmXCMP_No_Split == device_peripheral.EPWM_XCMP_ALLOC_CMPA[4].name) //XCMP1-4 to CMPA
    {
        utilityDisplayActiveRegisters(inst, ui,0, 4, false);
    }
    else  if(inst.epwmXCMP_No_Split == device_peripheral.EPWM_XCMP_ALLOC_CMPA[5].name) //XCMP1-5 to CMPA
    {
        utilityDisplayActiveRegisters(inst, ui,0, 5, false);
    }
    else  if(inst.epwmXCMP_No_Split == device_peripheral.EPWM_XCMP_ALLOC_CMPA[6].name) //XCMP1-6 to CMPA
    {
        utilityDisplayActiveRegisters(inst, ui,0, 6, false);
    }
    else  if(inst.epwmXCMP_No_Split == device_peripheral.EPWM_XCMP_ALLOC_CMPA[7].name) //XCMP1-7 to CMPA
    {
        utilityDisplayActiveRegisters(inst, ui, 0, 7, false);
    }
    else  if(inst.epwmXCMP_No_Split == device_peripheral.EPWM_XCMP_ALLOC_CMPA[8].name) //XCMP1-8 to CMPA
    {
        utilityDisplayActiveRegisters(inst, ui, 0, 8, false);
    }

    utilityDisplayShadowRegisters(inst,ui,false);
}

function onSelectSplit_A(inst, ui)
{
    utilityDisplayActiveRegisters(inst, ui, 0, 4, true);

    if(inst.epwmXCMP_Split_A == device_peripheral.EPWM_XCMP_ALLOC_CMPA[0].name) //No XCMP to CMPA
    {
        //Do Nothing
    }
    else  if(inst.epwmXCMP_Split_A == device_peripheral.EPWM_XCMP_ALLOC_CMPA[1].name) //XCMP1 to CMPA
    {
        utilityDisplayActiveRegisters(inst, ui, 0, 1, false);
    }
    else  if(inst.epwmXCMP_Split_A == device_peripheral.EPWM_XCMP_ALLOC_CMPA[2].name) //XCMP1-2 to CMPA
    {
        utilityDisplayActiveRegisters(inst, ui, 0, 2, false);
    }
    else  if(inst.epwmXCMP_Split_A == device_peripheral.EPWM_XCMP_ALLOC_CMPA[3].name) //XCMP1-3 to CMPA
    {
        utilityDisplayActiveRegisters(inst, ui, 0, 3, false);
    }
    else  if(inst.epwmXCMP_Split_A == device_peripheral.EPWM_XCMP_ALLOC_CMPA[4].name) //XCMP1-4 to CMPA
    {
        utilityDisplayActiveRegisters(inst, ui, 0, 4, false);
    }

    utilityDisplayShadowRegisters(inst,ui,false);
}

function onSelectSplit_B(inst, ui)
{
    utilityDisplayActiveRegisters(inst, ui, 5, 8, true);

    if(inst.epwmXCMP_Split_B == device_peripheral.EPWM_XCMP_ALLOC_CMPB[0].name) //XCMP5 to CMPB
    {
        utilityDisplayActiveRegisters(inst, ui, 4, 5, false);
    }
    else  if(inst.epwmXCMP_Split_B == device_peripheral.EPWM_XCMP_ALLOC_CMPB[1].name) //XCMP5-6 to CMPB
    {
        utilityDisplayActiveRegisters(inst, ui, 4, 6, false);
    }
    else  if(inst.epwmXCMP_Split_B == device_peripheral.EPWM_XCMP_ALLOC_CMPB[2].name) //XCMP5-7 to CMPB
    {
        utilityDisplayActiveRegisters(inst, ui, 4, 7, false);
    }
    else  if(inst.epwmXCMP_Split_B == device_peripheral.EPWM_XCMP_ALLOC_CMPB[3].name) //XCMP5-8 to CMPB
    {
        utilityDisplayActiveRegisters(inst, ui, 4, 8, false);
    }
    utilityDisplayShadowRegisters(inst,ui,false);
}

/* Array of configurables that are common across device families */
let config = [
    {
        name: "epwmXCMP_enableMode",
        displayName : "Enable EPWM XCMP Mode",
        description : 'Check to enable EPWM XCMP Mode Operation',
        hidden      : false,
        default     : false,
        onChange    : onClickEnable
    },

    {
        name: "epwmXCMP_SplitCheck",
        displayName : "Split EPWM XCMP Registers",
        description : 'Split EPWM XCMP Registers between CMPA and CMPB',
        hidden      : true,
        default     : false,
        onChange    : onClickSplit
    },

    {
        name: "epwmXCMP_No_Split",
        displayName : "Allocate All XCMP Registers To CMPA",
        description : 'Allocate All EPWM XCMP Registers To CMPA',
        hidden      : true,
        default     : device_peripheral.EPWM_XCMP_ALLOC_CMPA[0].name,
        options     : device_peripheral.EPWM_XCMP_ALLOC_CMPA,
        onChange    : onSelectNoSplit,
    },

    {
        name: "epwmXCMP_Split_A",
        displayName : "Allocate EPWM XCMP Registers To CMPA",
        description : 'Allocate EPWM XCMP Registers To CMPA',
        hidden      : true,
        default     : device_peripheral.EPWM_XCMP_ALLOC_CMPA[0].name,
        options     : [device_peripheral.EPWM_XCMP_ALLOC_CMPA[0],
                       device_peripheral.EPWM_XCMP_ALLOC_CMPA[1],
                       device_peripheral.EPWM_XCMP_ALLOC_CMPA[2],
                       device_peripheral.EPWM_XCMP_ALLOC_CMPA[3],
                       device_peripheral.EPWM_XCMP_ALLOC_CMPA[4]],
        onChange    : onSelectSplit_A,
    },

    {
        name: "epwmXCMP_Split_B",
        displayName : "Allocate EPWM XCMP Registers To CMPB",
        description : 'Allocate EPWM XCMP Registers To CMPB',
        hidden      : true,
        default     : device_peripheral.EPWM_XCMP_ALLOC_CMPB[0].name,
        options     : [device_peripheral.EPWM_XCMP_ALLOC_CMPB[0],
                       device_peripheral.EPWM_XCMP_ALLOC_CMPB[1],
                       device_peripheral.EPWM_XCMP_ALLOC_CMPB[2],
                       device_peripheral.EPWM_XCMP_ALLOC_CMPB[3]],
        onChange    : onSelectSplit_B,
    },
];

let active_config=[];

for (let i = 1; i < 9; i++)
{
    let XCMP_ACTIVE = {
        name: "epwmXCMP" + i +"_Active",
        displayName : "XCMP" + i +" Active",
        description : "XCMP" + i +" Active value",
        hidden      : true,
        default     : 0,
    }
    active_config.push(XCMP_ACTIVE);
}

let shdw1_config=[];

for (let i = 1; i < 9; i++)
{
    let XCMP_SHADOW1 = {
        name: "epwmXCMP" + i +"_Shdw1",
        displayName : "XCMP" + i +" Shadow 1",
        description : "XCMP" + i +" Shadow 1 value",
        hidden      : true,
        default     : 0,
    }
    shdw1_config.push(XCMP_SHADOW1);
}

let shdw2_config=[];

for (let i = 1; i < 9; i++)
{
    let XCMP_SHADOW2 = {
        name: "epwmXCMP" + i +"_Shdw2",
        displayName : "XCMP" + i +" Shadow 2",
        description : "XCMP" + i +" Shadow 2 value",
        hidden      : true,
        default     : 0,
    }
    shdw2_config.push(XCMP_SHADOW2);
}

let shdw3_config=[];

for (let i = 1; i < 9; i++)
{
    let XCMP_SHADOW3 = {
        name: "epwmXCMP" + i +"_Shdw3",
        displayName : "XCMP" + i +" Shadow 3",
        description : "XCMP" + i +" Shadow 3 value",
        hidden      : true,
        default     : 0,
    }
    shdw3_config.push(XCMP_SHADOW3);
}

let xminmax_config=[];

xminmax_config = xminmax_config.concat([
    {
        name: "epwmXMin_Active",
        displayName : "XMin Active",
        description : 'XMin Active Register value',
        hidden      : true,
        default     : 0,
    }
]);

xminmax_config = xminmax_config.concat([
    {
        name: "epwmXMax_Active",
        displayName : "XMax Active",
        description : 'XMax Active Register value',
        hidden      : true,
        default     : 0,
    }
]);

xminmax_config = xminmax_config.concat([
    {
        name: "epwmXMin_Shdw1",
        displayName : "XMin Shadow 1",
        description : 'XMin Shadow 1 Register value',
        hidden      : true,
        default     : 0,
    }
]);

xminmax_config = xminmax_config.concat([
    {
        name: "epwmXMax_Shdw1",
        displayName : "XMax Shadow 1",
        description : 'XMax Shadow 1 Register value',
        hidden      : true,
        default     : 0,
    }
]);

xminmax_config = xminmax_config.concat([
    {
        name: "epwmXMin_Shdw2",
        displayName : "XMin Shadow 2",
        description : 'XMin Shadow 2 Register value',
        hidden      : true,
        default     : 0,
    }
]);

xminmax_config = xminmax_config.concat([
    {
        name: "epwmXMax_Shdw2",
        displayName : "XMax Shadow 2",
        description : 'XMax Shadow 2 Register value',
        hidden      : true,
        default     : 0,
    }
]);

xminmax_config = xminmax_config.concat([
    {
        name: "epwmXMin_Shdw3",
        displayName : "XMin Shadow 3",
        description : 'XMin Shadow 3 Register value',
        hidden      : true,
        default     : 0,
    }
]);

xminmax_config = xminmax_config.concat([
    {
        name: "epwmXMax_Shdw3",
        displayName : "XMax Shadow 3",
        description : 'XMax Shadow 3 Register value',
        hidden      : true,
        default     : 0,
    }
]);

let xtbprd_config=[];

xtbprd_config = xtbprd_config.concat([
    {
        name: "epwmXTBPRD_Active",
        displayName : "XTbPrd Active",
        description : 'XTbPrd Active Register value',
        hidden      : true,
        default     : 0,
    }
]);

xtbprd_config = xtbprd_config.concat([
    {
        name: "epwmXTBPRD_Shdw1",
        displayName : "XTbPrd Shadow 1",
        description : 'XTbPrd Shadow 1 Register value',
        hidden      : true,
        default     : 0,
    }
]);

xtbprd_config = xtbprd_config.concat([
    {
        name: "epwmXTBPRD_Shdw2",
        displayName : "XTbPrd Shadow 2",
        description : 'XTbPrd Shadow 2 Register value',
        hidden      : true,
        default     : 0,
    }
]);

xtbprd_config = xtbprd_config.concat([
    {
        name: "epwmXTBPRD_Shdw3",
        displayName : "XTbPrd Shadow 3",
        description : 'XTbPrd Shadow 3 Register value',
        hidden      : true,
        default     : 0,
    }
]);


config = config.concat([
    {
    name: "GROUP_ACTIVE",
    displayName: "XCMP ACTIVE",
    description: "",
    longDescription: "",
    config: active_config,
    },
]);

config = config.concat([
    {
    name: "GROUP_SHADOW1",
    displayName: "XCMP SHADOW 1",
    description: "",
    longDescription: "",
    config: shdw1_config,
    },
]);

config = config.concat([
    {
    name: "GROUP_SHADOW2",
    displayName: "XCMP SHADOW 2",
    description: "",
    longDescription: "",
    config: shdw2_config,
    },
]);

config = config.concat([
    {
    name: "GROUP_SHADOW3",
    displayName: "XCMP SHADOW 3",
    description: "",
    longDescription: "",
    config: shdw3_config,
    },
]);

let array = ["ACTIVE","SHADOW_1","SHADOW_2","SHADOW_3"];


for(let index in array)
{
    let str = array[index];
    let aq_config=[];
    for (let aqOutputIndex in device_peripheral.EPWM_ActionQualifierOutputModule)
    {
        let aqOutput = device_peripheral.EPWM_ActionQualifierOutputModule[aqOutputIndex];

        for (let xcmpaqEventIndex in device_peripheral.EPWM_XCMPActionQualifierOutputEvent)
        {
            if(aqOutput.displayName.replace("Output", "") == "EPWMXB ")
                if(xcmpaqEventIndex<4) continue;

            let xcmpaqEvent = device_peripheral.EPWM_XCMPActionQualifierOutputEvent[xcmpaqEventIndex];

            let  XCMP_AQ = {
                name: "epwmXAQCTL_" + aqOutput.name + "_" + xcmpaqEvent.name.replace("EPWM_AQ_OUTPUT_", "")+"_"+str,
                displayName : aqOutput.displayName.replace("Output", "") + xcmpaqEvent.displayName+" "+str,
                description : '',
                hidden      : true,
                default     : device_peripheral.EPWM_ActionQualifierOutput[0].name,
                options     : device_peripheral.EPWM_ActionQualifierOutput,
            };
            aq_config.push(XCMP_AQ);
        }
    }
    config = config.concat([
        {
        name: "GROUP_AQ"+str,
        displayName: "XCMP ACTION QUALIFIER "+str,
        description: "",
        longDescription: "",
        config: aq_config,
        },
    ]);
}

config = config.concat([
    {
    name: "GROUP_XMINMAX",
    displayName: "XMINMAX",
    description: "",
    longDescription: "",
    config: xminmax_config,
    },
]);

config = config.concat([
    {
    name: "GROUP_XTBPRD",
    displayName: "XTBPRD",
    description: "",
    longDescription: "",
    config: xtbprd_config,
    },
]);

// XLOADCTL
function onSelectLoadMode(inst, ui)
{
    if(inst.epwmXLOADCTL_Loadmode == device_peripheral.EPWM_XCMPXloadCtlLoadMode[0].name)
    {
        ui["epwmXLOADCTL_Loadonce"].hidden = false;
    }
    else if(inst.epwmXLOADCTL_Loadmode == device_peripheral.EPWM_XCMPXloadCtlLoadMode[1].name)
    {
        ui["epwmXLOADCTL_Loadonce"].hidden = true;
        //Resetting to default value. For indices refer the comment at top of file
        inst["epwmXLOADCTL_Loadonce"] = device_peripheral.EPWM_XCMP_XLOADCTL_SHDWBUFPTR[0].name;
    }
}

function onSelectShadowLevel(inst, ui)
{
    for(let i=6; i<9 || (i>9 && i<13); i++)
        for(let j=0;j<8;j++)
            ui[config[i].config[j].name].hidden = true;

    utilityDisplayShadowRegisters(inst, ui, false);
}

config = config.concat([
    {
        name: "GROUP_XLOADCTL",
        displayName: "XLOADCTL",
        longDescription: "",
        config: [
            {
                name: "epwmXLOADCTL_Loadmode",
                displayName: "Load Mode",
                description: "Load mode selection for Shadow registers",
                hidden      : true,
                default     : device_peripheral.EPWM_XCMPXloadCtlLoadMode[0].name,
                options     : device_peripheral.EPWM_XCMPXloadCtlLoadMode,
                onChange    : onSelectLoadMode,
            },
            {
                name: "epwmXLOADCTL_Shadowlevel",
                displayName: "Shadow Level",
                description: "Shadow Register Level Allocation Options",
                hidden      : true,
                default     : device_peripheral.EPWM_XCMP_XLOADCTL_SHDWLEVEL[0].name,
                options     : device_peripheral.EPWM_XCMP_XLOADCTL_SHDWLEVEL,
                onChange    : onSelectShadowLevel,
            },
            {
                name: "epwmXLOADCTL_Loadonce",
                displayName: "Shadow Register Set Load Once",
                description: "Register Load event count",
                hidden      : true,
                default     : device_peripheral.EPWM_XCMP_XLOADCTL_SHDWBUFPTR[0].name,
                options     : device_peripheral.EPWM_XCMP_XLOADCTL_SHDWBUFPTR,
            },
            {
                name: "epwmXLOADCTL_RepeatBuf2",
                displayName: "Repeat Count Shadow Buffer 2",
                description: "Repeat Count Shadow Buffer 2",
                hidden      : true,
                default     : 0,
            },
            {
                name: "epwmXLOADCTL_RepeatBuf3",
                displayName: "Repeat Count Shadow Buffer 3",
                description: "Repeat Count Shadow Buffer 3",
                hidden      : true,
                default     : 0,
            }
        ]
    }
])

let epwmXCMPSubmodule = {
    displayName: "EPWM XCMP",
    // maxInstances: common.peripheralCount("EPWM"),
    defaultInstanceName: "EPWM_XCMP",
    description: "Enhanced Pulse Width Modulator XCMP Mode Operation",
    config: config,
    templates: {
        boardc : "",
        boardh : ""
    },
}


exports = epwmXCMPSubmodule;