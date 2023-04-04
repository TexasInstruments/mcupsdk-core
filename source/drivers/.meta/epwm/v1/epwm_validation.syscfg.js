
let validation_warning = "warning"
let validation_error = "error"
let validation_info = "info"

let epwm_validation = [
	// Validation #1
	{
		type : validation_warning,
		name :  "CMP value is out of range of TBPRD. Refer to table, Behavior if CMPA/CMPB is Greater than the Period, within the TRM",
		func : (inst, validation, name) => {
			if (inst["epwmTimebase_period"] <
				inst["epwmCounterCompare_cmpA"])
			{
				validation.logWarning(name, inst, "epwmCounterCompare_cmpA");
			}
			if (inst["epwmTimebase_period"] <
				inst["epwmCounterCompare_cmpB"])
			{
				validation.logWarning(name, inst, "epwmCounterCompare_cmpB");
			}
			if (inst["epwmTimebase_period"] <
				inst["epwmCounterCompare_cmpC"])
			{
				validation.logWarning(name, inst, "epwmCounterCompare_cmpC");
			}
			if (inst["epwmTimebase_period"] <
				inst["epwmCounterCompare_cmpD"])
			{
				validation.logWarning(name, inst, "epwmCounterCompare_cmpD");
			}

		},
			devices : [
                "am263x"
			]
		},

    // Validation #2
	{
		type : validation_warning,
		name :  "It is recommended to use a non-zero counter compare value when using shadow to active load of action qualifier A/B control register on TBCTR=0 boundary",
		func : (inst, validation, name) => {
			if ((inst["epwmCounterCompare_cmpA"] == 0  || inst["epwmCounterCompare_cmpB"] == 0)
				&& inst["epwmActionQualifier_EPWM_AQ_OUTPUT_A_shadowMode"] == 1
				&& (inst["epwmActionQualifier_EPWM_AQ_OUTPUT_A_shadowEvent"] == "EPWM_AQ_LOAD_ON_CNTR_ZERO" ||
					inst["epwmActionQualifier_EPWM_AQ_OUTPUT_A_shadowEvent"] == "EPWM_AQ_LOAD_ON_CNTR_ZERO_PERIOD" ||
					inst["epwmActionQualifier_EPWM_AQ_OUTPUT_A_shadowEvent"] == "EPWM_AQ_LOAD_ON_SYNC_CNTR_ZERO" ||
					inst["epwmActionQualifier_EPWM_AQ_OUTPUT_A_shadowEvent"] == "EPWM_AQ_LOAD_ON_SYNC_CNTR_ZERO_PERIOD"))
			{
				validation.logWarning(name, inst, "epwmCounterCompare_cmpA");
				validation.logWarning(name, inst, "epwmCounterCompare_cmpB");
			}
			if ((inst["epwmCounterCompare_cmpA"] == 0  || inst["epwmCounterCompare_cmpB"] == 0)
				&& inst["epwmActionQualifier_EPWM_AQ_OUTPUT_B_shadowMode"] == 1
				&& (inst["epwmActionQualifier_EPWM_AQ_OUTPUT_B_shadowEvent"] == "EPWM_AQ_LOAD_ON_CNTR_ZERO" ||
					inst["epwmActionQualifier_EPWM_AQ_OUTPUT_B_shadowEvent"] == "EPWM_AQ_LOAD_ON_CNTR_ZERO_PERIOD" ||
					inst["epwmActionQualifier_EPWM_AQ_OUTPUT_B_shadowEvent"] == "EPWM_AQ_LOAD_ON_SYNC_CNTR_ZERO" ||
					inst["epwmActionQualifier_EPWM_AQ_OUTPUT_B_shadowEvent"] == "EPWM_AQ_LOAD_ON_SYNC_CNTR_ZERO_PERIOD"))
			{
				validation.logWarning(name, inst, "epwmCounterCompare_cmpA");
				validation.logWarning(name, inst, "epwmCounterCompare_cmpB");
			}

		},
			devices : [
                "am263x"
			]
		},

	// Validation #3
	{
		type : validation_warning,
		name :  "Unused action set for out of range event",
		func : (inst, validation, name) => {
			let noChangeOutput = "EPWM_AQ_OUTPUT_NO_CHANGE";
			let epwm_outX = ["A", "B"];
			if (inst["epwmTimebase_counterMode"] == "EPWM_COUNTER_MODE_DOWN")
			{
				for (let epwm_out of epwm_outX)
				{
					if (inst["epwmActionQualifier_EPWM_AQ_OUTPUT_" + epwm_out + "_ON_TIMEBASE_UP_CMPA"]
						!= noChangeOutput)
					{
						validation.logWarning(name, inst, "epwmActionQualifier_EPWM_AQ_OUTPUT_" + epwm_out + "_ON_TIMEBASE_UP_CMPA");
					}
					if (inst["epwmActionQualifier_EPWM_AQ_OUTPUT_" + epwm_out + "_ON_TIMEBASE_UP_CMPB"]
						!= noChangeOutput)
					{
						validation.logWarning(name, inst, "epwmActionQualifier_EPWM_AQ_OUTPUT_" + epwm_out + "_ON_TIMEBASE_UP_CMPB");
					}
					if (inst["epwmActionQualifier_EPWM_AQ_OUTPUT_" + epwm_out + "_ON_T1_COUNT_UP"]
						!= noChangeOutput)
					{
						validation.logWarning(name, inst, "epwmActionQualifier_EPWM_AQ_OUTPUT_" + epwm_out + "_ON_T1_COUNT_UP");
					}
				}
			}
			else if (inst["epwmTimebase_counterMode"] == "EPWM_COUNTER_MODE_UP")
			{
				for (let epwm_out of epwm_outX)
				{
					if (inst["epwmActionQualifier_EPWM_AQ_OUTPUT_" + epwm_out + "_ON_TIMEBASE_DOWN_CMPA"]
						!= noChangeOutput)
					{
						validation.logWarning(name, inst, "epwmActionQualifier_EPWM_AQ_OUTPUT_" + epwm_out + "_ON_TIMEBASE_DOWN_CMPA");
					}
					if (inst["epwmActionQualifier_EPWM_AQ_OUTPUT_" + epwm_out + "_ON_TIMEBASE_DOWN_CMPB"]
						!= noChangeOutput)
					{
						validation.logWarning(name, inst, "epwmActionQualifier_EPWM_AQ_OUTPUT_" + epwm_out + "_ON_TIMEBASE_DOWN_CMPB");
					}
					if (inst["epwmActionQualifier_EPWM_AQ_OUTPUT_" + epwm_out + "_ON_T1_COUNT_DOWN"]
						!= noChangeOutput)
					{
						validation.logWarning(name, inst, "epwmActionQualifier_EPWM_AQ_OUTPUT_" + epwm_out + "_ON_T1_COUNT_DOWN");
					}
				}
			}

		},
			devices : [
                "am263x"
			]
		},

	// Validation #4
	{
		type : validation_error,
		name :  "Input value is out of range (16-bit value)",
		func : (inst, validation, name) => {
			if (inst["epwmCounterCompare_cmpA"] < 0 ||
				inst["epwmCounterCompare_cmpA"] > 65535)
			{
				validation.logError(name, inst, "epwmCounterCompare_cmpA");
			}
			if (inst["epwmCounterCompare_cmpB"] < 0 ||
				inst["epwmCounterCompare_cmpB"] > 65535)
			{
				validation.logError(name, inst, "epwmCounterCompare_cmpB");
			}
			if (inst["epwmCounterCompare_cmpC"] < 0 ||
				inst["epwmCounterCompare_cmpC"] > 65535)
			{
				validation.logError(name, inst, "epwmCounterCompare_cmpC");
			}
			if (inst["epwmCounterCompare_cmpD"] < 0 ||
				inst["epwmCounterCompare_cmpD"] > 65535)
			{
				validation.logError(name, inst, "epwmCounterCompare_cmpD");
			}
			// Digital Compare
			if (inst["epwmDigitalCompare_blankingWindowOffset"] < 0 ||
				inst["epwmDigitalCompare_blankingWindowOffset"] > 65535)
			{
				validation.logError(name, inst, "epwmDigitalCompare_blankingWindowOffset");
			}
			if (inst["epwmDigitalCompare_blankingWindowLength"] < 0 ||
				inst["epwmDigitalCompare_blankingWindowLength"] > 65535)
			{
				validation.logError(name, inst, "epwmDigitalCompare_blankingWindowLength");
			}
			if (inst["epwmDigitalCompare_SWVDELVAL"] < 0 ||
				inst["epwmDigitalCompare_SWVDELVAL"] > 65535)
			{
				validation.logError(name, inst, "epwmDigitalCompare_SWVDELVAL");
			}
			// Time Base
			if (inst["epwmTimebase_counterValue"] < 0 ||
				inst["epwmTimebase_counterValue"] > 65535)
			{
				validation.logError(name, inst, "epwmTimebase_counterValue");
			}
			if (inst["epwmTimebase_period"] < 0 ||
				inst["epwmTimebase_period"] > 65535)
			{
				validation.logError(name, inst, "epwmTimebase_period");
			}
			if (inst["epwmTimebase_phaseShift"] < 0 ||
				inst["epwmTimebase_phaseShift"] > 65535)
			{
				validation.logError(name, inst, "epwmTimebase_phaseShift");
			}
            if (inst["epwmDE_Threshold"] < 0 ||
                inst["epwmDE_Threshold"] > 65535)
			{
		         validation.logError(name, inst, "epwmDE_Threshold");
			}
            if (inst["epwmMDB_delayMDLA"] < 0 ||
                inst["epwmMDB_delayMDLA"] > 65535)
			{
		         validation.logError(name, inst, "epwmMDB_delayMDLA");
			}
            if (inst["epwmMDB_delayMDLB"] < 0 ||
                inst["epwmMDB_delayMDLB"] > 65535)
			{
		         validation.logError(name, inst, "epwmMDB_delayMDLB");
			}

            /* --------------- XCMP --------------- */
            let register_set = ["_Active","_Shdw1","_Shdw2","_Shdw3"];

            for(let i = 1; i< 9; i++)  //XCMP[1-8]
            {
                for(let j = 0; j< 4; j++)
                {
                    let str = "epwmXCMP" + i + register_set[j];
                    if (inst[str] < 0 || inst[str] > 65535)
                        {
                            validation.logError(name, inst, str);
                        }
                }
            }

            let array = ["epwmXMin", "epwmXMax", "epwmXTBPRD"];

            for(let i = 0; i< array.length; i++)
            {
                for(let j = 0; j< 4; j++)
                {
                    let str = array[i] + register_set[j];
                    if (inst[str] < 0 || inst[str] > 65535)
                        {
                            validation.logError(name, inst, str);
                        }
                }
            }
            /* --------------- ---- --------------- */
		},
		devices : [
                "am263x"
			]
		},

    // Validation #5
	{
		type : validation_error,
		name :  "Input value is out of range (14-bit value)",
		func : (inst, validation, name) => {
			if (inst["epwmDeadband_delayRED"] < 0 ||
				inst["epwmDeadband_delayRED"] > 16384)
			{
				validation.logError(name, inst, "epwmDeadband_delayRED");
			}
			if (inst["epwmDeadband_delayFED"] < 0 ||
				inst["epwmDeadband_delayFED"] > 16384)
			{
				validation.logError(name, inst, "epwmDeadband_delayFED");
			}
		},
		devices : [
                "am263x"
			]
		},

	// Validation #6
	{
		type : validation_error,
		name :  "Same event may not be chosen for the DC filter input source and the valley capture signal source",
		func : (inst, validation, name) => {
			if ((inst["epwmDigitalCompare_dcFilterInput"] == "EPWM_DC_WINDOW_SOURCE_DCAEVT1" && inst["epwmDigitalCompare_valleyCaptureSource"] == "EPWM_VALLEY_TRIGGER_EVENT_DCAEVT1") ||
		        (inst["epwmDigitalCompare_dcFilterInput"] == "EPWM_DC_WINDOW_SOURCE_DCAEVT2" && inst["epwmDigitalCompare_valleyCaptureSource"] == "EPWM_VALLEY_TRIGGER_EVENT_DCAEVT2") ||
		        (inst["epwmDigitalCompare_dcFilterInput"] == "EPWM_DC_WINDOW_SOURCE_DCBEVT1" && inst["epwmDigitalCompare_valleyCaptureSource"] == "EPWM_VALLEY_TRIGGER_EVENT_DCBEVT1") ||
		        (inst["epwmDigitalCompare_dcFilterInput"] == "EPWM_DC_WINDOW_SOURCE_DCBEVT2" && inst["epwmDigitalCompare_valleyCaptureSource"] == "EPWM_VALLEY_TRIGGER_EVENT_DCBEVT2"))
		    {
		         validation.logError(name, inst, "epwmDigitalCompare_dcFilterInput");
		         validation.logError(name, inst, "epwmDigitalCompare_valleyCaptureSource");
		    }

		},
		devices : [
                "am263x"
			]
		},

    // Validation #7
	{
		type : validation_info,
		name :  "For the condition to remain latched, a minimum of 3*TBCLK sync pulse width is required.",
		func : (inst, validation, name) => {
			if(inst["epwmTripZone_oneShotSource"].length >= 1)
			{
				validation.logInfo(name, inst, "epwmTripZone_oneShotSource");
			}
			if(inst["epwmTripZone_cbcSource"].length >= 1)
			{
				validation.logInfo(name, inst, "epwmTripZone_cbcSource");
			}
		},
		devices : [
                "am263x"
			]
		},

	// Validation #8
	{
		type : validation_info,
		name :  "For the condition to remain latched, a minimum of 3*TBCLK sync pulse width is required.",
		func : (inst, validation, name) => {
			if(inst["epwmDigitalCompare_EPWM_DC_MODULE_A_EPWM_DC_EVENT_1_latchMode"] == "EPWM_DC_CBC_LATCH_ENABLED")
			{
				validation.logInfo(name, inst, "epwmDigitalCompare_EPWM_DC_MODULE_A_EPWM_DC_EVENT_1_latchMode");
			}
			if(inst["epwmDigitalCompare_EPWM_DC_MODULE_A_EPWM_DC_EVENT_2_latchMode"] == "EPWM_DC_CBC_LATCH_ENABLED")
			{
				validation.logInfo(name, inst, "epwmDigitalCompare_EPWM_DC_MODULE_A_EPWM_DC_EVENT_2_latchMode");
			}
			if(inst["epwmDigitalCompare_EPWM_DC_MODULE_B_EPWM_DC_EVENT_1_latchMode"] == "EPWM_DC_CBC_LATCH_ENABLED")
			{
				validation.logInfo(name, inst, "epwmDigitalCompare_EPWM_DC_MODULE_B_EPWM_DC_EVENT_1_latchMode");
			}
			if(inst["epwmDigitalCompare_EPWM_DC_MODULE_B_EPWM_DC_EVENT_2_latchMode"] == "EPWM_DC_CBC_LATCH_ENABLED")
			{
				validation.logInfo(name, inst, "epwmDigitalCompare_EPWM_DC_MODULE_B_EPWM_DC_EVENT_2_latchMode");
			}
		},
		devices : [
                "am263x"
			]
		},

    // Validation #9
	{
		type : validation_info,
		name :  "If the blanking window crosses the CTR = 0 or CTR = PRD boundary, the next window still starts at the same offset value after the CTR = 0 or CTR = PRD pulse.",
		func : (inst, validation, name) => {
			if (inst["epwmDigitalCompare_useBlankingWindow"] == 1)
			{
		         validation.logInfo(name, inst, "epwmDigitalCompare_blankingWindowOffset");
			}
		},
		devices : [
                "am263x"
			]
		},

	// Validation #10
	{
		type : validation_error,
		name :  "If RED/FED both applied to output B, output A will be invalid unless output A is EPWMxA (S1) or output A is Bpath (S6)",
		func : (inst, validation, name) => {
			if (inst["epwmDeadband_inputFED"] == "EPWM_DB_INPUT_DB_RED" && (inst["epwmDeadband_outputSwapOutA"] != true && inst["epwmDeadband_enableRED"] == true))
			{
		         validation.logError(name, inst, "epwmDeadband_inputFED");
			}
		},
		devices : [
                "am263x"
			]
		},

    // Validation #11
	{
		type : validation_warning,
		name :  "Clearing the phase shift value will cause the EPWM module to ignore the synchronization input pulse, if any.",
		func : (inst, validation, name) => {
			if (inst["epwmTimebase_phaseEnable"] == 1 && inst["epwmTimebase_phaseShift"] == 0)
			{
		         validation.logWarning(name, inst, "epwmTimebase_phaseShift");
			}

		},
		devices : [
                "am263x"
			]
		},

    // Validation #12
	{
		type : validation_info,
		name :  "For perfectly synchronized TBCLKs across multiple EPWM modules, the prescaler bits in the TBCTL register of each EPWM module must be set identically",
		func : (inst, validation, name) => {
		        validation.logInfo(name, inst, "epwmTimebase_clockDiv");

		},
		devices : [
                "am263x"
			]
		},

    // Validation #13
	{
		type : validation_info,
		name :  "Shadow to active load event selection bits for individual shadowed registers will be ignored and global load mode takes effect for registers with global load enabled",
		func : (inst, validation, name) => {
			if (inst["epwmGlobalLoad_gld"] == 1)
			{
		        validation.logInfo(name, inst, "epwmGlobalLoad_gld");
		    }

		},
		devices : [
                "am263x"
			]
		},

	// Validation #14
	{
		type : validation_warning,
		name :  "The SYNCEVT signal is only propagated through when PHSEN is SET.",
		func : (inst, validation, name) => {
			if (inst["epwmGlobalLoad_gld"] == 1 && (inst["epwmGlobalLoad_gldMode"] == "EPWM_GL_LOAD_PULSE_SYNC" || inst["epwmGlobalLoad_gldMode"] == "EPWM_GL_LOAD_PULSE_SYNC_OR_CNTR_ZERO"
				|| inst["epwmGlobalLoad_gldMode"] == "EPWM_GL_LOAD_PULSE_SYNC_OR_CNTR_PERIOD" || inst["epwmGlobalLoad_gldMode"] == "EPWM_GL_LOAD_PULSE_SYNC_CNTR_ZERO_PERIOD") &&
				inst["epwmTimebase_phaseEnable"] == 1 && inst["epwmTimebase_phaseShift"] == 0)
			{
		        validation.logWarning(name, inst, "epwmGlobalLoad_gldMode");
		    }

		},
		devices : [
                "am263x"
			]
		},

	// Validation #15
	{
		type : validation_info,
		name :  "This counter compare value can only be used to generate an event in the event trigger submodule",
		func : (inst, validation, name) => {
			if (inst["epwmCounterCompare_cmpC"] != 0)
			{
		        validation.logInfo(name, inst, "epwmCounterCompare_cmpC");
		    }
		    if (inst["epwmCounterCompare_cmpD"] != 0)
			{
		        validation.logInfo(name, inst, "epwmCounterCompare_cmpD");
		    }

		},
		devices : [
                "am263x"
			]
		},

	// Validation #16
	{
		type : validation_info,
		name :  "When PWMxB is derived from PWMxA using the DEDB_MODE bit and by delaying rising edge and falling edge by the phase shift amount, if the duty cycle value on PWMxA is less than this phase shift amount, PWMxA’s falling edge has precedence over the delayed rising edge for PWMxB. It is recommended to make sure the duty cycle value of the current waveform fed to the deadband module is greater than the required phase shift amount.",
		func : (inst, validation, name) => {
			if (inst["epwmDeadband_inputRED"] == "EPWM_DB_INPUT_EPWMA" && inst["epwmDeadband_inputFED"] == "EPWM_DB_INPUT_DB_RED" && inst["epwmDeadband_enableRED"] == 1 && inst["epwmDeadband_delayRED"] != 0)
			{
		        validation.logInfo(name, inst, "epwmDeadband_delayRED");
		    }
		},
		devices : [
                "am263x"
			]
		},

	// Validation #17
	{
		type : validation_warning,
		name :  "If the CBC interrupt is enabled and DCAEVT2 or DCBEVT2 are selected as CBC trip sources, it is not necessary to also enable the DCAEVT2/DCBEVT2 interrupts.",
		func : (inst, validation, name) => {
			if (((inst["epwmTripZone_cbcSource"].includes("EPWM_TZ_SIGNAL_DCAEVT2") || inst["epwmTripZone_cbcSource"].includes("EPWM_TZ_SIGNAL_DCBEVT2")) &&
				(inst["epwmTripZone_tzInterruptSource"].includes("EPWM_TZ_INTERRUPT_DCAEVT2") || inst["epwmTripZone_tzInterruptSource"].includes("EPWM_TZ_INTERRUPT_DCBEVT2")) &&
				inst["epwmTripZone_tzInterruptSource"].includes("EPWM_TZ_INTERRUPT_CBC")))

			{
		        validation.logWarning(name, inst, "epwmTripZone_tzInterruptSource");
		    }
		},
		devices : [
                "am263x"
			]
		},

	// Validation #18
	{
		type : validation_warning,
		name :  "If the OSHT interrupt is enabled and DCAEVT1 or DCBEVT1 are selected as OSHT trip sources, it is not necessary to also enable the DCAEVT1/DCBEVT1 interrupts.",
		func : (inst, validation, name) => {
			if (((inst["epwmTripZone_oneShotSource"].includes("EPWM_TZ_SIGNAL_DCAEVT1") || inst["epwmTripZone_oneShotSource"].includes("EPWM_TZ_SIGNAL_DCBEVT1")) &&
				(inst["epwmTripZone_tzInterruptSource"].includes("EPWM_TZ_INTERRUPT_DCAEVT1") || inst["epwmTripZone_tzInterruptSource"].includes("EPWM_TZ_INTERRUPT_DCBEVT1")) &&
				inst["epwmTripZone_tzInterruptSource"].includes("EPWM_TZ_INTERRUPT_OST")))

			{
		        validation.logWarning(name, inst, "epwmTripZone_tzInterruptSource");
		    }
		},
		devices : [
                "am263x"
			]
		},

	// Validation #19
	{
		type : validation_error,
		name :  "Count down direction with a phase value of 0 and in up-down counter mode is invalid.",
		func : (inst, validation, name) => {
			if (inst["epwmTimebase_phaseEnable"] == true && inst["epwmTimebase_phaseShift"] == 0 && inst["epwmTimebase_counterMode"] == "EPWM_COUNTER_MODE_UP_DOWN"
				&& inst["epwmTimebase_counterModeAfterSync"] == "EPWM_COUNT_MODE_DOWN_AFTER_SYNC")
			{
		         validation.logError(name, inst, "epwmTimebase_counterMode");
			}

		},
		devices : [
                "am263x"
			]
		},

	// Validation #20
	{
		type : validation_info,
		name :  "T1/T2 selection and configuration of a trip/digital-compare event is indpendent of the configuration of that event in the Trip-Zone submodule",
		func : (inst, validation, name) => {
			if (inst["epwmActionQualifier_t1Source"] != "EPWM_AQ_TRIGGER_EVENT_TRIG_EPWM_SYNCIN")
			{
		         validation.logInfo(name, inst, "epwmActionQualifier_t1Source");
			}
			if (inst["epwmActionQualifier_t2Source"] != "EPWM_AQ_TRIGGER_EVENT_TRIG_EPWM_SYNCIN")
			{
		         validation.logInfo(name, inst, "epwmActionQualifier_t2Source");
			}
		},
		devices : [
                "am263x"
			]
		},

	// Validation #21
	{
		type : validation_warning,
		name :  "PWM Chopper will not be enabled when high-resolution dead-band is enabled",
		func : (inst, validation, name) => {
			if (inst["epwmChopper_useChopper"] ==1)
			{
		         validation.logWarning(name, inst, "epwmChopper_useChopper");
			}
		},
		devices : [
                "am263x"
			]
		},

	// Validation #22
	{
		type : validation_info,
		name :  "When DBRED/DBFED active is loaded with a new shadow value while DB counters are counting, the new DBRED / DBFED value only affects the NEXT PWMx edge and not the current edge.",
		func : (inst, validation, name) => {
			if (inst["epwmDeadband_redShadowMode"] ==1)
			{
		         validation.logInfo(name, inst, "epwmDeadband_redShadowMode");
			}
			if (inst["epwmDeadband_fedShadowMode"] ==1)
			{
		         validation.logInfo(name, inst, "epwmDeadband_fedShadowMode");
			}
		},
		devices : [
                "am263x"
			]
		},

	// Validation #23
	{
		type : validation_error,
		name :  "A deadband value of ZERO should not be used when the Global Shadow to Active Load is set to occur at CTR=ZERO",
		func : (inst, validation, name) => {
			if (inst["epwmDeadband_enableRED"] == 1 && inst["epwmDeadband_delayRED"] == 0 && inst["epwmDeadband_redGld"] ==1
				&& (inst["epwmGlobalLoad_gldMode"] == "EPWM_GL_LOAD_PULSE_CNTR_ZERO" ||
				inst["epwmGlobalLoad_gldMode"] == "EPWM_GL_LOAD_PULSE_CNTR_ZERO_PERIOD" ||
				inst["epwmGlobalLoad_gldMode"] == "EPWM_GL_LOAD_PULSE_SYNC_OR_CNTR_ZERO" ||
				inst["epwmGlobalLoad_gldMode"] == "EPWM_GL_LOAD_PULSE_SYNC_CNTR_ZERO_PERIOD"))
			{
		         validation.logError(name, inst, "epwmDeadband_delayRED");
			}
			if (inst["epwmDeadband_enableFED"] == 1 && inst["epwmDeadband_delayFED"] == 0 && inst["epwmDeadband_fedGld"] ==1
				&& (inst["epwmGlobalLoad_gldMode"] == "EPWM_GL_LOAD_PULSE_CNTR_ZERO" ||
				inst["epwmGlobalLoad_gldMode"] == "EPWM_GL_LOAD_PULSE_CNTR_ZERO_PERIOD" ||
				inst["epwmGlobalLoad_gldMode"] == "EPWM_GL_LOAD_PULSE_SYNC_OR_CNTR_ZERO" ||
				inst["epwmGlobalLoad_gldMode"] == "EPWM_GL_LOAD_PULSE_SYNC_CNTR_ZERO_PERIOD"))
			{
		         validation.logError(name, inst, "epwmDeadband_delayFED");
			}
		},
		devices : [
                "am263x"
			]
		},

	// Validation #24
	{
		type : validation_error,
		name :  "A deadband value of PERIOD should not be used when the Global Shadow to Active Load is set to occur at CTR=PRD",
		func : (inst, validation, name) => {
			if (inst["epwmDeadband_enableRED"] == 1 && inst["epwmDeadband_delayRED"] == inst["epwmTimebase_period"] && inst["epwmDeadband_redGld"] ==1
				&& (inst["epwmGlobalLoad_gldMode"] == "EPWM_GL_LOAD_PULSE_CNTR_PERIOD" ||
				inst["epwmGlobalLoad_gldMode"] == "EPWM_GL_LOAD_PULSE_CNTR_ZERO_PERIOD" ||
				inst["epwmGlobalLoad_gldMode"] == "EPWM_GL_LOAD_PULSE_SYNC_OR_CNTR_PERIOD" ||
				inst["epwmGlobalLoad_gldMode"] == "EPWM_GL_LOAD_PULSE_SYNC_CNTR_ZERO_PERIOD"))
			{
		         validation.logError(name, inst, "epwmDeadband_delayRED");
			}
			if (inst["epwmDeadband_enableFED"] == 1 && inst["epwmDeadband_delayFED"] == inst["epwmTimebase_period"] && inst["epwmDeadband_fedGld"] ==1
				&& (inst["epwmGlobalLoad_gldMode"] == "EPWM_GL_LOAD_PULSE_CNTR_PERIOD" ||
				inst["epwmGlobalLoad_gldMode"] == "EPWM_GL_LOAD_PULSE_CNTR_ZERO_PERIOD" ||
				inst["epwmGlobalLoad_gldMode"] == "EPWM_GL_LOAD_PULSE_SYNC_OR_CNTR_PERIOD" ||
				inst["epwmGlobalLoad_gldMode"] == "EPWM_GL_LOAD_PULSE_SYNC_CNTR_ZERO_PERIOD"))
			{
		         validation.logError(name, inst, "epwmDeadband_delayFED");
			}
		},
		devices : [
                "am263x"
			]
		},

	// Validation #25
	{
		type : validation_info,
		name :  "The one-shot trip condition must be cleared manually by writing to the TZCLR[OST] bit.",
		func : (inst, validation, name) => {
			if (inst["epwmTripZone_oneShotSource"] !=0)
			{
		         validation.logInfo(name, inst, "epwmTripZone_oneShotSource");
			}
		},
		devices : [
                "am263x"
			]
		},

	// Validation #26
	{
		type : validation_error,
		name :  "The STOPEDGE value must be greater than the STARTEDGE value",
		func : (inst, validation, name) => {
			if (parseInt(inst["epwmDigitalCompare_stopValleyCaptureTriggerCount"]) <  parseInt(inst["epwmDigitalCompare_startValleyCaptureTriggerCount"]))
			{
		         validation.logError(name, inst, "epwmDigitalCompare_stopValleyCaptureTriggerCount");
			}
		},
		devices : [
                "am263x"
			]
		},

	// Validation #27
	{
		type : validation_warning,
		name :  "Errata Advisory: An ePWM glitch can occur if a trip remains active at the end of the Blanking Window",
		func : (inst, validation, name) => {
			if (inst["epwmDigitalCompare_useBlankingWindow"] && (inst["epwmTripZone_oneShotSource"] == "EPWM_TZ_SIGNAL_DCAEVT1" ||
				inst["epwmTripZone_oneShotSource"] == "EPWM_TZ_SIGNAL_DCBEVT1"))
			{
		         validation.logWarning(name, inst, "epwmTripZone_oneShotSource");
			}
			if (inst["epwmDigitalCompare_useBlankingWindow"] && (inst["epwmTripZone_cbcSource"] == "EPWM_TZ_SIGNAL_DCAEVT2" ||
				inst["epwmTripZone_oneShotSource"] == "EPWM_TZ_SIGNAL_DCBEVT2"))
			{
		         validation.logWarning(name, inst, "epwmTripZone_cbcSource");
			}
		},
		devices : [
                "am263x"
			]
		},

	// Validation #28
	{
		type : validation_error,
		name :  "Errata Advisory: ePWM Dead-Band Delay Value Cannot be Set to 0 When Using Shadow Load Mode for RED/FED",
		func : (inst, validation, name) => {
			if (inst["epwmDeadband_enableRED"] && inst["epwmDeadband_delayRED"] == "0" && inst["epwmDeadband_redShadowMode"])
			{
		         validation.logError(name, inst, "epwmDeadband_delayRED");
			}
			if (inst["epwmDeadband_enableFED"] && inst["epwmDeadband_delayFED"] == "0" && inst["epwmDeadband_fedShadowMode"])
			{
		         validation.logError(name, inst, "epwmDeadband_delayFED");
			}
		},
		devices : [
                "am263x"
			]
		},

	// Validation #29
	{
		type : validation_warning,
		name :  "Errata Advisory: Trip Events Will Not be Filtered by the Blanking Window for the First 3 Cycles After the Start of a Blanking Window",
		func : (inst, validation, name) => {
			if (inst["epwmDigitalCompare_useBlankingWindow"] == 1)
			{
		         validation.logWarning(name, inst, "epwmDigitalCompare_blankingWindowOffset");
			}
		},
		devices : [
                "am263x"
			]
		},

	// Validation #30
	{
		type : validation_info,
		name :  "Errata Advisory: Event Latch(DCxEVTxLAT) of DC Event-Based CBC Trip May not Extend Trigger Pulse as Expected When Asynchronous Path is Selected",
		func : (inst, validation, name) => {
			if (inst["epwmDigitalCompare_EPWM_DC_MODULE_A_EPWM_DC_EVENT_1_latchMode"] == "EPWM_DC_CBC_LATCH_ENABLED" && inst["epwmDigitalCompare_EPWM_DC_MODULE_A_EPWM_DC_EVENT_1_eventSync"] == "EPWM_DC_EVENT_INPUT_NOT_SYNCED")
			{
				validation.logInfo(name, inst, "epwmDigitalCompare_EPWM_DC_MODULE_A_EPWM_DC_EVENT_1_latchMode");
			}
			if (inst["epwmDigitalCompare_EPWM_DC_MODULE_A_EPWM_DC_EVENT_2_latchMode"] == "EPWM_DC_CBC_LATCH_ENABLED" && inst["epwmDigitalCompare_EPWM_DC_MODULE_A_EPWM_DC_EVENT_2_eventSync"] == "EPWM_DC_EVENT_INPUT_NOT_SYNCED")
			{
				validation.logInfo(name, inst, "epwmDigitalCompare_EPWM_DC_MODULE_A_EPWM_DC_EVENT_2_latchMode");
			}
			if (inst["epwmDigitalCompare_EPWM_DC_MODULE_B_EPWM_DC_EVENT_1_latchMode"] == "EPWM_DC_CBC_LATCH_ENABLED" && inst["epwmDigitalCompare_EPWM_DC_MODULE_B_EPWM_DC_EVENT_1_eventSync"] == "EPWM_DC_EVENT_INPUT_NOT_SYNCED")
			{
				validation.logInfo(name, inst, "epwmDigitalCompare_EPWM_DC_MODULE_B_EPWM_DC_EVENT_1_latchMode");
			}
			if (inst["epwmDigitalCompare_EPWM_DC_MODULE_B_EPWM_DC_EVENT_2_latchMode"] == "EPWM_DC_CBC_LATCH_ENABLED" && inst["epwmDigitalCompare_EPWM_DC_MODULE_B_EPWM_DC_EVENT_2_eventSync"] == "EPWM_DC_EVENT_INPUT_NOT_SYNCED")
			{
				validation.logInfo(name, inst, "epwmDigitalCompare_EPWM_DC_MODULE_B_EPWM_DC_EVENT_2_latchMode");
			}
		},
		devices : [
                "am263x"
			]
		},

	// Validation #31
	{
		type : validation_info,
		name :  "If the EPWMxSYNCI signal is held HIGH, the sync will NOT continously occur. The EPWMxSYNCI is rising edge activated.",
		func : (inst, validation, name) => {
			if (inst["epwmTimebase_phaseEnable"] == 1 && inst["epwmTimebase_phaseShift"] > 0)
			{
				validation.logInfo(name, inst, "epwmTimebase_phaseEnable");
			}
		},
		devices : [
                "am263x"
			]
		},

	// Validation #32
	{
		type : validation_info,
		name :  "Conflicting actions on the TZCTL, TZCTL2, TZCTLDCA or TZCTLDCB registers. Refer to the TRM for the priority scheme.",
		func : (inst, validation, name) => {
			if (inst["epwmTripZone_useAdvancedEPWMTripZoneActions"] == 0 && (inst["epwmTripZone_EPWM_TZ_ACTION_EVENT_TZA"] != inst["epwmTripZone_EPWM_TZ_ACTION_EVENT_DCAEVT1"]
				|| inst["epwmTripZone_EPWM_TZ_ACTION_EVENT_TZA"] != inst["epwmTripZone_EPWM_TZ_ACTION_EVENT_DCAEVT2"] || inst["epwmTripZone_EPWM_TZ_ACTION_EVENT_DCAEVT1"] !=
				inst["epwmTripZone_EPWM_TZ_ACTION_EVENT_DCAEVT2"]))
			{
				validation.logInfo(name, inst, "epwmTripZone_EPWM_TZ_ACTION_EVENT_TZA");
			}
			if (inst["epwmTripZone_useAdvancedEPWMTripZoneActions"] == 1 && (inst["epwmTripZone_EPWM_TZ_ADV_ACTION_EVENT_TZA_U"] != inst["epwmTripZone_EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_U_A"]
				|| inst["epwmTripZone_EPWM_TZ_ADV_ACTION_EVENT_TZA_U"] != inst["epwmTripZone_EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_U_A"] || inst["epwmTripZone_EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_U_A"] !=
				inst["epwmTripZone_EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_U_A"]))
			{
				validation.logInfo(name, inst, "epwmTripZone_EPWM_TZ_ADV_ACTION_EVENT_TZA_U");
			}
			if (inst["epwmTripZone_useAdvancedEPWMTripZoneActions"] == 1 && (inst["epwmTripZone_EPWM_TZ_ADV_ACTION_EVENT_TZA_D"] != inst["epwmTripZone_EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_D_A"]
				|| inst["epwmTripZone_EPWM_TZ_ADV_ACTION_EVENT_TZA_D"] != inst["epwmTripZone_EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_D_A"] || inst["epwmTripZone_EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_D_A"] !=
				inst["epwmTripZone_EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_D_A"]))
			{
				validation.logInfo(name, inst, "epwmTripZone_EPWM_TZ_ADV_ACTION_EVENT_TZA_D");
			}
			if (inst["epwmTripZone_useAdvancedEPWMTripZoneActions"] == 0 && (inst["epwmTripZone_EPWM_TZ_ACTION_EVENT_TZB"] != inst["epwmTripZone_EPWM_TZ_ACTION_EVENT_DCBEVT1"]
				|| inst["epwmTripZone_EPWM_TZ_ACTION_EVENT_TZB"] != inst["epwmTripZone_EPWM_TZ_ACTION_EVENT_DCBEVT2"] || inst["epwmTripZone_EPWM_TZ_ACTION_EVENT_DCBEVT1"] !=
				inst["epwmTripZone_EPWM_TZ_ACTION_EVENT_DCBEVT2"]))
			{
				validation.logInfo(name, inst, "epwmTripZone_EPWM_TZ_ACTION_EVENT_TZB");
			}
			if (inst["epwmTripZone_useAdvancedEPWMTripZoneActions"] == 1 && (inst["epwmTripZone_EPWM_TZ_ADV_ACTION_EVENT_TZB_U"] != inst["epwmTripZone_EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_U_B"]
				|| inst["epwmTripZone_EPWM_TZ_ADV_ACTION_EVENT_TZB_U"] != inst["epwmTripZone_EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_U_B"] || inst["epwmTripZone_EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_U_B"] !=
				inst["epwmTripZone_EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_U_B"]))
			{
				validation.logInfo(name, inst, "epwmTripZone_EPWM_TZ_ADV_ACTION_EVENT_TZB_U");
			}
			if (inst["epwmTripZone_useAdvancedEPWMTripZoneActions"] == 1 && (inst["epwmTripZone_EPWM_TZ_ADV_ACTION_EVENT_TZB_D"] != inst["epwmTripZone_EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_D_B"]
				|| inst["epwmTripZone_EPWM_TZ_ADV_ACTION_EVENT_TZB_D"] != inst["epwmTripZone_EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_D_B"] || inst["epwmTripZone_EPWM_TZ_ADV_ACTION_EVENT_DCxEVT1_D_B"] !=
				inst["epwmTripZone_EPWM_TZ_ADV_ACTION_EVENT_DCxEVT2_D_B"]))
			{
				validation.logInfo(name, inst, "epwmTripZone_EPWM_TZ_ADV_ACTION_EVENT_TZB_D");
			}

		},
		devices : [
                "am263x"
			]
	},
    // Validation #33
	{
		type : validation_warning,
		name :  "XCMP mode works only if CTRMODE = up count mode",
		func : (inst, validation, name) => {
			if (inst["epwmXCMP_enableMode"] == true && inst["epwmTimebase_counterMode"] != "EPWM_COUNTER_MODE_UP")
			{
		         validation.logError(name, inst, "epwmTimebase_counterMode");
			}

		},
		devices : [
                "am263x"
			]
	},
    // Validation #34
	{
		type : validation_error,
		name :  "Input value is out of range (3-bit value)",
		func : (inst, validation, name) => {
			if (inst["epwmXLOADCTL_RepeatBuf2"] < 0 || inst["epwmXLOADCTL_RepeatBuf2"] > 7)
			{
		         validation.logError(name, inst, "epwmXLOADCTL_RepeatBuf2");
			}
            if (inst["epwmXLOADCTL_RepeatBuf3"] < 0 || inst["epwmXLOADCTL_RepeatBuf3"] > 7)
			{
		         validation.logError(name, inst, "epwmXLOADCTL_RepeatBuf3");
			}

		},
		devices : [
                "am263x"
			]
	},

    // Validation #35
	{
		type : validation_error,
		name :  "Input value is out of range (8-bit value)",
		func : (inst, validation, name) => {
			if (inst["epwmDE_reEntryDelay"] > 255)
			{
		         validation.logError(name, inst, "epwmDE_reEntryDelay");
			}
            if (inst["epwmDE_DecrementStep"] > 255)
			{
		         validation.logError(name, inst, "epwmDE_DecrementStep");
			}
            if (inst["epwmDE_IncrementStep"] > 255)
			{
		         validation.logError(name, inst, "epwmDE_IncrementStep");
			}
            if (inst["hrpwm_cmpaHR"] < 0 || inst["hrpwm_cmpaHR"] > 255)
			{
				validation.logError(name, inst, "hrpwm_cmpaHR");
			}
			if (inst["hrpwm_cmpbHR"] < 0 || inst["hrpwm_cmpbHR"] > 255)
			{
				validation.logError(name, inst, "hrpwm_cmpbHR");
			}
			if (inst["hrpwm_tbprdHR"] < 0 || inst["hrpwm_tbprdHR"] > 255)
			{
				validation.logError(name, inst, "hrpwm_tbprdHR");
			}
			if (inst["hrpwm_tbphsHR"] < 0 || inst["hrpwm_tbphsHR"] > 255)
			{
				validation.logError(name, inst, "hrpwm_tbphsHR");
			}
		},
		devices : [
                "am263x"
			]
	},

     // Validation #36
	{
		type : validation_error,
		name :  "Input value is out of range (7-bit value)",
		func : (inst, validation, name) => {
			if (inst["hrpwm_DBredHR"] < 0 || inst["hrpwm_DBredHR"] > 127)
			{
				validation.logError(name, inst, "hrpwm_DBredHR");
			}
			if (inst["hrpwm_DBfedHR"] < 0 || inst["hrpwm_DBfedHR"] > 127)
			{
				validation.logError(name, inst, "hrpwm_DBfedHR");
			}
		},
		devices : [
            "am263x"
			]
	},

     // Validation #37
    {
		type : validation_warning,
		name :  "In HR mode, if CTRMODE= up count mode, the deadband module is not supported",
		func : (inst, validation, name) => {
			if (inst["epwmTimebase_counterMode"] == "EPWM_COUNTER_MODE_UP" && inst["hrpwm_enable"] == true)
			{
				validation.logWarning(name, inst, "hrpwm_edgeModeDB");
			}
		},
		devices : [
            "am263x"
			]
	},

    // Validation #38
    {
		type : validation_error,
		name :  "Auto-conversion must always be enabled for high resolution period mode",
		func : (inst, validation, name) => {
			if (inst["hrpwm_enable"] == true && inst["hrpwm_periodEnable"] == true && inst["hrpwm_autoConv"] == false && (inst["hrpwm_controlModeA"] == "HRPWM_MEP_DUTY_PERIOD_CTRL" || inst["hrpwm_controlModeB"] == "HRPWM_MEP_DUTY_PERIOD_CTRL"))
			{
				validation.logError(name, inst, "hrpwm_periodEnable");
			}
		},
		devices : [
            "am263x"
			]
	},

    // Validation #39
	{
		type : validation_info,
		name :  "The PWM Chopper will not be enabled when high-resolution deadband is enabled",
		func : (inst, validation, name) => {
			if (inst["hrpwm_enable"] == true && inst["epwmChopper_useChopper"] == true && inst["hrpwm_edgeModeDB"] != "HRPWM_DB_MEP_CTRL_DISABLE")
			{
				validation.logInfo(name, inst, "epwmChopper_useChopper");
				validation.logInfo(name, inst, "hrpwm_edgeModeDB");
			}
		},
		devices : [
            "am263x"
			]
	},

    // Validation #40
	{
		type : validation_error,
		name :  "High-resolution dead-band RED and FED requires Half-Cycle clocking mode (DBCTL[HALFCYCLE] = 1)",
		func : (inst, validation, name) => {
			if (inst["hrpwm_enable"] == true && inst["hrpwm_edgeModeDB"] != "HRPWM_DB_MEP_CTRL_DISABLE" && inst["epwmDeadband_deadbandCounterClockRate"]== "EPWM_DB_COUNTER_CLOCK_FULL_CYCLE")
			{
				validation.logError(name, inst, "epwmDeadband_deadbandCounterClockRate");
			}
		},
		devices : [
            "am263x"
			]
	},

    // Validation #41
	{
		type : validation_warning,
		name :  "TBPRDHR should not be used with Global load. If high resolution period must be changed in the application, write to the individual period registers from an ePWM ISR (The ISR must be synchronous with the PWM switching period), where the Global Load One-Shot bit is also written to.",
		func : (inst, validation, name) => {
			if (inst["hrpwm_enable"] == true && inst["hrpwm_periodEnable"] == true && inst["epwmTimebase_periodGld"] == true && inst["epwmGlobalLoad_gld"] == true)
			{
				validation.logWarning(name, inst, "epwmTimebase_periodGld");
			}
		},
		devices : [
            "am263x"
			]
		},

    // Validation #42
	{
		type : validation_warning,
		name :  "When high-resolution period control is enabled, on ePWMxA or ePWMxB output only, the non hi-res output will have +/- 1 TBCLK cycle jitter",
		func : (inst, validation, name) => {
			if (inst["hrpwm_enable"] == true && inst["hrpwm_periodEnable"] == true && inst["epwmTimebase_counterMode"] == "EPWM_COUNTER_MODE_UP" && ((inst["hrpwm_controlModeA"] == "HRPWM_MEP_DUTY_PERIOD_CTRL" && inst["hrpwm_controlModeB"] == "HRPWM_MEP_PHASE_CTRL") || (inst["hrpwm_controlModeA"] == "HRPWM_MEP_PHASE_CTRL" && inst["hrpwm_controlModeB"] == "HRPWM_MEP_DUTY_PERIOD_CTRL")))
			{
				validation.logWarning(name, inst, "hrpwm_periodEnable");
			}
		},
		devices : [
            "am263x"
			]
	},

    // Validation #43
	{
		type : validation_warning,
		name :  "When high-resolution period control is enabled, on ePWMxA or ePWMxB output only, the non hi-res output will have +/- 2 TBCLK cycle jitter",
		func : (inst, validation, name) => {
			if (inst["hrpwm_enable"] == true && inst["hrpwm_periodEnable"] == true && inst["epwmTimebase_counterMode"] == "EPWM_COUNTER_MODE_UP_DOWN" && ((inst["hrpwm_controlModeA"] == "HRPWM_MEP_DUTY_PERIOD_CTRL" && inst["hrpwm_controlModeB"] == "HRPWM_MEP_PHASE_CTRL") || (inst["hrpwm_controlModeA"] == "HRPWM_MEP_PHASE_CTRL" && inst["hrpwm_controlModeB"] == "HRPWM_MEP_DUTY_PERIOD_CTRL")))
			{
				validation.logWarning(name, inst, "hrpwm_periodEnable");
			}
		},
		devices : [
            "am263x"
			]
	},

    // Validation #44
	{
		type : validation_error,
		name :  "High-resolution period is not compatible with down-count mode.",
		func : (inst, validation, name) => {
			if (inst["hrpwm_enable"] == true && inst["epwmTimebase_counterMode"] == "EPWM_COUNTER_MODE_DOWN" && inst["hrpwm_periodEnable"] == true && (inst["hrpwm_controlModeA"] == "HRPWM_MEP_DUTY_PERIOD_CTRL" || inst["hrpwm_controlModeB"] == "HRPWM_MEP_DUTY_PERIOD_CTRL"))
			{
				validation.logError(name, inst, "hrpwm_periodEnable");
			}
		},
		devices : [
            "am263x"
			]
		},

    // Validation #45
	{
		type : validation_error,
		name :  "For High Ressolution Period TBPRD must have shadow loading enabled if HRPWM is used",
		func : (inst, validation, name) => {
			if (inst["hrpwm_enable"] == true && inst["hrpwm_periodEnable"] == true && (inst["hrpwm_controlModeA"] == "HRPWM_MEP_DUTY_PERIOD_CTRL" || inst["hrpwm_controlModeB"] == "HRPWM_MEP_DUTY_PERIOD_CTRL") && inst["epwmTimebase_periodLoadMode"] != "EPWM_PERIOD_SHADOW_LOAD")
			{
				validation.logError(name, inst, "hrpwm_periodEnable");
			}
		},
		devices : [
            "am263x"
			]
		},

        // Validation #46
	{
		type : validation_error,
		name :  "For High Resolution Period: CMPA must have shadow loading enabled for when TBCTR=PRD",
		func : (inst, validation, name) => {
			if (inst["hrpwm_enable"] == true && inst["hrpwm_periodEnable"] == true && inst["epwmTimebase_counterMode"] == "EPWM_COUNTER_MODE_UP" && inst["hrpwm_edgeModeA"] != "HRPWM_MEP_CTRL_DISABLE" && inst["hrpwm_controlModeA"] == "HRPWM_MEP_DUTY_PERIOD_CTRL" && (inst["epwmCounterCompare_enableShadowLoadModeCMPA"] == false || inst["epwmCounterCompare_shadowLoadModeCMPA"] != "EPWM_COMP_LOAD_ON_CNTR_PERIOD"))
			{
				validation.logError(name, inst, "hrpwm_edgeModeA");
			}
		},
		devices : [
				"am263x"
			]
		},
	// Validation #47
	{
		type : validation_error,
		name :  "For High Resolution Period: CMPB must have shadow loading enabled for when TBCTR=PRD",
		func : (inst, validation, name) => {
			if (inst["hrpwm_enable"] == true && inst["hrpwm_periodEnable"] == true && inst["epwmTimebase_counterMode"] == "EPWM_COUNTER_MODE_UP" && inst["hrpwm_edgeModeB"] != "HRPWM_MEP_CTRL_DISABLE" && inst["hrpwm_controlModeB"] == "HRPWM_MEP_DUTY_PERIOD_CTRL" && (inst["epwmCounterCompare_enableShadowLoadModeCMPB"] == false || inst["epwmCounterCompare_shadowLoadModeCMPB"] != "EPWM_COMP_LOAD_ON_CNTR_PERIOD"))
			{
				validation.logError(name, inst, "hrpwm_edgeModeB");
			}
		},
		devices : [
				"am263x"
			]
		},
	// Validation #48
	{
		type : validation_error,
		name :  "For High Resolution Period: CMPA must have shadow loading enabled for when TBCTR=0 or PRD",
		func : (inst, validation, name) => {
			if (inst["hrpwm_enable"] == true && inst["hrpwm_periodEnable"] == true && inst["epwmTimebase_counterMode"] == "EPWM_COUNTER_MODE_UP_DOWN" && inst["hrpwm_edgeModeA"] != "HRPWM_MEP_CTRL_DISABLE" && inst["hrpwm_controlModeA"] == "HRPWM_MEP_DUTY_PERIOD_CTRL" && (inst["epwmCounterCompare_enableShadowLoadModeCMPA"] == false || inst["epwmCounterCompare_shadowLoadModeCMPA"] != "EPWM_COMP_LOAD_ON_CNTR_ZERO_PERIOD"))
			{
				validation.logError(name, inst, "hrpwm_edgeModeA");
			}
		},
		devices : [
				"am263x"
			]
		},
	// Validation #49
	{
		type : validation_error,
		name :  "For High Resolution Period: CMPB must have shadow loading enabled for when TBCTR=0 or PRD",
		func : (inst, validation, name) => {
			if (inst["hrpwm_enable"] == true && inst["hrpwm_periodEnable"] == true && inst["epwmTimebase_counterMode"] == "EPWM_COUNTER_MODE_UP_DOWN" && inst["hrpwm_edgeModeB"] != "HRPWM_MEP_CTRL_DISABLE" && inst["hrpwm_controlModeB"] == "HRPWM_MEP_DUTY_PERIOD_CTRL" && (inst["epwmCounterCompare_enableShadowLoadModeCMPB"] == false || inst["epwmCounterCompare_shadowLoadModeCMPB"] != "EPWM_COMP_LOAD_ON_CNTR_ZERO_PERIOD"))
			{
				validation.logError(name, inst, "hrpwm_edgeModeB");
			}
		},
		devices : [
				"am263x"
			]
		},
	// Validation #50
	{
		type : validation_warning,
		name :  "For High Resolution Period: Shadow loading of CMPBHR should occur when TBCTR=0 or PRD and MEP should control both edges",
		func : (inst, validation, name) => {
			if (inst["hrpwm_enable"] == true && inst["hrpwm_periodEnable"] == true && inst["hrpwm_controlModeB"] == "HRPWM_MEP_DUTY_PERIOD_CTRL" && (inst["hrpwm_HRLoadB"] != "HRPWM_LOAD_ON_CNTR_ZERO_PERIOD" || inst["hrpwm_edgeModeB"] != "HRPWM_MEP_CTRL_RISING_AND_FALLING_EDGE"))
			{
				validation.logWarning(name, inst, "hrpwm_edgeModeB");
			}
		},
		devices : [
				"am263x"
			]
		},
	// Validation #51
	{
		type : validation_warning,
		name :  "For High Resolution Period: Shadow loading of CMPAHR should occur when TBCTR=0 or PRD and MEP should control both edges",
		func : (inst, validation, name) => {
			if (inst["hrpwm_enable"] == true && inst["hrpwm_periodEnable"] == true && inst["hrpwm_controlModeA"] == "HRPWM_MEP_DUTY_PERIOD_CTRL" && (inst["hrpwm_HRLoadA"] != "HRPWM_LOAD_ON_CNTR_ZERO_PERIOD" || inst["hrpwm_edgeModeA"] != "HRPWM_MEP_CTRL_RISING_AND_FALLING_EDGE"))
			{
				validation.logWarning(name, inst, "hrpwm_edgeModeA");
			}
		},
		devices : [
				"am263x"
			]
		},
	// Validation #52
	{
		type : validation_error,
		name :  "When using HRPWM, the DBRED and DBFED values must be greater than 3 to use hi-res deadband",
		func : (inst, validation, name) => {
			if (inst["hrpwm_enable"] == true && inst["hrpwm_edgeModeDB"] == "HRPWM_DB_MEP_CTRL_DISABLE" && ((inst["epwmDeadband_delayRED"] < 3 && inst["epwmDeadband_enableRED"] == true )|| (inst["epwmDeadband_delayFED"] < 3 && inst["epwmDeadband_enableFED"] == true)))
			{
				validation.logError(name, inst, "epwmDeadband_delayRED");
				validation.logError(name, inst, "epwmDeadband_delayFED");
				validation.logError(name, inst, "hrpwm_DBredHR");
				validation.logError(name, inst, "hrpwm_DBfedHR");
			}
		},
		devices : [
				"am263x"
			]
		},
	// Validation #53
	{
		type : validation_error,
		name :  "When using DBREDHR, DBRED must be greater than or equal to 7",
		func : (inst, validation, name) => {
			if (inst["hrpwm_enable"] == true && inst["hrpwm_edgeModeDB"] == "HRPWM_DB_MEP_CTRL_RED" && inst["hrpwm_DBredHR"] != 0 && inst["epwmDeadband_delayRED"] < 7 && inst["epwmDeadband_enableRED"] == true)
			{
				validation.logError(name, inst, "hrpwm_DBredHR");
			}
		},
		devices : [
				"am263x"
			]
		},
	// Validation #54
	{
		type : validation_error,
		name :  "When using DBFEDHR, DBFED must be greater than or equal to 7",
		func : (inst, validation, name) => {
			if (inst["hrpwm_enable"] == true && inst["hrpwm_edgeModeDB"] == "HRPWM_DB_MEP_CTRL_FED" && inst["hrpwm_DBfedHR"] != 0 && inst["epwmDeadband_delayFED"] < 7 && inst["epwmDeadband_enableFED"] == true)
			{
				validation.logError(name, inst, "hrpwm_DBfedHR");
			}
		},
		devices : [
				"am263x"
			]
		},
	// Validation #55
	{
		type : validation_error,
		name :  "When using DBREDHR and DBFEDHR, DBRED and DBFED must be greater than or equal to 7",
		func : (inst, validation, name) => {
			if (inst["hrpwm_enable"] == true && inst["hrpwm_edgeModeDB"] == "HRPWM_DB_MEP_CTRL_RED_FED" && (inst["hrpwm_DBfedHR"] != 0 || inst["hrpwm_DBredHR"] != 0) && ((inst["epwmDeadband_delayFED"] < 7 && inst["epwmDeadband_enableRED"] == true) || (inst["epwmDeadband_delayRED"] < 7 && inst["epwmDeadband_enableFED"] == true)))
			{
				validation.logError(name, inst, "hrpwm_DBredHR");
				validation.logError(name, inst, "hrpwm_DBfedHR");
			}
		},
		devices : [
				"am263x"
			]
		},
	// Validation #56
	{
		type : validation_error,
		name :  "A CMPAHR value of 0 is not supported",
		func : (inst, validation, name) => {
			if (inst["hrpwm_cmpaHR"] == 0 && inst["hrpwm_controlModeA"] == "HRPWM_MEP_DUTY_PERIOD_CTRL")
			{
				validation.logError(name, inst, "hrpwm_cmpaHR");
			}
		},
		devices : [
				"am263x"
			]
		},
	// Validation #57
	{
		type : validation_error,
		name :  "A CMPBHR value of 0 is not supported",
		func : (inst, validation, name) => {
			if (inst["hrpwm_cmpbHR"] == 0 && inst["hrpwm_controlModeB"] == "HRPWM_MEP_DUTY_PERIOD_CTRL")
			{
				validation.logError(name, inst, "hrpwm_cmpbHR");
			}
		},
		devices : [
				"am263x"
			]
		},
	// Validation #58
	{
		type : validation_warning,
		name :  "If doing complementary signals through deadband, a CMPBHR value is still needed.",
		func : (inst, validation, name) => {
			if (inst["hrpwm_enabled"] == true && (inst["hrpwm_controlModeA"] == "HRPWM_MEP_DUTY_PERIOD_CTRL" || inst["hrpwm_controlModeB"] == "HRPWM_MEP_DUTY_PERIOD_CTRL") && inst["epwmDeadband_inputRED"]== "EPWM_DB_INPUT_EPWMA" && inst["epwmDeadband_inputFED"]== "EPWM_DB_INPUT_EPWMA" && inst["epwmDeadband_polarityRED"]== "EPWM_DB_POLARITY_ACTIVE_HIGH" && inst["epwmDeadband_polarityFED"]== "EPWM_DB_POLARITY_ACTIVE_LOW" && inst["epwmDeadband_enableRED"] == true && inst["epwmDeadband_enableFED"] == true && inst["epwmDeadband_outputSwapOutA"] == false && inst["epwmDeadband_outputSwapOutB"] == false)
			{
				validation.logWarning(name, inst, "hrpwm_cmpbHR");
			}
		},
		devices : [
				"am263x"
			]
		},

	// Validation #59
	{
		type : validation_info,
		name :  "MEP becomes operational: •Three EPWMCLK cycles after the period starts when high-resolution period (TBPRDHR) control is not enabled. • When high resolution period (TBPRDHR) control is enabled via the HRPCTL register – In up-count mode: three EPWMCLK cycles after the period starts until three EPWMCLK cycles before the period ends. – In up-down count mode: when counting up, three cycles after CTR = 0 until three cycles before CTR = PRD, and when counting down, three cycles after CTR = PRD until three cycles before CTR= 0.",
		func : (inst, validation, name) => {
			if (inst["hrpwm_enable"] == true)
			{
				validation.logInfo(name, inst, "hrpwm_enable");
			}
		},
		devices : [
				"am263x"
			]
		},
	// Validation #60
	{
		type : validation_warning,
		name :  "For TBPHS:TBPHSHR synchronization with high-resolution period, set both high resolution phase shift load and phase shift load. In up-down count mode these bits must be set to 1 regardless of the contents of TBPHSHR.",
		func : (inst, validation, name) => {
			if (inst["hrpwm_enable"] == true && inst["hrpwm_periodEnable"] == true && (inst["hrpwm_controlModeA"] == "HRPWM_MEP_PHASE_CTRL" || inst["hrpwm_controlModeB"] == "HRPWM_MEP_PHASE_CTRL") && (inst["hrpwm_phaseLoadEnable"] != true || inst["epwmTimebase_phaseEnable"] != true))
			{
				validation.logWarning(name, inst, "hrpwm_phaseLoadEnable");
				validation.logWarning(name, inst, "epwmTimebase_phaseEnable");
			}
		},
		devices : [
				"am263x"
			]
		},
	// Validation #61
	{
		type : validation_info,
		name :  "When high-resolution period mode is enabled, an EPWMxSYNC pulse will introduce +/- 1 - 2 cycle jitter to the PWM (+/- 1 cycle in up-count mode and +/- 2 cycle in up-down count mode).When the 'Sync Out Pulse' is configured to software, a software synchronization pulse should be issued only once during high-resolution period initialization. If a software sync pulse is applied while the PWM is running, the jitter will appear on the PWM output at the time of the sync pulse.",
		func : (inst, validation, name) => {
			if (inst["hrpwm_enable"] == true && inst["hrpwm_periodEnable"] == true && (inst["hrpwm_controlModeA"] == "HRPWM_MEP_PHASE_CTRL" || inst["hrpwm_controlModeB"] == "HRPWM_MEP_PHASE_CTRL"))
			{
				validation.logInfo(name, inst, "hrpwm_periodEnable");
			}
		},
		devices : [
				"am263x"
			]
		},
	// Validation #62
	{
		type : validation_warning,
		name :  "If counter mode is up count mode, edge control of both edges is not supported",
		func : (inst, validation, name) => {
			if (inst["hrpwm_enable"] == true && inst["epwmTimebase_counterMode"] == "EPWM_COUNTER_MODE_UP" && inst["hrpwm_controlModeA"] == "HRPWM_MEP_DUTY_PERIOD_CTRL" && inst["hrpwm_edgeModeA"] == "HRPWM_MEP_CTRL_RISING_AND_FALLING_EDGE")
			{
				validation.logWarning(name, inst, "epwmTimebase_counterMode");
				validation.logWarning(name, inst, "hrpwm_controlModeA");
			}
		},
		devices : [
				"am263x"
			]
		},
	// Validation #63
	{
		type : validation_warning,
		name :  "If counter mode is up count mode, edge control of both edges is not supported",
		func : (inst, validation, name) => {
			if (inst["hrpwm_enable"] == true && inst["epwmTimebase_counterMode"] == "EPWM_COUNTER_MODE_UP" && inst["hrpwm_controlModeB"] == "HRPWM_MEP_DUTY_PERIOD_CTRL" && inst["hrpwm_edgeModeB"] == "HRPWM_MEP_CTRL_RISING_AND_FALLING_EDGE")
			{
				validation.logWarning(name, inst, "epwmTimebase_counterMode");
				validation.logWarning(name, inst, "hrpwm_controlModeB");
			}
		},
		devices : [
				"am263x"
			]
		},
    //Validation #64
    {
        type: validation_error,
        name: "On enabling XCMP mode, the configurations for normal Counter Compare and Action Qualifier submodule will get suppressed",
        func : (inst, validation, name) => {
            if(inst["epwmXCMP_enableMode"])
            {
                validation.logInfo(name, inst, "epwmXCMP_enableMode");
            }
        },
        devices : [
            "am263x"
        ]
    }
]

exports = {
	validation_error : validation_error,
	validation_warning : validation_warning,
	validation_info : validation_info,
	epwm_validation : epwm_validation,
}