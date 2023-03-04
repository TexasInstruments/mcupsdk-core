let inst_name;

function onValidate(inst, validation)
{
  for (let channel = 1; channel <= 4; channel++)
  {
    inst_name= "Ch" + channel.toString() + "_COSR";
	let cosr = inst[inst_name];

	if(cosr < 1 || cosr > 32)
	{
		validation.logError(
            "COSR should be between 1 to 32! - Check Filter_" + channel + " COSR = " + cosr,
            inst, inst_name);
	}
    inst_name= "Ch" + channel.toString() + "_DOSR";
	let dosr = inst[inst_name];

	if(dosr < 1 || dosr > 256)
	{
		validation.logError(
            "DOSR should be between 1 to 256! - Check Filter_" + channel + " DOSR = " + dosr,
            inst, inst_name);
	}
    inst_name= "Ch" + channel.toString() + "_HLT1";
	let HLT1 = inst[inst_name];

	if(HLT1 < 0 || HLT1 > 32767)
	{
		validation.logError(
            "HLT1 should be between 0 to 32767! - Check Filter_" + channel + " HLT1 = " + HLT1,
            inst, inst_name);
	}
    inst_name= "Ch" + channel.toString() + "_HLT2";
	let HLT2 = inst[inst_name];

	if(HLT2 < 0 || HLT2 > 32767)
	{
		validation.logError(
            "HLT2 should be between 0 to 32767! - Check Filter_" + channel + " HLT2 = " + HLT2,
            inst, inst_name);
	}
    inst_name= "Ch" + channel.toString() + "_HLT";
	let HLT = inst[inst_name];

	if(HLT < 0 || HLT > 32767)
	{
		validation.logError(
            "HLT should be between 0 to 32767! - Check Filter_" + channel + " HLT = " + HLT,
            inst, inst_name);
	}
    inst_name= "Ch" + channel.toString() + "_HLTZ";
	let HLTZ = inst[inst_name];

	if(HLTZ < 0 || HLTZ > 32767)
	{
		validation.logError(
            "HLTZ should be between 0 to 32767! - Check Filter_" + channel + " HLTZ = " + HLTZ,
            inst, inst_name);
	}
    inst_name= "Ch" + channel.toString() + "_LLT1";
	let LLT1 = inst[inst_name];

	if(LLT1 < 0 || LLT1 > 32767)
	{
		validation.logError(
            "LLT1 should be between 0 to 32767! - Check Filter_" + channel + " LLT1 = " + LLT1,
            inst, inst_name);
	}
    inst_name= "Ch" + channel.toString() + "_LLT2";
	let LLT2 = inst[inst_name];

	if(LLT2 < 0 || LLT2 > 32767)
	{
		validation.logError(
            "LLT2 should be between 0 to 32767! - Check Filter_" + channel + " LLT2 = " + LLT2,
            inst, inst_name);
	}
    inst_name= "Ch" + channel.toString() + "_LLT";
	let LLT = inst[inst_name];

	if(LLT < 0 || LLT > 32767)
	{
		validation.logError(
            "LLT should be between 0 to 32767! - Check Filter_" + channel + " LLT = " + LLT,
            inst, inst_name);
	}

	for(let event = 1; event <= 2; event++)
	{
        inst_name= "Ch" + channel.toString() + "_samplePrescale_CEVT" + event.toString();
		let Prescale_CEVT = inst[inst_name];

		if(Prescale_CEVT < 0 || Prescale_CEVT > 1023)
		{
		  validation.logError(
            "Comparator event Prescale should be between 0 to 1023! - Check Filter_" + channel + " CEVT" + event.toString() + "Prescale = " + Prescale_CEVT,
            inst, inst_name);
		}
        inst_name= "Ch" + channel.toString() + "_Threshold_CEVT" + event.toString();
		let threshold_CEVT = inst[inst_name];

		if(threshold_CEVT < 0 || threshold_CEVT > 31)
		{
		  validation.logError(
            "Comparator event Threshold should be between 0 to 31! - Check Filter_" + channel + " CEVT" + event.toString() + "Threshold = " + threshold_CEVT,
            inst, inst_name);
		}
        inst_name= "Ch" + channel.toString() + "_sampleWindow_CEVT" + event.toString();
		let sampleWindow_CEVT = inst[inst_name];

		if(sampleWindow_CEVT < 0 || sampleWindow_CEVT > 31)
		{
		  validation.logError(
            "Comparator event samplewindow should be between 0 to 31! - Check Filter_" + channel + " CEVT" + event.toString() + "sample window = " + sampleWindow_CEVT,
            inst, inst_name);
		}
	}
  }
}

exports =
{
    onValidate : onValidate,
}
