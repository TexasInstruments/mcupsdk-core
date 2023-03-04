let common   = system.getScript("/common");

function getBaseLog(x, y) {
  return Math.log(y) / Math.log(x);
}

function getShiftvalue(FilterType, OSR, rshift, datarep)
{
  let SincFast = [];
  let Sinc1 = [];
  let Sinc2 = [];
  let Sinc3 = [];

  let shiftValue;

  //let FilterType = "SINC3";
  //let OSR = 256;

  let shift_Sinc1 = [];
  let shift_Sinc2 = [];
  let shift_SincFast = [];
  let shift_Sinc3 = [];

  Sinc1[OSR] 	= OSR;
  Sinc2[OSR] 	= OSR*OSR;
  SincFast[OSR] = 2 * OSR*OSR;
  Sinc3[OSR]    = OSR*OSR*OSR;

  shift_SincFast[OSR] = ((getBaseLog(2, SincFast[OSR]) - 15) < 0) ? 0 : Math.ceil((getBaseLog(2, SincFast[OSR]) - 15));
  shift_Sinc1[OSR] 	  = ((getBaseLog(2, Sinc1[OSR]) - 15) < 0) ? 0 : Math.ceil((getBaseLog(2, Sinc1[OSR]) - 15));
  shift_Sinc2[OSR] = ((getBaseLog(2, Sinc2[OSR]) - 15) < 0) ? 0 : Math.ceil((getBaseLog(2, Sinc2[OSR]) - 15));
  shift_Sinc3[OSR] = ((getBaseLog(2, Sinc3[OSR]) - 15) < 0) ? 0 : Math.ceil((getBaseLog(2, Sinc3[OSR]) - 15));


  //let FilterType = 'SINC3';
  let filter_32bit_Output = 0;
  let filter_16bit_Output = 0;

  switch(FilterType)
  {
  case 'SDFM_FILTER_SINC_1':
    shiftValue = shift_Sinc1[OSR];
	filter_32bit_Output = Sinc1[OSR];

	filter_16bit_Output = Sinc1[OSR] >> shift_Sinc1[OSR];
    break;

  case 'SDFM_FILTER_SINC_2':
    shiftValue = shift_Sinc2[OSR];
	filter_32bit_Output = Sinc2[OSR];

    filter_16bit_Output = Sinc2[OSR] >> shift_Sinc2[OSR];
	if(filter_16bit_Output >= 32768)
	{
	   filter_16bit_Output = filter_16bit_Output >> 1; shiftValue++;
	}
    break;

  case 'SDFM_FILTER_SINC_FAST':
    shiftValue = shift_SincFast[OSR];
	filter_32bit_Output = SincFast[OSR];

    filter_16bit_Output = SincFast[OSR] >> shift_SincFast[OSR];
	if(filter_16bit_Output >= 32768)
	{
	   filter_16bit_Output = filter_16bit_Output >> 1; shiftValue++;
	}
    break;

  case 'SDFM_FILTER_SINC_3':
    shiftValue = shift_Sinc3[OSR];
	filter_32bit_Output = Sinc3[OSR];

    filter_16bit_Output = Sinc3[OSR] >> shift_Sinc3[OSR];
    if(filter_16bit_Output >= 32768)
	{
	   filter_16bit_Output = filter_16bit_Output >> 1; shiftValue++;
	}
    break;

  }

  if(rshift)
  {
	return shiftValue;
  }
  else
  {
   if(datarep == "SDFM_DATA_FORMAT_16_BIT")
   {
	 return Math.floor(filter_16bit_Output);
   }
   if(datarep == "SDFM_DATA_FORMAT_32_BIT")
   {
	 return filter_32bit_Output;
   }
  }
}

exports =
{
	getShiftvalue : getShiftvalue,
	getBaseLog: getBaseLog
}
