Steps for generating ENDAT binary header files is given below.
1. In the make file, porovide correct paths for CCS(CCS_PATH) and code generation tool(CG_TOOL_ROOT).
2. Provide the required mode as shown below. If user does not provide input, default mode taken is SINGLE.
   $(MCU_PLUS_SDK_PATH)\source\motor_control\position_sense\endat\firmware>gmake all CHANNELS=SINGLE
   $(MCU_PLUS_SDK_PATH)\source\motor_control\position_sense\endat\firmware>gmake all CHANNELS=MULTI
3. Above step will generate either endat_master_bin.h or endat_master_multi_bin.h.