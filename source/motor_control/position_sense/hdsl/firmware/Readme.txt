Steps for generating HDSL binary header files is given below.
1. In the make file, porovide correct paths for CCS(CCS_PATH) and code generation tool(CG_TOOL_ROOT).
2. Provide the required mode as shown below. If user does not provide input, default mode taken is FREE_RUN_MODE.
   $(MCU_PLUS_SDK_PATH)\source\motor_control\position_sense\hdsl\firmware>gmake all HDSL_MODE=FREE_RUN_MODE
   $(MCU_PLUS_SDK_PATH)\source\motor_control\position_sense\hdsl\firmware>gmake all HDSL_MODE=SYNC_MODE
3. Above step will generate either hdsl_master_icssg_bin.h or hdsl_master_icssg_sync_bin.h.