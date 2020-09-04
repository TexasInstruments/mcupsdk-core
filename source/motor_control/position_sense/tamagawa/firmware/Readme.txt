Steps for generating TAMAGAWA binary header files is given below.
1. In the make file, porovide correct paths for CCS(CCS_PATH) and code generation tool(CG_TOOL_ROOT).
2. Provide the required input as shown below.
   $(MCU_PLUS_SDK_PATH)\source\motor_control\position_sense\tamagawa\firmware>gmake all   
3. Above step will generate tamagawa_receiver_bin.h.