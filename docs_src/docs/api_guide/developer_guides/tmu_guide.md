# Trigonometric Math Unit Guide {#TMU_GUIDE}

[TOC]

## Introduction
MCU PLUS SDK integrates a software library to use the TMU (Trigonometric Math Unit) hardware in AM263Px. This TMU library provides efficient (assembly level functions) and MATHLIB compatible API interface for performing trigonometric calculations, a critical component in many real-time applications such as Motor Control and Digital Power. TMU is used to efficiently execute commonly used Trigonometric and Arithmetic Operations by accelerating these math functions, and providing a performance boost to the user application.

TMU library supported by SDK has the following features:

- TMU supports 6 critical trigonometric operations that are widely used in control applications
    - SIN
    - COS
    - ATAN
    - LOG
    - IEXP
    - ATAN2 using QUADF

\imageStyle{tmu_operations.png,width:60%}
    \image html tmu_operations.png "TMU supported operations"
- TMU has it's own memory-mapped set of registers. It supports 8 result registers which enables pipelining of 8 back-to-back operations, with each operation making use of a different result register
- Single cycle context save and restore operations. This feature is essential in scenarios where if a TMU operation running in the main program gets interrupted, and the TMU operation that uses the same result register as the operation present in the main program is being used within the ISR. This case will lead to losing the value that gets stored in the result register after the main program's TMU operation gets completed, as now  the ISR is also using the same result register and overwriting will take place
- Generation of Underflow and Overflow interrupts to show errors in mathematical operations


## Support for Trigonometric Operations

TMU supports a mapping between the Operand registers and the result registers. There are different 8 operand1 registers(R0, R1, R2, R3, R4, R5, R6, R7) for each operation and these registers are sitting at different memory locations. So in total there are 48 operand1 registers available. Users need to perform a write to these registers with the input value, in order to trigger a TMU operation. Based on which address the write happens, TMU gets three key details for performing TMU operation:
- The operation that the user wants to perform
- The result register into which the result will get stored
- The input value that needs to be taken for the operations

### SIN Operation
The eight operand1 registers for SIN are as folows:

- SIN_R0 - SIN operation's first operand1 register present at memory location 0x60040. Write to this register with the input value will trigger a SIN operation and the result will be present in R0 result register
- SIN_R1 - SIN operation's second operand1 register present at memory location 0x60048. Write to this register with the input value will trigger a SIN operation and the result will be present in R1 result register
- SIN_R2 - SIN operation's third operand1 register present at memory location 0x60050. Write to this register with the input value will trigger a SIN operation and the result will be present in R2 result register
- SIN_R3 - SIN operation's fourth operand1 register present at memory location 0x60058. Write to this register with the input value will trigger a SIN operation and the result will be present in R3 result register
- SIN_R4 - SIN operation's fifth operand1 register present at memory location 0x60060. Write to this register with the input value will trigger a SIN operation and the result will be present in R4 result register
- SIN_R5 - SIN operation's sixth operand1 register present at memory location 0x60068. Write to this register with the input value will trigger a SIN operation and the result will be present in R5 result register
- SIN_R6 - SIN operation's seventh operand1 register present at memory location 0x60070. Write to this register with the input value will trigger a SIN operation and the result will be present in R6 result register
- SIN_R7 - SIN operation's eigth operand1 register present at memory location 0x60078. Write to this register with the input value will trigger a SIN operation and the result will be present in R7 result register

### COS Operation
The eight operand1 registers for COS are as folows:

- COS_R0 - COS operation's first operand1 register present at memory location 0x60080. Write to this register with the input value will trigger a COS operation and the result will be present in R0 result register
- COS_R1 - COS operation's second operand1 register present at memory location 0x60088. Write to this register with the input value will trigger a COS operation and the result will be present in R1 result register
- COS_R2 - COS operation's third operand1 register present at memory location 0x60090. Write to this register with the input value will trigger a COS operation and the result will be present in R2 result register
- COS_R3 - COS operation's fourth operand1 register present at memory location 0x60098. Write to this register with the input value will trigger a COS operation and the result will be present in R3 result register
- COS_R4 - COS operation's fifth operand1 register present at memory location 0x600A0. Write to this register with the input value will trigger a COS operation and the result will be present in R4 result register
- COS_R5 - COS operation's sixth operand1 register present at memory location 0x600A8. Write to this register with the input value will trigger a COS operation and the result will be present in R5 result register
- COS_R6 - COS operation's seventh operand1 register present at memory location 0x600B0. Write to this register with the input value will trigger a COS operation and the result will be present in R6 result register
- COS_R7 - COS operation's eigth operand1 register present at memory location 0x600B8. Write to this register with the input value will trigger a COS operation and the result will be present in R7 result register

### ATAN Operation
The eight operand1 registers for ATAN are as folows:

- ATAN_R0 - ATAN operation's first operand1 register present at memory location 0x600C0. Write to this register with the input value will trigger a ATAN operation and the result will be present in R0 result register
- ATAN_R1 - ATAN operation's second operand1 register present at memory location 0x600C8. Write to this register with the input value will trigger a ATAN operation and the result will be present in R1 result register
- ATAN_R2 - ATAN operation's third operand1 register present at memory location 0x600D0. Write to this register with the input value will trigger a ATAN operation and the result will be present in R2 result register
- ATAN_R3 - ATAN operation's fourth operand1 register present at memory location 0x600D8. Write to this register with the input value will trigger a ATAN operation and the result will be present in R3 result register
- ATAN_R4 - ATAN operation's fifth operand1 register present at memory location 0x600E0. Write to this register with the input value will trigger a ATAN operation and the result will be present in R4 result register
- ATAN_R5 - ATAN operation's sixth operand1 register present at memory location 0x600E8. Write to this register with the input value will trigger a ATAN operation and the result will be present in R5 result register
- ATAN_R6 - ATAN operation's seventh operand1 register present at memory location 0x600F0. Write to this register with the input value will trigger a ATAN operation and the result will be present in R6 result register
- ATAN_R7 - ATAN operation's eigth operand1 register present at memory location 0x600F8. Write to this register with the input value will trigger a ATAN operation and the result will be present in R7 result register

### IEXP Operation
The eight operand1 registers for IEXP are as folows:

- IEXP_R0 - IEXP operation's first operand1 register present at memory location 0x60140. Write to this register with the input value will trigger a IEXP operation and the result will be present in R0 result register
- IEXP_R1 - IEXP operation's second operand1 register present at memory location 0x60148. Write to this register with the input value will trigger a IEXP operation and the result will be present in R1 result register
- IEXP_R2 - IEXP operation's third operand1 register present at memory location 0x60150. Write to this register with the input value will trigger a IEXP operation and the result will be present in R2 result register
- IEXP_R3 - IEXP operation's fourth operand1 register present at memory location 0x60158. Write to this register with the input value will trigger a IEXP operation and the result will be present in R3 result register
- IEXP_R4 - IEXP operation's fifth operand1 register present at memory location 0x60160. Write to this register with the input value will trigger a IEXP operation and the result will be present in R4 result register
- IEXP_R5 - IEXP operation's sixth operand1 register present at memory location 0x60168. Write to this register with the input value will trigger a IEXP operation and the result will be present in R5 result register
- IEXP_R6 - IEXP operation's seventh operand1 register present at memory location 0x60170. Write to this register with the input value will trigger a IEXP operation and the result will be present in R6 result register
- IEXP_R7 - IEXP operation's eigth operand1 register present at memory location 0x60178. Write to this register with the input value will trigger a IEXP operation and the result will be present in R7 result register

### LOG Operation
The eight operand1 registers for LOG are as folows:

- LOG_R0 - LOG operation's first operand1 register present at memory location 0x60180. Write to this register with the input value will trigger a LOG operation and the result will be present in R0 result register
- LOG_R1 - LOG operation's second operand1 register present at memory location 0x60188. Write to this register with the input value will trigger a LOG operation and the result will be present in R1 result register
- LOG_R2 - LOG operation's third operand1 register present at memory location 0x60190. Write to this register with the input value will trigger a LOG operation and the result will be present in R2 result register
- LOG_R3 - LOG operation's fourth operand1 register present at memory location 0x60198. Write to this register with the input value will trigger a LOG operation and the result will be present in R3 result register
- LOG_R4 - LOG operation's fifth operand1 register present at memory location 0x601A0. Write to this register with the input value will trigger a LOG operation and the result will be present in R4 result register
- LOG_R5 - LOG operation's sixth operand1 register present at memory location 0x601A8. Write to this register with the input value will trigger a LOG operation and the result will be present in R5 result register
- LOG_R6 - LOG operation's seventh operand1 register present at memory location 0x601B0. Write to this register with the input value will trigger a LOG operation and the result will be present in R6 result register
- LOG_R7 - LOG operation's eigth operand1 register present at memory location 0x601B8. Write to this register with the input value will trigger a LOG operation and the result will be present in R7 result register

### QUADF Operation
The eight operand1 registers for QUADF are as folows:

- QUADF_R0_R1 - QUADF operation's first operand1 register present at memory location 0x601C0. Write to this register with the input value will trigger a QUADF operation and the result will be present in R0 and R1 result register
- QUADF_R1_R2 - QUADF operation's second operand1 register present at memory location 0x601C8. Write to this register with the input value will trigger a QUADF operation and the result will be present in R1 and R2 result register
- QUADF_R2_R3 - QUADF operation's third operand1 register present at memory location 0x601D0. Write to this register with the input value will trigger a QUADF operation and the result will be present in R2 and R3 result register
- QUADF_R3_R4 - QUADF operation's fourth operand1 register present at memory location 0x601D8. Write to this register with the input value will trigger a QUADF operation and the result will be present in R3 and R4 result register
- QUADF_R4_R5 - QUADF operation's fifth operand1 register present at memory location 0x601E0. Write to this register with the input value will trigger a QUADF operation and the result will be present in R4 and R5 result register
- QUADF_R5_R6 - QUADF operation's sixth operand1 register present at memory location 0x601E8. Write to this register with the input value will trigger a QUADF operation and the result will be present in R5 and R6 result register
- QUADF_R6_R7 - QUADF operation's seventh operand1 register present at memory location 0x601F0. Write to this register with the input value will trigger a QUADF operation and the result will be present in R6 and R7 result register

In use cases where the Operation takes in two inputs for instance QUAD operation, the second input value needs to be put into QUADF32_DIVF32_OP2 operand register present at memory location 0x60240. This needs to be done first before writing the first input value into the operand1 register.


## Context Save and Restore

TMU result registers supports context save functionality, which can enable the use of TMU in an ISR context while simultaneously used in main function

When an interrupt occurs and ISR starts running, the context save can be initiated by writing ‘1’ to the SAVE bit of CONTEXT_SAVE register present at memory location 0x60308. TMU result register values are saved in the corresponding Context Save register. There are 8 Context save registers, and there is a mapping between the result registers and context save registers. The R0 result register value gets stored in Context Save r0 register, R1 result register value gets stored into Context Save r1 register and so on. This context save of result registers will only happen once all the TMU operations initiated before writing to CONTEXT_SAVE register is complete, this ensures context save happens at correct point. After saving the context, ISR can use TMU without any restriction

Restoring TMU result registers with value present in Context Save registers can be initiated by writing ‘1’ to the RESTORE bit present in CONTEXT_RESTORE register. This register is present at the memory location 0x60310.
The source file for TMU operations supports two macros, **ISR_TMU_CONTEXT_SAVE** and **ISR_TMU_CONTEXT_RESTORE**. ISR_TMU_CONTEXT_SAVE will initiate a write to the SAVE bit of the CONTEXT_SAVE register. Users can directly use this macro in their code to initiate the Context Save operation. ISR_TMU_CONTEXT_RESTORE will initiate a write to the RESTORE bit of the CONTEXT_RESTORE register, users can directly use this macro in their code to initiate the Context Restore operation


\imageStyle{tmu_context_save_dev.png,width:60%}
\image html tmu_context_save_dev.png "TMU Context Save and Restore workflow"


## Compiler Support
There is no compiler support for TMU, the users will have to use SDK for TMU operations


## Steps to use TMU APIs

In SDK we have integrated TMU support using register independent inline assembly code for each of the functions that is supported by TMU. The SDK contains support for writing into one of the OPERAND 1 register supported by an operation. The TMU functionalities supported by SDK includes:

- **SINPUF** - The TMU source file supports API **'ti_tmu_sin_pu'** which takes in an input value that is in per unit representation and returns sine value. This function writes this input value into sin's first operand1 register(0x60040) and the result will get generated into R0 result register(0x60280). User's can make a direct call to this API in their application code to initiate a SIN operation using TMU. To generate the result in other result registers, user's can add API calls in the TMU source file that takes in the input value and does the write to another sin Operand 1 register which will generate the result to a different result register supported by TMU.


- **SIN** - The TMU source file supports API **'ti_tmu_sin'** which takes in an input value that is in radians representation and returns sine value. This function will first convert the input value from radians to per unit by multiplying the input with 1/2PI, and will then write this value into sin's first operand1 register(0x60040). The result will get generated into R0 result register(0x60280). User's can make a direct call to this API in their application code to initiate a SIN operation using TMU. To generate the result in other result registers, user's can add API calls in the TMU source file that takes in the input value and does the write to another sin operand1 register which will generate the result to a different result register supported by TMU.

- **COSPUF** - The TMU source file supports API **'ti_tmu_cos_pu'** which takes in an input value that is in per unit representation and returns cosine value. This function writes this input value into cos's first operand1 register(0x60080) and the result will get generated into R0 result register(0x60280). User's can make a direct call to this API in their application code to initiate a COS operation using TMU. To generate the result in other result registers, user's can add API calls in the TMU source file that takes in the input value and does the write to another cos operand1 register which will generate the result to a different result register supported by TMU.

- **COS** - The TMU source file supports API **'ti_tmu_cos'** which takes in an input value that is in radians representation and returns cosine value. This function will first convert the input value from radians to per unit by mutiplying the input with 1/2PI, and will then write this value into cos's second operand1 register(0x60088). The result will get generated into R1 result register(0x60288). User's can make a direct call to this API in their application code to initiate a COS operation using TMU. To generate the result in other result registers, user's can add API calls in the TMU source file that takes in the input value and does the write to another cos operand1 register which will generate the result to a different result register supported by TMU.

- **ATANPU** - The TMU source file supports API **'ti_tmu_atan_pu'** which takes in an input value that is in per unit representation and returns atan value. This function writes this input value into atan's third operand1 register(0x600D0) and the result will get generated into R2 result register(0x60290). User's can make a direct call to this API in their application code to initiate a ATAN operation using TMU. To generate the result in other result registers, user's can add API calls in the TMU source file that takes in the input value and does the write to another atan operand1 register which will generate the result to a different result register supported by TMU.

- **ATAN** - The TMU source file supports API **'ti_tmu_atan'** which takes in an input value that is in radians representation and returns atan value. Tis function will first write the input value write this value into atan's third operand1 register(0x60D0). The result will get generated into R2 result register(0x60290), this value is multiplied with 2PI to convert it to per unit form. User's can make a direct call to this API in their application code to initiate a ATAN operation using TMU. To generate the result in other result registers, user's can add API calls in the TMU source file that takes in the input value and does the write to another atan operand1 register which will generate the result to a different result register supported by TMU.

- **ATAN2 using QUAD and ATAN** - The TMU source file supports API **'ti_tmu_atan2'** which takes in two input values x and y that is in radians representation and returns atan2 value. This function will first store the second input to OPERAND 2 register(0x60240) and then store the first input into QUAD's seventh operand1 register(0x601F0) which will generate result into the R6 and R7 result registers. R6 will contain ratio of the two inputs and R7 will contain the quadrant value of the two inputs. The ratio value is then taken and stored into atan's first operand1(0x600C0) register which will generate result into the R0 result register. This atan value(present in R0) and quadrant value present in (R7) is added togetehr and then multiplied with 2PI to get ATAN2 value. User's can make a direct call to this API in their application code to initiate a ATAN2 operation using TMU. To generate the result in other result registers, user's can add API calls in the TMU source file that takes in the input value and does the write to another atan/quadrant operand1 register which will generate the result to a different result register supported by TMU.

- **IEXP to base 2** - The TMU source file supports API **'ti_tmu_iexp_pu'** which takes in one input value and computes inverse exponential value to the base 2. This function writes this input value into iexp's fourth operand1 register(0x60158) and the result will get generated into R3 result register(0x60298). User's can make a direct call to this API in their application code to initiate a IEXP operation using TMU. To generate the result in other result registers, user's can add API calls in the TMU source file that takes in the input value and does the write to another iexp Operand 1 register which will generate the result to a different result register supported by TMU.

- **IEXP to base e** - The TMU source file supports API **'ti_tmu_iexp_e_pu'** which takes in one input value and computes inverse exponential value to the base e. This function first multiples the input with log2(e) to change the base and then writes this value into iexp's third operand1 register(0x60110). The result will get generated into R2 result register(0x60290). User's can make a direct call to this API in their application code to initiate a IEXP operation using TMU. To generate the result in other result registers, user's can add API calls in the TMU source file that takes in the input value and does the write to another iexp Operand 1 register which will generate the result to a different result register supported by TMU.

- **LOG to base 2** - The TMU source file supports API **'ti_tmu_log_pu'** which takes in one input value and computes logarithmic value to the base 2. This function writes this input value into log's fourth operand1 register(0x60198) and the result will get generated into R3 result register(0x60298). User's can make a direct call to this API in their application code to initiate a LOG operation using TMU. To generate the result in other result registers, user's can add API calls in the TMU source file that takes in the input value and does the write to another log Operand 1 register which will generate the result to a different result register supported by TMU.

- **LOG to base e** - The TMU source file supports API **'ti_tmu_log_e_pu'** which takes in one input value and computes logarithmic value to the base e. This function writes this input value into log's fourth operand1 register(0x60198) and the result will get generated into R3 result register(0x60298). This result is then multiplied with the reciprocal of log2(e). User's can make a direct call to this API in their application code to initiate a LOG operation using TMU. To generate the result in other result registers, user's can add API calls in the TMU source file that takes in the input value and does the write to another log Operand 1 register which will generate the result to a different result register supported by TMU.

- **SINCOSPUF** - The TMU source file supports API **'ti_tmu_sincos_pu'** which takes in three inputs, the input value is per unit form, address of a variable that stores result of sin operation and address of a variable that will store result of cos operation. This function will first write the input value into the sin's first operand1 register(0x60040) and then write the input value into cos's second operand1 register(0x60080), so the sin result will get generated in R0 result register and cos result will get generated in R1 result register. A delay of 4 NOP's to ensure the computation is complete before result is read, post this the value present in R0 result register is assigned to sin result variable and the value present in the R1 result register is assigned to cos result variable. User's can make a direct call to this API in their application code to initiate a SINCOS back-to-back operation using TMU. To generate the result in other result registers, user's can add API calls in the TMU source file that takes in the input value and does the write to another sin or cos Operand 1 register which will generate the result to a different result register supported by TMU.

- **SINCOS** - The TMU source file supports API **ti_tmu_sincos** which takes in three inputs, the input value is radiant form, address of a variable that stores result of sin operation and address of a variable that will store result of cos operation. This function will first convert the input value from radians to per unit by multiplying with 1/2PI. This value is written into sin's first operand1 register(0x60040) and then cos's second operand1 register(0x60080), so the sin result will get generated in R0 result register and cos result will get generated in R1 result register. A delay of 4 NOP's is added to ensure the computation is complete before result is read, post this the value present in R0 result register is assigned to sin result variable and the value present in the R1 result register is assigned to cos result variable. User's can make a direct call to this API in their application code to initiate a SINCOS back-to-back operation using TMU. To generate the result in other result registers, user's can add API calls in the TMU source file that takes in the input value and does the write to another sin or cos Operand 1 register which will generate the result to a different result register supported by TMU.


See also the TMU examples code and documentation for more information on how TMU can be used

