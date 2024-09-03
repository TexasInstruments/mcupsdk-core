
#  Build, Load and Run Example {#BUILD_LOAD_RUN_EXAMPLE}

\note For quick start tutorial on MCU+SDK under 10 minutes, please watch [Getting started with AM26x microcontrollers and MCU PLUS SDK](https://www.youtube.com/watch?v=1ggz1AeIoIA)

## Application Execution Flow

  \imageStyle{complete_bootflow.png,width:100%}
  \image html complete_bootflow.png "Build, load and run application"


## Building an application

\note SDK supports building applications via both **Makefile** and **CCS**. 
		- `makefile_ccs_bootimage_gen` file present inside a CCS project will have steps involved in building an application via CCS.
		- `makefile` present inside each example folder consists of the steps involved in building the application via make.

\note Refer \ref GETTING_STARTED_BUILD for an example of CCS and Makefile based build.


- Building a particular example project compiles all source files present in drivers, examples and sysconfig, links them, then generates a .out binary.

- Using the .out, we generate .appimage / .mcelf images.
	
- Once a multicore application image is built, we can load it into memory.

- .out file can be directly loaded into CCS. Refer \ref CCS_LOAD_RUN

For detailed information on building visit \ref BUILDING_APPLICATION

## Loading an application

- Loading is when an application is written into RAM/Flash memory.

- There are 3 ways to load a binary:
	- via CCS
	- via TI Uniflash tool
	- via Python scripts

- Once the application is loaded, we can run it.

For detailed information on loading visit \ref LOADING_APPLICATION

## Running an application

- Bootloaders are responsible for reading the loaded application and running it on the respective CPU.

- Depending on the boot media, there are different types of SBLs that can be used.

- If application is found in the address specified to bootloader, it parses the file, loads into respective CPUs and releases core from reset.

- Booting an RPRC image: \ref BOOTFLOW_RPRC_BOOT
- Booting an MCELF image: \ref BOOTFLOW_MCELF_BOOT

For detailed information on loading visit \ref RUNNING_APPLICATION