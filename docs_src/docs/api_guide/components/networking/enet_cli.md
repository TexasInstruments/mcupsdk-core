# Enet CLI {#ENET_CLI}

[TOC]

## Introduction

The Enet CLI library provides a command line interpreter for networking applications.

## Features

- APIs to easily enable CLI in any networking application.
- Built-in commands which can aid in debugging.
- API to create custom commands in application.
- Portable to any command interpreter library. (Currently uses FreeRTOS_Plus_CLI.)

### List of Built-In Commands

 Module | Command | Description
 -------|---------|------------
 enet_cfg | `enet_cfg mqprio <port_num> [{-i❘-e} <map0> ... <map7>]` | Modifies priority remap registers on the CPSW. Use `-i` tag to modify VLAN priority regeneration map. Use `-e` to modify QoS egress priority map. Specifying only the `port` will show the current remap register values of the port.
 ^ | `enet_cfg tracelvl [<lvl>]`| Sets the trace level for enet processes to `lvl`. Refer to [this](\ref ENET_TRACE_API) for more information on trace levels. Prints current trace level if `lvl` is not specified
 ^ | `enet_cfg classifier [-r] [-c <rx_chn>] [-e <ether_type>] [-p <mac_port>] [-sm <src_mac_addr>] [-dm <dest_mac_addr>] [-sip <src_ipv4_addr>] [-dip <dest_ipv4_addr>] [-iv <inner_vid>] [-ov <outer_vid>] [-pcp <priority>]` | Configures a classifier with the specified rules. Use the `-r` tag to remove the classifer with the specified rules. Specifying `rx_chn` along with `-r` tag will remove all classifiers associated to the specified Rx channel.
 ^ | `enet_cfg help` | Prints a list of all commands available in the `enet_cfg` module.
 enet_dbg | `enet_dbg cpswstats <port_num> [-r]` | Prints the CPSW statistics data of the specified port. Use the `-r` tag to reset the data after its printed.
 ^ | `enet_dbg dumpale` | Prints the ALE table. (Works only in debug mode.)
 ^ | `enet_dbg dumppolicer` | Prints the policer table.
 ^ | `enet_dbg help` | Prints a list of all commands available in the `enet_dbg` module.
 phy | `phy_scan` | Prints a list of available PHYs.
 ^ | `phy status [<mac_port>]` | Prints the link status of the PHY at the specified MAC port. Prints the link status of all PHYs, if `mac_port` is not specified.
 ^ | `phy dump <mac_port>` | Prints the data of PHY registers at the specified MAC port.
 ^ | `phy write <mac_port> <addr> <data>` | Writes the `data` to the PHY register with address `addr` attached to the specified MAC port.
 ^ | `phy read <mac_port> <addr>` | Prints the data stored in the PHY register with address `addr` attached to the specified MAC port.
 ^ | `phy help` | Prints a list of all commands available in the `phy` module.
 utils | `utils cpuload` | Prints current CPU usage.
 ^ | `utils readmem <start_addr> [<num_of_words>]` | Prints value stored in memory starting from `start_addr`. Prints only one word if `num_of_words` is not specified.
 ^ | `utils writemem <start_addr> <word1> ...` | Writes to memory starting from `start_addr`.
 ^ | `utils help` | Prints a list of all commands available in the `utils` module.
 
## Application Programming Interface
 
For details on the APIs, refer to \ref ENET_CLI_API.

## Demo and Examples

\ref EXAMPLES_ENET_CLI_APP

## Usage Guide

- To use the built-in commands, the enet_cli library uses certain data (like enet handle, core ID, and so on) that needs to be initialized first using `EnetCli_init()`.
- Enable the built-in commands using the `EnetCli_registerBuiltInCommands()` function.
- To process a command, use the `EnetCli_processCommand()` function. A command can return more than one line of output and the function might be required to run several times to get the full output. A sample code snippet to handle this case:
\code
...
bool moreDataToFollow;
...
/* Continuously take commands from the user */
while (1)
{
    DebugP_scanf(rxBuffer, MAX_READ_BUFFER_LEN);
    /* Repeatedly call the process function until function returns false.
       The function will return false once no more output data is remaining.*/
    do
    {
        moreDataToFollow = EnetCli_processCommand(rxBuffer, txBuffer,
            MAX_WRITE_BUFFER_LEN);
        DebugP_log(txBuffer);
        memset(txBuffer, 0x00, MAX_WRITE_BUFFER_LEN);
    } while (moreDataToFollow);
    memset(rxBuffer, 0x00, MAX_READ_BUFFER_LEN);
}
...
\endcode

## Adding Custom Commands

Depending on the command interpreter used, the procedure to add custom commands will vary.

### FreeRTOS_Plus_CLI

- Define the functions that will be executed when the custom commands are used. Refer to the [FreeRTOS+CLI documentation](https://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_CLI/FreeRTOS_Plus_CLI_Implementing_A_Command.html) for more information.
- Define an array of commands. The array should be of the type `CLI_Command_Definition_t`. For more information on defining commands refer to the [FreeRTOS+CLI documentation](https://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_CLI/FreeRTOS_Plus_CLI_Registering_A_Command.html).
- Use the `EnetCli_registerCustomCommands()` function to register the commands to the command interpreter.
- The custom commands will automatically be processed when using the `EnetCli_processCommand()` function.
