# FIREWALL {#DRIVERS_FIREWALL_PAGE}

[TOC]
Firewall is a module used to implement overall SoC security by providing a mechanism to assign and restrict device resources to a given main entity or Secure/Non-secure/Priv/User world.
The Firewall driver provides API to perform initialization and configuration of regions.

## Features Supported

- Support a multi number of regions, depending on the firewall.
- Two overlapping regions
- Lock Mode and Cache Mode
- Support up to 3 privilege and permission slots

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure below parameters
- Firewall ID and Region Instances
- Number of Regions
- Region Index
- Lock, Cache, Background mode selection
- Start Address and End Address
- Transaction permission like Priv-ID, Read, Write, Debug, Cacheable Configuration

\note Check enable manual address in SysConfig to enter manual region configuration.

## Features NOT supported

- More than 1 overlap regions not supported.
- The minimum memory region size is 4KB.
- DMSC targets regions are not supported.

## Important Usage Guidelines

- There can be only one background region per firewall. Foreground regions can have overlapping
addresses only with the background region
- In case two regions overlap, foreground region takes precedence and its permissions are taken into effect.
- The regions must be 4KB aligned.

## Example Usage

Include the below file to access the APIs
\snippet Firewall_sample.c include

Instance open Example
\snippet Firewall_sample.c open

Instance close Example
\snippet Firewall_sample.c close

Config Firewall region Example
\snippet Firewall_sample.c cfgfwlReg


## API

\ref DRV_FIREWALL_MODULE