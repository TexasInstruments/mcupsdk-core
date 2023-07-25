# Migrating examples from package to package in AM273x {#SOC_MIGRATION}

[TOC]

\note This script is validated only for AM273X ZCE to NZN package migration.

## Installing dependencies

- Install SysConfig version 1.16.0 or above from [here](https://www.ti.com/tool/SYSCONFIG#downloads)
- Verify that node is installed in your system. You can check this by entering this command in your terminal/command prompt.
<br>
```bash
$ node --version
```
- If not present, download it from [here](https://nodejs.org/dist/v12.18.4/). v12.18.4 is preferred.

## Setup

- The migration script titled **migrate.js** is situated inside the "{your_sdk_path}/tools/migration_script" folder.

- The script takes in 6 arguments, apart from **node** and **migrate.js** in this order:

    1. Absolute path to product.json: {sdk_path}/.metadata/product.json
    2. Device from which you are migrating: the existing sysconfig files are present for this device.
    3. Device to which you are migrating
    4. Package from which you are migrating: ZCE is the defaualt package for am273x in the SDK.
    5. Package to which you are migrating: in this case all sysconfig configuration will be migrated to NZN.
    6. Absolute path to the examples you want to migrate.

- Example:
```bash
    node migrate.js {absolute to product.json} {deviceFrom} {deviceTo} {packageFrom} {packageTo} {absolute sdk or example path}
```

## Migrating examples

To run the script, open terminal/command prompt inside the SDK folder that has the migrate.js script.

In this case, navigate to migration_script folder using:
```bash
$ cd tools/migration_script
```

For example, let's say you want to switch all the am273x examples in the whole of SDK from package **ZCE** to **NZN**. Then, enter this command in your terminal:
```bash
$ node migrate.js "{your_sdk_path}/.metadata/product.json" "AM273x" "AM273x" "ZCE" "NZN" "{your_sdk_path}"
```
The arguments **deviceFrom** and **deviceTo** are same in this case, both AM273x.

When you run this command, the migrate.js script recursively traverses through the whole SDK path, searches for all the examples that are currently in **packageFrom**, and then migrates those examples to **packageTo**.

You can also use the script to switch individual examples. Let's say you want to switch only the 'mcasp' examples, than replace the last argument with:
```
"{your_sdk_path}/examples/drivers/mcasp"
```
It does in-place modification of **example.syscfg**, **makefile**, and **example.projectspec** files of the examples.

The list of files successfully migrated and files that failed to migrate can be observed from the **log_file.txt** generated inside the tools/migration_script folder of the SDK.
