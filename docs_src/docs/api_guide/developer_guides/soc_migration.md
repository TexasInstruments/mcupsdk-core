# Migrating examples from package to package in @VAR_SOC_NAME {#SOC_MIGRATION}

[TOC]

## Installing dependencies

- Install SysConfig version 1.16.0 or above from [here](https://www.ti.com/tool/SYSCONFIG#downloads)
- Verify that node is installed in your system. You can check this by entering this command in your terminal/command prompt.
<br>
```bash
$ node --version
```
- If not present, download it from [here](https://nodejs.org/en/download). **Download v14.0.0 or above.**

## Setup

- The migration script titled **migrate.js** is situated inside the "{your_sdk_path}/tools/migration_script" folder.

- The script can take in total 8 arguments, apart from **node** and **migrate.js**:

    1. Absolute path to product.json: {sdk_path}/.metadata/product.json ```(--product)```
    2. Device from which you are migrating: the existing sysconfig files are present for this device. ```(--sourceDevice)```
    3. Device to which you are migrating ```(--destinationDevice)```
    4. Package from which you are migrating ```(--sourcePackage)```
    5. Package to which you are migrating ```(--destinationPackage)```
    6. Part from which you are migrating ```(--sourcePart)```
    7. Part to which you are migrating ```(--destinationPart)```
    8. Absolute path to the examples you want to migrate ```(--migratePath)```

- The package and part arguments can be skipped if not required.

\note The script is not validated for part migration yet.

\cond SOC_AM263PX
## Migrating examples to SIP (@VAR_SOC_MIGRATE_TO_PACKAGE) package
\endcond

\cond SOC_AM273X
## Migrating examples to @VAR_SOC_MIGRATE_TO_PACKAGE package
\endcond

To run the script, open terminal/command prompt inside the SDK folder that has the migrate.js script.

In this case, navigate to migration_script folder using:

    $ cd tools/migration_script


For example, let's say you want to switch all the @VAR_SOC_NAME examples in the whole of SDK from package **@VAR_SOC_DEFAULT_PACKAGE** to **@VAR_SOC_MIGRATE_TO_PACKAGE**. Then, enter this command in your terminal:

    $ node migrate.js --product {your_sdk_path}/.metadata/product.json --sourceDevice @VAR_SOC_NAME --destinationDevice @VAR_SOC_NAME --sourcePackage @VAR_SOC_DEFAULT_PACKAGE --destinationPackage @VAR_SOC_MIGRATE_TO_PACKAGE --migratePath {your_sdk_path}


The arguments **sourceDevice** and **destinationDevice** are same in this case, both @VAR_SOC_NAME.

When you run this command, the migrate.js script recursively traverses through the whole SDK path, searches for all the examples that are currently in **sourcePackage**, and then migrates those examples to **destinationPackage**.

You can also use the script to switch individual examples. Let's say you want to switch only the 'mcasp' examples, than replace the last argument with:

    "{your_sdk_path}/examples/drivers/mcasp"

It does in-place modification of **example.syscfg**, **makefile**, and **example.projectspec** files of the examples.

The list of files successfully migrated and files that failed to migrate can be observed from the **log_file.txt** generated inside the tools/migration_script folder of the SDK.

To understand the list of arguments, their aliases and description enter this command:

    $ node migrate.js --help


### Excluding examples from migration

To skip certain examples from being migrated, add the SDK relative path to those folders in the file **exclude_list_@VAR_SOC_NAME_LOWER.js** inside migration_script/soc folder.
