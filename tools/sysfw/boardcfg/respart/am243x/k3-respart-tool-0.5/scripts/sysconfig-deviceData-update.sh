#!/bin/bash

# Script to patch Sysconfig tool to enable devices that are not yet officially supported

# Prerequisites:
# - Make sure you have installed Sysconfig tool. https://www.ti.com/tool/download/SYSCONFIG 

# Usage:
#  ./sysconfig-deviceData-update.sh <soc> --sysconfig_dir=/path/to/sysconfig/installation
# Example usage:
# ./sysconfig-deviceData-update.sh j784s4 --sysconfig_dir=/path/to/sysconfig/installation

# Parse CLI arguments
soc=$1
sysconfig=""

for i in "$@"; do
    case $i in
        -s=*|--sysconfig_dir=*) # Path to Sysconfig Installation Directory
            sysconfig="${i#*=}"
            shift
            ;;  
        -h|--help)
        echo "Usage: $0 \<soc\> --sysconfig_dir=/path/to/sysconfig/installation"
        echo 
        echo "Supported SOCs:-"
        echo "- j784s4"
        echo ""
        exit 0
        ;;
        -*)
            echo "!!!WARNING!!! - IGNORING INVALID FLAG: $i"
            shift
            ;;
    esac
done


# Check CLI arguments
if [ "${soc}" != "j784s4" ]; then
    echo "!!!ERROR!!! - Invalid SOC: ${soc}"
    exit 1
fi

if [ "${sysconfig}" == "" ] || [ ! -d ${sysconfig} ]; then
    echo "!!!ERROR!!! - Invalid Sysconfig Installation Directory: ${sysconfig}"
    exit 1
fi

# Specify paths relative to script
SCRIPT=$(readlink -f $0)
SCRIPTPATH=`dirname $SCRIPT`
K3RESPARTDIR=`dirname $SCRIPTPATH`

# Read devices.json content for ${soc}
devices_update_file="${K3RESPARTDIR}/deviceData/${soc}/devices_${soc}.json"
IFS=$'\n' read -d '' -r -a devices_update_content < "${devices_update_file}"
devices_update_content=$(echo "${devices_update_content[@]}" | tr -d '\n')

# update Sysconfig dist/deviceData
cp -r ${K3RESPARTDIR}/deviceData/${soc}/. ${sysconfig}/dist/deviceData

# Update devices.json to enable ${soc}
sed -i "s/{\"devices\":\[/{\"devices\":\[${devices_update_content}/g" "${sysconfig}/dist/deviceData/devices.json"

echo "Successfully updated Sysconfig tool to support ${soc}"
