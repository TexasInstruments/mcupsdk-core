#!/usr/bin/python3
import fileinput
import argparse
import os

################################################################################
##                          Main program starts here                          ##
################################################################################

parser = argparse.ArgumentParser(prog='setup.py', formatter_class=argparse.RawTextHelpFormatter,
        description='setup.py - Setup script for K3 Resource Partitioning Tool')
parser.add_argument('-s', '--sysconfig_dir', required=True, dest='sysconfig_dir',
        action='store', help='Path to SysConfig installation directory')
args = parser.parse_args()

path = os.path.join(args.sysconfig_dir, "dist")
path = os.path.join(path, "ui.js")

with fileinput.FileInput(path, inplace=True, backup='.bak') as file:
    for line in file:
        print(line.replace('html(){return"<p>$$$$ommitted_html$$$$</p>"}', ''), end='')

print("Setup finished successfully.\n")
