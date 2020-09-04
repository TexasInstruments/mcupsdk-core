# Script to create a binary blob of boardcfgs

import argparse
import os
import subprocess
from re import sub
from random import randint
import shutil
import struct

# Boardcfg types
BOARDCFG     = 0xB
BOARDCFG_RM  = 0xC
BOARDCFG_SEC = 0xD
BOARDCFG_PM  = 0xE

temp_desc_file_name = "desc_file.tmp"
temp_bcfg_file_name = "bcfg_file.tmp"

def create_blob(args):
	if((not os.path.exists(args.bcfg)) or (not os.path.exists(args.bcfg_rm)) or (not os.path.exists(args.bcfg_pm)) or (not os.path.exists(args.bcfg_sec))):
		# We need all the boardcfgs to make the blob, so if even one is not present exit with fail (this is just a restriction we impose for simplicity)
		print("One of the boardcfg files provided do not exist, exiting blob creation ...")
		exit(2)

	# Assume all boardcfg elements are present
	num_elems = 4
	sw_rev = args.sw_rev
	devgrp = args.devgrp

	if(sw_rev is None):
		sw_rev = 1

	if(devgrp is None):
		devgrp = 0

	# All files exist, continue
	out_filename = args.output_file

	if(out_filename is None):
		out_filename = "sysfw_boardcfg_blob.bin"

	# Write number of elements and swrev to the outfile
	outfile_fh = open(out_filename, 'wb+')

	# Write the header bytes which contain SWREV and number of boardcfg elements
	header_bytes = outfile_fh.write(struct.pack('<BB', num_elems, sw_rev))

	# Structure of the blob will be like
	'''
	 -----------------------
	|       NUM ELEMS       |
	 -----------------------
	|         SWREV         |
	 -----------------------           -----------------------
	|     BOARDCFG_DESC_1   |-------->|      BOARDCFG_TYPE    |
	 -----------------------           -----------------------
	|     BOARDCFG_DESC_2   |         |     BOARDCFG_OFFSET   |
	 -----------------------           -----------------------
	|     BOARDCFG_DESC_3   |         |          SIZE         |
	 -----------------------           -----------------------
	|     BOARDCFG_DESC_4   |         |         DEVGRP        |
	 -----------------------           -----------------------
	|       BOARDCFG_1      |         |          RSVD?        |
	 -----------------------           -----------------------
	|       BOARDCFG_2      |
	 -----------------------
	|       BOARDCFG_3      |
	 -----------------------
	|       BOARDCFG_4      |
	 -----------------------
	'''
	# So make descriptors and keep adding them to a different file.
	boardcfg_desc_fmt = '<HHHBB'

	desc_file = open(temp_desc_file_name, "wb+")
	bcfg_file = open(temp_bcfg_file_name, "wb+")

	# Offset of first boarcfg would be at num_elems+swrev+num_elems*desc_size bytes
	offset = header_bytes + num_elems * struct.calcsize(boardcfg_desc_fmt)

	bcfg_list = [(args.bcfg, BOARDCFG), (args.bcfg_sec, BOARDCFG_SEC), (args.bcfg_pm, BOARDCFG_PM), (args.bcfg_rm, BOARDCFG_RM)]

	for bcfg_obj in bcfg_list:
		with open(bcfg_obj[0], 'rb') as f:
			bcfg = f.read()
			size = len(bcfg)
			desc = struct.pack(boardcfg_desc_fmt, bcfg_obj[1], offset, size, devgrp, 0)
			desc_file.write(desc)
			bcfg_file.write(bcfg)
			offset += size

	desc_file.seek(0)
	bcfg_file.seek(0)

	# copy the desc file to out file
	shutil.copyfileobj(desc_file, outfile_fh)
	shutil.copyfileobj(bcfg_file, outfile_fh)

	# close the files
	outfile_fh.close()
	desc_file.close()
	bcfg_file.close()

	os.remove(temp_desc_file_name)
	os.remove(temp_bcfg_file_name)

	print("SYSFW Boardcfg blob created at {}".format(os.path.abspath(out_filename)))

my_parser = argparse.ArgumentParser(description="Creates a binary blob of boardcfg to be loaded to SYSFW data section in ROM combined boot")
my_parser.add_argument('--sw-rev',      type=int, help='Software revision')
my_parser.add_argument('--devgrp',      type=int, help='Device Group')
my_parser.add_argument('--bcfg',        type=str, help='Path to the baseport boardcfg binary file')
my_parser.add_argument('--bcfg-pm',     type=str, help='Path to the boardcfg (PM) binary file')
my_parser.add_argument('--bcfg-rm',     type=str, help='Path to the boardcfg (RM) binary file')
my_parser.add_argument('--bcfg-sec',    type=str, help='Path to the boardcfg (SECURITY) binary file')
my_parser.add_argument('--output-file', type=str, help='Path to the output file')

args = my_parser.parse_args()

create_blob(args)
