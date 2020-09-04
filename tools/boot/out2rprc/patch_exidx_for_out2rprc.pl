#!/usr/bin/perl
#The above line invokes perl from the current path

#
# Copyright (c) Texas Instruments Incorporated 2019
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
#
#   Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the
#   distribution.
#
#   Neither the name of Texas Instruments Incorporated nor the names of
#   its contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#

#
# This script file patches an elf binary to force the exidx section to be set 
# to type "PROGBITS". This makes the out2rprc tool fill in the exidx section 
# in the converted image. This is only required for ELF files containing an 
# "exidx" section created by GNU toolchains typically for C++ exception 
# handling on ARM A8/A9/A15. Pure C/assembly executables usually do not 
# contain this section. Presence of exidx section can be verified by 
# inspecting the output .map file. The patching is performed in place.
#
# To use Perl with PRSDK, please see: 
#     http://software-dl.ti.com/processor-sdk-rtos/esd/docs/06_01_00_08/rtos/index_overview.html?highlight=strawberry#command
#
# Windows host: download Strawberry perl
# Ubuntu host: sudo apt-get install perl-base
# Other Linux host: Use your distro's package manager to install a perl interpreter
#

#=============================================================================
# Code starts here ...
#=============================================================================

use strict 'vars';

die "Syntax: $0 elfile.out\n" if ($#ARGV != 0);

my ($elfFile) = @ARGV;

open FILE, "+<$elfFile" or die "Can't open $elfFile\n";
binmode FILE or die "Can't binmode\n";

# Check magic
die "$elfFile not elf\n" if (read32(0) != 0x464c457f);

# Find section table and names
my $sect_off   = read32(0x20);
my $sect_ents  = read16(0x30);
my $string_idx = read16(0x32);

# Find string table
my $string_sect_hdr = $string_idx * 0x28 + $sect_off;
my $string_off = read32($string_sect_hdr + 0x10);

# Find a section named ".ARM.extab";
for (my $i = 0; $i < $sect_ents; $i++) {
	my $sectoffset = 0x28 * $i + $sect_off;
	my $nameoffset = $string_off + read32 ($sectoffset);
	my $name = readasciiz($nameoffset);
	print "Section $i: $name\n";
	if ($name eq ".ARM.exidx") {
		my $type = read32($sectoffset + 4);
		if ($type == 0x70000001) {
			# Change it to 1
			write32($sectoffset + 4, 0x00000001);
			# Check it
			if (read32($sectoffset + 4) == 0x00000001)
			{
				print "Successfully patched type to PROGBITS\n";
				exit(0);
			} else {
				die "Type write failed\n";
			}
		} else {
			printf "Found unexpected type 0x%08x\n", $type;
			die "Failure\n";
		}
	}
}

# Read 32 bits binary little endian at specific offset
sub read32 {
	my ($offset) = @_;

	seek FILE, $offset, SEEK_SET or die "Can't seek $offset\n";

	my (@val);

	push @val, ord getc FILE;
	push @val, ord getc FILE;
	push @val, ord getc FILE;
	push @val, ord getc FILE;

	return ($val[3] << 24) | ($val[2] << 16) | ($val[1] << 8) | $val[0];
}

# Write 32 bits binary little endian at specific offset
sub write32 {
	my ($offset, $val) = @_;

	seek FILE, $offset, SEEK_SET or die "Can't seek $offset\n";

	my ($bytes);

	# write little endian one byte at a time
	print FILE chr(($val      ) & 0xff);
	print FILE chr(($val >>  8) & 0xff);
	print FILE chr(($val >> 16) & 0xff);
	print FILE chr(($val >> 24) & 0xff);
}

# Read 16 bits binary little endian at specific offset
sub read16 {
	my ($offset) = @_;

	seek FILE, $offset, SEEK_SET or die "Can't seek $offset\n";

	my (@val);

	push @val, ord getc FILE;
	push @val, ord getc FILE;

	return ($val[1] << 8) | $val[0];
}

# Read an ASCIIZ at offset
sub readasciiz {
	my ($offset) = @_;

	seek FILE, $offset, SEEK_SET or die "Can't seek $offset\n";

	my (@vals, $val);

	do {
		my $valc = getc FILE;
		$val = ord $valc; # turn into a number
		push @vals, $valc if ($val);
	} while ($val);

	return join ("", @vals);
}

__END__
