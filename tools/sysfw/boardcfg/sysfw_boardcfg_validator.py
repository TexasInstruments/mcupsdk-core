#!/usr/bin/env python3
#
# System Firmware Board Configuration Validation
# Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
#
# This software is licensed under the standard terms and conditions in the
# Texas Instruments Incorporated Technology and Software Publicly
# Available Software License Agreement, a copy of which is included in
# the software download.
#
# Use this script to validate and sort System Firmware board configuration
# data contained within a compiled and linked binary image (.bin, .elf, etc)
# The script can be configured to output a new binary image with the
# sorted data.  In the case of RM board configuration resource entries, if
# the entries were pre-sorted prior to compile time the sort option is not
# needed.  However, it can be run anyway with no harm to the output binary.
# The script will throw an error when an output binary option is provided
# and the generated output binary size and input binary size do not match
# exactly.
#
# Two options are given to provide a log of the board configuration data
# and validation progress.  The options allow the logging to be directed
# to the console or to a file.

import os
import io
import sys
import json
import argparse
import textwrap
import struct
import operator
from collections import namedtuple
from collections import OrderedDict


class sysfw_boardcfg_rules:

    """ Class for processing the board configuration rules
    """
    dir_location = os.path.dirname(os.path.realpath(__file__))
    rules_file_name = 'sysfw_boardcfg_rules.json'

    def get_rules_file(self):
        """ Return the current rules file
        """
        return self.rules_file

    def set_rules_file(self, fname):
        """ Set up rules file
        """
        try:
            self.fh = io.open(
                fname,
                mode='r',
                encoding="utf8",
                errors='ignore')
        except Exception as ex:
            raise Exception(fname + ":  File not readable?:" + str(ex))
        # Make sure rules is a JSON file
        self.json_rules = json.load(self.fh, object_pairs_hook=OrderedDict)

        # Find all magic numbers to search for in binary
        self.magic_numbers = dict()
        for struct, description in self.json_rules['format_rules'].items():
            try:
                self.magic_numbers['struct_name'] = struct
                self.magic_numbers[description['subhdr']
                                   ['value']['magic']] = struct
            except BaseException:
                pass

        self.format_rules = self.json_rules['format_rules']
        self.validation_rules = self.json_rules['validation_rules']

        self.rules_file = fname
        return

    def get_bytes(self, fmt, mirror=True):
        fmt = '<' + fmt
        in_bytes = self.input_binary_class.get_bytes(struct.calcsize(fmt))
        u_bytes = struct.unpack(fmt, in_bytes)
        if self.output_binary_class and mirror:
            # Mirror bytes directly into a specified output binary
            self.output_binary_class.send_bytes(in_bytes)

        return u_bytes

    def get_substruct_size(self, fmt_entry):
        total_bytes = 0
        multiplier = 1
        incomplete_type = False
        for k, v in fmt_entry.items():
            if k == 'fmt':
                try:
                    total_bytes += self.get_substruct_size(
                        self.format_rules[v])
                except BaseException:
                    total_bytes += struct.calcsize(v)
            elif k == 'elements':
                multiplier = v
            elif k == 'incomplete_type':
                incomplete_type = True
            elif isinstance(v, dict):
                total_bytes += self.get_substruct_size(v)

        if incomplete_type:
            # Total bytes is zero for incomplete type as the last element in
            # structure
            total_bytes = 0

        return total_bytes * multiplier

    def is_substruct(self, fmt_entry):
        # Get substructure size
        try:
            fmt = self.format_rules[fmt_entry['subhdr']['fmt']]['size']['fmt']
        except BaseException:
            fmt = fmt_entry['subhdr']['fmt']

        u_bytes = self.get_bytes(fmt)[0]
        expected_bytes = self.get_substruct_size(
            self.format_rules[fmt_entry['subhdr']['value']['size']])
        self.output_class.send_next_line('Substructure size:')
        self.output_class.send_next_line('    Found    - %s' % u_bytes)
        self.output_class.send_next_line('    Expected - %s' % expected_bytes)
        if u_bytes != expected_bytes:
            self.output_class.send_next_line(
                'Not a valid boardcfg substructure.  Continuing search...')
            return False
        return True

    def get_elem_format(self, elem_desc):
        fmt = ''
        for k, v in elem_desc.items():
            if k == 'fmt':
                try:
                    fmt += self.get_elem_format(self.format_rules[v])
                except BaseException:
                    fmt += v
            elif isinstance(v, dict):
                fmt += self.get_elem_format(v)

        return fmt

    def validate_rm_resources(self, resources, validator):
        r_dict = [r._asdict() for r in resources]
        num_entries = len(r_dict)
        max_entries = 0

        for constraint in validator['constraints']:
            for k, v in constraint.items():
                if k == 'max_resource_entries':
                    max_entries = v
                    break
            if max_entries:
                break

        if num_entries > max_entries:
            self.output_class.send_next_line(
                'ERROR: Found %s resource entries when only %s allowed!' % (num_entries, max_entries))
            valid = False
        else:
            self.output_class.send_next_line(
                'Found %s resource entries' % num_entries)

            matched_r_dict = list()
            for entry in r_dict:
                valid = False
                closest_matches = list()
                e_start = entry['start_resource']
                e_end = entry['start_resource'] + entry['num_resource']

                for v_entry in validator['values']:
                    if entry['type'] == v_entry['type']:
                        v_start = v_entry['start_resource']
                        v_end = v_entry['start_resource'] + \
                            v_entry['num_resource']
                        closest_matches.append(v_entry)

                        if e_start >= v_start and e_end <= v_end:
                            if entry['num_resource'] != 0:
                                # Entries with a number of resources equal to
                                # zero are allowed but shouldn't be validated
                                # against adjacent ranges so do not pass them
                                # along for further validation
                                matched_r_dict.append(entry)
                            valid = True

                if not valid:
                    self.output_class.send_next_line(
                        'ERROR: Entry does not match any valid entries!')
                    for k, v in entry.items():
                        self.output_class.send_next_line('%s - %s' % (k, v))

                    self.output_class.send_next_line(
                        'Closest validation matches...')
                    if closest_matches:
                        for cm in closest_matches:
                            for k, v in cm.items():
                                self.output_class.send_next_line(
                                    '%s - %s' % (k, v))
                    else:
                        self.output_class.send_next_line('None')

                    break

            r_dict = matched_r_dict

        if valid:
            host_id_all = 128
            for pre, cur, post in zip(
                    r_dict[:-2:], r_dict[1:-1:], r_dict[2::]):
                pre_end = pre['start_resource'] + pre['num_resource'] - 1
                cur_end = cur['start_resource'] + cur['num_resource'] - 1

                if (cur['type'] == pre['type'] and
                    cur['type'] == post['type'] and
                    pre_end >= cur['start_resource'] and
                        pre_end >= post['start_resource']):
                    self.output_class.send_next_line(
                        'ERROR: Three overlapping ranges of the same type found!')

                    self.output_class.send_next_line(
                        '1st range:')
                    for k, v in pre.items():
                        self.output_class.send_next_line(
                            '%s - %s' % (k, v))
                    self.output_class.send_next_line(
                        '2nd range:')
                    for k, v in cur.items():
                        self.output_class.send_next_line(
                            '%s - %s' % (k, v))
                    self.output_class.send_next_line(
                        '3rd range:')
                    for k, v in post.items():
                        self.output_class.send_next_line(
                            '%s - %s' % (k, v))
                    valid = False
                    break

                if (cur['type'] == pre['type'] and
                    pre_end >= cur['start_resource'] and
                    (cur['host_id'] == host_id_all or
                     pre['host_id'] == host_id_all or
                     cur['host_id'] == pre['host_id'])):
                    self.output_class.send_next_line(
                        'ERROR: Adjacent ranges with at least one assigned to HOST_ID_ALL or both assigned to same host are overlapping!')

                    self.output_class.send_next_line(
                        '1st range:')
                    for k, v in pre.items():
                        self.output_class.send_next_line(
                            '%s - %s' % (k, v))
                    self.output_class.send_next_line(
                        '2nd range:')
                    for k, v in cur.items():
                        self.output_class.send_next_line(
                            '%s - %s' % (k, v))
                    valid = False
                    break

                if (cur['type'] == post['type'] and
                    cur_end >= post['start_resource'] and
                    (cur['host_id'] == host_id_all or
                     post['host_id'] == host_id_all or
                     cur['host_id'] == post['host_id'])):
                    self.output_class.send_next_line(
                        'ERROR: Adjacent ranges with at least one assigned to HOST_ID_ALL or both assigned to same host are overlapping!')

                    self.output_class.send_next_line(
                        '1st range:')
                    for k, v in cur.items():
                        self.output_class.send_next_line(
                            '%s - %s' % (k, v))
                    self.output_class.send_next_line(
                        '2nd range:')
                    for k, v in post.items():
                        self.output_class.send_next_line(
                            '%s - %s' % (k, v))
                    valid = False
                    break

        return valid

    def validate_substruct(self, fmt_entry, inline_sort):
        for k, v in fmt_entry.items():
            if inline_sort and 'sort_order' in v.keys():
                mirror_bytes = False
            else:
                mirror_bytes = True

            fmt = self.get_elem_format(v)

            if v['fmt'] in self.format_rules.keys():
                Structure = namedtuple('Structure', ' '.join(
                    self.format_rules[v['fmt']].keys()))

            if 'size_bytes' in v.keys():
                if isinstance(v['size_bytes'], int):
                    entries = v['size_bytes'] / struct.calcsize(fmt)
                elif isinstance(v['size_bytes'], str):
                    entries = int(fmt_entry[v['size_bytes']]
                                  ['value'] / struct.calcsize(fmt))

                fmt_entry[k]['value'] = list()
                for i in range(entries):
                    entry = Structure._make(self.get_bytes(fmt, mirror_bytes))
                    self.output_class.send_next_line(entry)
                    fmt_entry[k]['value'].append(entry)
            elif 'elements' in v.keys():
                if isinstance(v['elements'], int):
                    entries = v['elements']
                elif isinstance(v['elements'], str):
                    entries = fmt_entry[v['elements']]['value']

                fmt_entry[k]['value'] = list()
                for i in range(entries):
                    entry = Structure._make(self.get_bytes(fmt, mirror_bytes))
                    self.output_class.send_next_line(entry)
                    fmt_entry[k]['value'].append(entry)
            else:
                fmt_entry[k]['value'] = self.get_bytes(fmt, mirror_bytes)[0]

            if self.inline_sort and 'sort_order' in v.keys():
                self.output_class.send_next_line('Sorting entries...')
                fmt_entry[k]['value'].sort(
                    key=operator.attrgetter(
                        *v['sort_order']))

                for entry in fmt_entry[k]['value']:
                    self.output_class.send_next_line(entry)
                    if self.output_binary_class:
                        # Use * operator to unpack entry when passed to pack
                        self.output_binary_class.send_bytes(
                            struct.pack(fmt, *entry))
            if 'validator' in v.keys():
                self.output_class.send_next_line(
                    'Validating %s' % v['validator'])
                validator = self.validation_rules[v['validator']][self.soc]

                if v['validator'] == 'boardcfg_rm_resasg_entry':
                    if not self.validate_rm_resources(
                            fmt_entry[k]['value'], validator):
                        if self.output_binary_class:
                            # Delete the output binary if there's a failure
                            self.output_binary_class.delete()
                        self.output_class.send_next_line('Exiting...')
                        sys.exit(1)
                    else:
                        self.output_class.send_next_line('All entries valid')

    def process_data(self, soc, input_binary_class, output_class,
                     output_binary_class, inline_sort=False):
        self.soc = soc
        self.input_binary_class = input_binary_class
        self.output_class = output_class
        self.output_binary_class = output_binary_class
        self.inline_sort = inline_sort

        # Read two unsigned bytes at a time
        while (self.input_binary_class.size() -
               self.input_binary_class.position()) > 1:
            u_bytes = self.get_bytes('H')[0]
            for m in self.magic_numbers.keys():
                if hex(u_bytes) == m.lower():
                    self.output_class.send_next_line(
                        'Found magic number %s at byte position %s' %
                        (hex(u_bytes), self.input_binary_class.position() - struct.calcsize('H')))
                    if self.is_substruct(
                            self.json_rules['format_rules'][self.magic_numbers[m]]):
                        self.output_class.send_next_line(
                            'Parsing %s substructure...' % self.magic_numbers[m])

                        fmt_entry = self.json_rules['format_rules'][self.magic_numbers[m]].copy(
                        )
                        del fmt_entry['subhdr']
                        self.validate_substruct(fmt_entry, inline_sort)

                    # In some cases a sub-structure ends on an odd byte.
                    # Realign the search to an even byte since no boardcfg
                    # structure will start on an odd byte
                    if self.input_binary_class.position() & 0x1:
                        self.get_bytes('B')[0]

        # If there's one byte left...
        while self.input_binary_class.is_eof() is not True:
            self.get_bytes('B')[0]

        if self.output_binary_class:
            if self.input_binary_class.size() != self.output_binary_class.position():
                self.output_class.send_next_line(
                    'ERROR: Input and output binary file sizes do not match!')
                self.output_class.send_next_line('Exiting...')
                sys.exit(1)

        self.output_class.send_next_line('Validation complete.')

    def __init__(self):
        self.rules_file = os.path.join(self.dir_location, self.rules_file_name)
        return


class sysfw_binary_input_file:

    """ Input Class: Binary File
    """

    def __init__(self, fname=None):
        try:
            self.fh = io.open(
                fname,
                mode='rb')
        except Exception as ex:
            raise Exception(fname + ":  File not found?:" + str(ex))
        self.fsize = os.fstat(self.fh.fileno()).st_size
        return

    def reset_to_start(self):
        self.fh.seek(0)
        return True

    def position(self):
        return self.fh.tell()

    def size(self):
        return self.fsize

    def is_eof(self):
        return self.position() == self.fsize

    def get_bytes(self, num=1):
        b = self.fh.read(num)
        return b


class sysfw_binary_output_file:

    """ Output Class: Binary File
    """

    def __init__(self, fname=None):
        try:
            self.fh = io.open(
                fname,
                mode='wb')
        except Exception as ex:
            raise Exception(fname + ":  File not writable?:" + str(ex))
        return

    def position(self):
        return self.fh.tell()

    def send_bytes(self, b):
        self.fh.write(b)

    def delete(self):
        self.fh.close()
        os.remove(self.fh.name)


class sysfw_validation_output_console:

    """ Output Class: print validation details to console
    """

    def __init__(self, fname=None):
        return

    def send_next_line(self, s):
        print(s)


class sysfw_validation_output_file:

    """ Output Class: print validation details to file
    """

    def __init__(self, fname=None):
        try:
            self.fh = io.open(
                fname,
                mode='w',
                encoding="utf8",
                errors='ignore')
        except Exception as ex:
            raise Exception(fname + ":  File not writable?:" + str(ex))
        return

    def send_next_line(self, s):
        print(s, file=self.fh)


class sysfw_trace_cli:

    """ This is the Base Command Line interface Class
    """

    def __init__(self):
        self.rules = sysfw_boardcfg_rules()
        return

    def parse_args(self, args=None):
        """ Helper to parse the command line arguments
        """
        help_text = "System Firmware Board Configuration Validator\n"
        help_text = help_text + "URL: "
        help_text = help_text + \
            "http://software-dl.ti.com/tisci/esd/latest/3_boardcfg/BOARDCFG.html"

        import_help = textwrap.dedent('''
        This script can also be called as a python module with the same arguments
        as the CLI.

        from sysfw_boardcfg_validator import *

        sysfw_cli_job(args_arr=["-b", "sysfw_v2019.11.elf","-l","/tmp/validation.log", "-i","-o","sysfw_v2109.11.new.elf"]);
        ''')
        parser = argparse.ArgumentParser(prog=__file__,
                                         description=help_text,
                                         formatter_class=argparse.ArgumentDefaultsHelpFormatter,
                                         epilog=import_help)
        optional = parser._action_groups.pop()

        # Required input arguments
        input_group = parser.add_argument_group(
            'Required arguments - Define the inputs')

        input_group.add_argument(
            '-b',
            '--binary_file',
            help="Binary File provided as input",
            action="store",
            required=True)

        input_group.add_argument(
            '-s',
            '--soc',
            help="SoC supported by input binary",
            action="store",
            type=str,
            choices={'am6', 'am65x_sr2', 'am64x', 'j721e', 'j721e_legacy', 'j7200'},
            required=True)

        # Required output arguments
        og = parser.add_argument_group(
            'Required arguments - Choose one of the outputs')
        output_group = og.add_mutually_exclusive_group(required=True)

        output_group.add_argument(
            '-l',
            '--log_output_file',
            help="Validation log output file",
            action="store")

        output_group.add_argument(
            '-L',
            '--log_output_console',
            help="Report results to console",
            action="store_true")

        # Optional arguments
        optional.add_argument(
            '-o',
            '--output_binary_file',
            help="Binary output file.  Inline edit options of the binary board configuration data are output to this file",
            action="store")

        optional.add_argument(
            '-r',
            '--rules_file',
            help="Alternate Board configuration rules file",
            action="store",
            default=self.rules.get_rules_file())

        optional.add_argument(
            '-i',
            '--inline_sort',
            help="Perform an inline sort of the Resource Management board configuration resource assignments.  The -o option must be specified to output result of sort",
            action="store_true",
            default=False)

        parser._action_groups.append(optional)
        self.cmd_args = parser.parse_args(args)

        if self.cmd_args.rules_file is not None:
            self.rules.set_rules_file(self.cmd_args.rules_file)

        self.input_binary_class = sysfw_binary_input_file(
            self.cmd_args.binary_file)

        self.output_binary_class = None
        if self.cmd_args.output_binary_file is not None:
            self.output_binary_class = sysfw_binary_output_file(
                self.cmd_args.output_binary_file)

        if self.cmd_args.log_output_console is True:
            self.output_class = sysfw_validation_output_console()

        if self.cmd_args.log_output_file is not None:
            self.output_class = sysfw_validation_output_file(
                self.cmd_args.log_output_file)

    def process_data(self):
        self.rules.process_data(
            self.cmd_args.soc,
            self.input_binary_class,
            self.output_class,
            self.output_binary_class,
            self.cmd_args.inline_sort)


def sysfw_cli_job(args_arr=None):
    cli = sysfw_trace_cli()
    cli.parse_args(args=args_arr)
    cli.process_data()


def sysfw_cli_wrapper():
    """ If we make this a pypi package eventually, this would be the entry point
    """

    source_debug = 0

    if source_debug == 1:
        sysfw_cli_job()
    else:
        try:
            sysfw_cli_job()
        except Exception as e:
            print (str(e))
            sys.exit(1)


if __name__ == '__main__':
    sysfw_cli_wrapper()

# Format via !autopep8 -i -a %
# vim: et:ts=4
