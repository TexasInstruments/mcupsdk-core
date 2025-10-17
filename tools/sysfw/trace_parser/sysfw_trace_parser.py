#!/usr/bin/env python
#
# System Firmware Log Parse utility
# Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
#
# This software is licensed under the standard terms and conditions in the
# Texas Instruments Incorporated Technology and Software Publicly
# Available Software License Agreement, a copy of which is included in
# the software download.

import os
import io
import sys
import multiprocessing
import json
import re
import argparse
import logging
import string
import time
import textwrap

from builtins import chr
from builtins import range

_control_chars = ''.join(map(chr, list(range(0, 32)) + list(range(127, 160))))
_control_char_re = re.compile('[%s]' % re.escape(_control_chars))

# Python 3 Hack
try:
    xrange
    _range = xrange
except NameError:
    _range = range


def create_mask(highest_bit, lowest_bit):
    """ Return a bit  mask using a start and end bit
    """

    m = 1 << (highest_bit + 1)
    m = m - 1

    m_c = 0
    if lowest_bit > 0:
        m_c = 1 << lowest_bit
        m_c = m_c - 1

    return m & (~m_c)


def cleanup_string(s):
    """ A magical combination of regex and string squashing to get a clean output
    """
    s2 = ''.join([x for x in s if x in string.printable])
    s = _control_char_re.sub('', s2)
    return s.strip()


class sysfw_trace_rules:

    """ Class for processing the trace rules
    """
    dir_location = os.path.dirname(os.path.realpath(__file__))
    rules_file_name = 'sysfw_trace_rules.json'
    trace_version = None

    def get_rules_file(self):
        """ Return the current rules file
        """
        return self.rules_file

    def set_rules_file(self, fname):
        """ Set up a new rules file
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
        self.json_rules = json.load(self.fh)

        # XXX: Introduce schema check later?

        self.primary_match_re = re.compile(
            r"" + self.json_rules['description_trace']['detect_regex'])
        trace_struct = self.json_rules['description_trace']

        self.domain_mask = create_mask(
            trace_struct['domain']['higher_bit'],
            trace_struct['domain']['lower_bit'])
        self.domain_shift = trace_struct['domain']['lower_bit']

        self.action_mask = create_mask(
            trace_struct['action_id']['higher_bit'],
            trace_struct['action_id']['lower_bit'])
        self.action_shift = trace_struct['action_id']['lower_bit']

        self.msg_mask = create_mask(
            trace_struct['message_data']['higher_bit'],
            trace_struct['message_data']['lower_bit'])
        self.msg_shift = trace_struct['message_data']['lower_bit']

        self.sub_action_mask = create_mask(
            trace_struct['sub_action_id']['higher_bit'],
            trace_struct['sub_action_id']['lower_bit'])
        self.sub_action_shift = trace_struct['sub_action_id']['lower_bit']

        self.sub_action_msg_mask = create_mask(
            trace_struct['sub_action_message_data']['higher_bit'],
            trace_struct['sub_action_message_data']['lower_bit'])
        self.sub_action_msg_shift = trace_struct['sub_action_message_data']['lower_bit']

        self.version_domain = trace_struct['trace_data_version_action']['domain']
        self.version_action_id = trace_struct['trace_data_version_action']['action']

        self.decoder_ring = self.json_rules['message_decode']

        self.rules_file = fname
        return

    def decode_trace_data_version(self, in_str, log_start=False):
        x = int(in_str, 16)
        domain = (x & self.domain_mask) >> self.domain_shift
        action_id = (x & self.action_mask) >> self.action_shift
        message_data = (x & self.msg_mask) >> self.msg_shift

        if (("0x%02X" % domain == self.version_domain) and
                ("0x%02X" % action_id == self.version_action_id)):
            self.trace_version = message_data
            if log_start:
                s = u"Configuring trace data version to: 0x%05X" % self.trace_version
                self.output_class.send_next_line(s)
        else:
            if log_start:
                s = ("Trace data version not defined!  The trace version must be\n"
                     "be specified via the -Tv command line option if the first\n"
                     "matched trace in the provided log is not the\n"
                     "TRACE_DATA_VERSION action\n")
                raise Exception(s)

    def primary_decode_string(self, in_str):
        x = int(in_str, 16)
        domain = (x & self.domain_mask) >> self.domain_shift
        action_id = (x & self.action_mask) >> self.action_shift
        message_data = (x & self.msg_mask) >> self.msg_shift
        sub_action_id = (x & self.sub_action_mask) >> self.sub_action_shift
        sub_action_message_data = (
            x & self.sub_action_msg_mask) >> self.sub_action_msg_shift
        domain_decode = None
        try:
            domain_decode = self.decoder_ring["0x%02X" % domain]
        except Exception:
            s = "0x%08X: Unknown DOMAIN:0x%02X Action: 0x%02X MSG:0x%06X " % (
                x, domain, action_id, message_data)

        action_decode = None
        modifier_string = ""
        if domain_decode is not None:
            action_domain_name = domain_decode['action_domain_name']
            action_modifiers = None
            try:
                action_modifiers = domain_decode['action_modifiers']
            except BaseException:
                pass
            if action_modifiers is not None:
                for idx in _range(len(action_modifiers)):
                    modifier = action_modifiers[idx]
                    modifier_mask = int(modifier['mask'], 16)
                    s1 = ''
                    if (modifier_mask & action_id):
                        s1 = "%s(%s)" % (
                            modifier['modifier_short'],
                            modifier['modifier_long'])
                        modifier_string = modifier_string + "/" + s1
                        # Make sure to clear the action_id mask
                        action_id = action_id & (~modifier_mask)

            try:
                action_decode = domain_decode['actions']["0x%02X" % action_id]
            except Exception:
                s = "0x%08X: %10s%s: Unknown Action: 0x%02X MSG:0x%06X " % (
                    x, action_domain_name, modifier_string, action_id, message_data)

        if action_decode is not None:
            try:
                # Find closest version less than or equal to trace version
                version = max([int(v, 16) for v in action_decode['versions'].keys() if int(
                    v, 16) <= self.trace_version])
                action_decode = action_decode['versions']["0x%05X" % version]
            except BaseException:
                pass

            action_short = action_decode['action_short']
            action_long = action_decode['action_long']
            action_str = "%s(%s)" % (action_short, action_long)
            s = "0x%08X: %10s%s: %40s:" % (
                x,
                action_domain_name, modifier_string,
                action_str)

            if 'sub_actions' in action_decode:
                try:
                    action_decode = action_decode['sub_actions']["0x%02X" %
                                                                 sub_action_id]
                except Exception:
                    s = s + " Unknown Sub-Action: 0x%02X " % sub_action_id
                message_data = sub_action_message_data

            try:
                msg_data_decode = action_decode['msg_data']
                for idx in _range(len(msg_data_decode)):
                    msg = msg_data_decode[idx]
                    msk = create_mask(msg['higher_bit'], msg['lower_bit'])
                    v = (message_data & msk) >> msg['lower_bit']
                    s1 = msg['name'] + ': ' + msg['fmt'] % v
                    s = s + ' ' + s1
            except Exception:
                s = s + " MSG:0x%06X" % message_data

        return cleanup_string(s)

    def process_data(self, input_class, output_class,
                     match_only, input_file_mode=False):
        self.input_class = input_class
        self.output_class = output_class
        self.match_only = match_only

        # If we do have a file, we could do a prescan to try and find if we
        # can find a trace version information somewhere.
        if input_file_mode is True and self.trace_version is None:
            err = None
            while self.input_class.is_eof() is not True:
                in_str = self.input_class.get_next_line()
                try:
                    self.decode_trace_data_version(in_str, log_start=True)
                    err = None
                except Exception as er:
                    err = er
                    continue
                break
            if err is not None:
                raise(err)
            s = self.input_class.reset_to_start()
            if s is False:
                raise Exception(
                    'Unable to reset log to start after trace scan. Please use -Tv option')

        while self.input_class.is_eof() is not True:
            in_str = self.input_class.get_next_line()
            print_it = False

            pmatch = self.primary_match_re.match(in_str)

            decode_string = in_str

            if match_only is False:
                print_it = True

            if pmatch is not None:
                print_it = True

                # First matched value must contain the trace version
                if self.trace_version is None:
                    self.decode_trace_data_version(in_str, log_start=True)
                else:
                    self.decode_trace_data_version(in_str)

                decode_string = self.primary_decode_string(in_str)

            if print_it is True:
                self.output_class.send_next_line(decode_string)

    def __init__(self):
        self.rules_file = os.path.join(self.dir_location, self.rules_file_name)
        return


class sysfw_trace_input_file:

    """ Input Class: File
    """

    def __init__(self, fname=None):
        try:
            self.fh = io.open(
                fname,
                mode='r',
                encoding="utf8",
                errors='ignore')
        except Exception as ex:
            raise Exception(fname + ":  File not found?:" + str(ex))
        self.fsize = os.fstat(self.fh.fileno()).st_size
        return

    def reset_to_start(self):
        self.fh.seek(0)
        return True

    def is_eof(self):
        return self.fh.tell() == self.fsize

    def get_next_line(self):
        ln = self.fh.readline()
        return cleanup_string(ln)


class sysfw_trace_input_serial:

    """ Input Class: Serial port read
    """

    def __init__(self, fname=None):
        try:
            import serial
        except ImportError as err:
            raise Exception('Please install python package pyserial')

        self.ser = serial.Serial()
        self.ser.port = fname
        self.ser.baudrate = 115200
        self.ser.bytesize = serial.EIGHTBITS  # number of bits per bytes
        self.ser.parity = serial.PARITY_NONE  # set parity check: no parity
        self.ser.stopbits = serial.STOPBITS_ONE  # number of stop bits
        self.ser.timeout = None  # block read
        self.ser.xonxoff = False  # disable software flow control
        self.ser.rtscts = False  # disable hardware (RTS/CTS) flow control
        self.ser.dsrdtr = False  # disable hardware (DSR/DTR) flow control
        self.ser.writeTimeout = 2  # timeout for write

        try:
            self.ser.open()
        except Exception as e:
            raise Exception(
                "error open serial port(" + fname + "): " + str(e) + ': Try: `python -m serial.tools.list_ports -v`')

        # We use \r in SYSFW.. so, wrap it back up:
        self.ser.flushInput()
        self.ser.flushOutput()
        self.ser_io = io.TextIOWrapper(
            io.BufferedRWPair(self.ser, self.ser, 1),
            newline='\n',
            line_buffering=True, encoding="ascii", errors='ignore')
        return

    def reset_to_start(self):
        return False

    def is_eof(self):
        # Assume always open unless otherwise..
        return False

    def get_next_line(self):
        ln = self.ser_io.readline()
        return cleanup_string(ln)


class sysfw_trace_output_console:

    """ Input Class: print to console
    """

    def __init__(self, fname=None, ts=None):
        self.ts = ts
        return

    def send_next_line(self, s):
        if self.ts is not None:
            s = self.ts.get_ts() + s
        print(s)


class sysfw_trace_output_file:

    """ Input Class: print to console
    """

    def __init__(self, fname=None, ts=None):
        try:
            self.fh = io.open(
                fname,
                mode='w',
                encoding="utf8",
                errors='ignore')
        except Exception as ex:
            raise Exception(fname + ":  File not writable?:" + str(ex))
        self.ts = ts
        return

    def send_next_line(self, s):
        if self.ts is not None:
            s = self.ts.get_ts() + s
        self.fh.write(s + '\n')


class ts:

    def _get_time_ms(self):
        return int(round(time.time() * 1000))

    def get_ts(self):
        self.c_time = self._get_time_ms()
        return "%10s: " % str(self.c_time - self.epoch)

    def reset_ts(self):
        self.epoch = self._get_time_ms()

    def __init__(self):
        self.epoch = self._get_time_ms()


class sysfw_trace_cli:

    """ This is the Base Command Line interface Class
    """

    def __init__(self):
        self.rules = sysfw_trace_rules()
        self.ts = ts()
        return

    def parse_args(self, args=None):
        """ Helper to parse the command line arguments
        """
        help_text = "System Firmware Log Parse utility\n"
        help_text = help_text + "URL: "
        help_text = help_text + \
            "http://software-dl.ti.com/tisci/esd/latest/4_trace/trace.html"

        import_help = textwrap.dedent('''
        This script can also be called as a python module with the same arguments
        as the CLI.

        from sysfw_trace_parser import *

        sysfw_cli_job(args_arr=["-l", "sysfw_v2019.03_fail.log","-o","/tmp/out.log","-Tv","0x00000"]);
        ''')
        parser = argparse.ArgumentParser(prog=__file__,
                                         description=help_text,
                                         formatter_class=argparse.ArgumentDefaultsHelpFormatter,
                                         epilog=import_help)
        optional = parser._action_groups.pop()
        ig = parser.add_argument_group(
            'Required arguments - Choose one of the inputs')
        input_group = ig.add_mutually_exclusive_group(required=True)

        input_group.add_argument(
            '-l',
            '--log_file',
            help="Log File provided as input",
            action="store")
        input_group.add_argument(
            '-d',
            '--serial_port',
            help="Provide Device as input: " +
            "Requires pyserial package installed: " +
            "See https://pyserial.readthedocs.io/",
            action="store")

        og = parser.add_argument_group(
            'Required arguments - Choose one of the outputs')
        output_group = og.add_mutually_exclusive_group(required=True)

        output_group.add_argument(
            '-o',
            '--output_file',
            help="Parse out the output to a file",
            action="store")

        output_group.add_argument(
            '-O',
            '--output_console',
            help="Log File to parse and report results to console",
            action="store_true")

        optional.add_argument(
            '-Tv',
            '--trace_data_version',
            help="Trace data version input in form \"0xYYZZZ\" where YY is the major and ZZZ is the minor version.  A trace data version found in the log will override this input",
            action="store")

        optional.add_argument(
            '-t',
            '--time_stamp_relative',
            help="Add TimeStamp to output in relative milliseconds(this is approximation ONLY)",
            action="store_true")

        optional.add_argument(
            '-r',
            '--rules_file',
            help="Alternate Rules file",
            action="store",
            default=self.rules.get_rules_file())

        optional.add_argument(
            '-Pm',
            '--print_match_only',
            help="Print just decoded data,",
            action="store_true",
            default=False)

        parser._action_groups.append(optional)
        self.cmd_args = parser.parse_args(args)

        if self.cmd_args.rules_file is not None:
            self.rules.set_rules_file(self.cmd_args.rules_file)

        self.input_file_mode = False
        if self.cmd_args.log_file is not None:
            self.input_class = sysfw_trace_input_file(self.cmd_args.log_file)
            self.input_file_mode = True

        if self.cmd_args.serial_port is not None:
            self.input_class = sysfw_trace_input_serial(
                self.cmd_args.serial_port)

        timestamp = None
        if self.cmd_args.time_stamp_relative is True:
            timestamp = self.ts
        if self.cmd_args.output_console is True:
            self.output_class = sysfw_trace_output_console(ts=timestamp)

        if self.cmd_args.output_file is not None:
            self.output_class = sysfw_trace_output_file(
                self.cmd_args.output_file, ts=timestamp)

        if self.cmd_args.trace_data_version is not None:
            self.rules.trace_version = int(
                self.cmd_args.trace_data_version, 16)
            s = u"Configuring trace data version to: 0x%05X" % self.rules.trace_version
            self.output_class.send_next_line(s)

    def process_data(self):
        self.ts.reset_ts()
        self.rules.process_data(
            self.input_class,
            self.output_class,
            self.cmd_args.print_match_only,
            self.input_file_mode)


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
