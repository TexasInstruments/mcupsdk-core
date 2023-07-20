import os
import sys
import time
import shlex
import struct
import argparse
import subprocess

BOOTLOADER_UNIFLASH_BUF_SIZE                         = 1024*512 # 512KB This has to be a 256 KB aligned value, because flash writes will be block oriented
BOOTLOADER_UNIFLASH_HEADER_SIZE                      = 32 # 32 B

BOOTLOADER_UNIFLASH_FILE_HEADER_MAGIC_NUMBER         = 0x46554C42 # BLUF
BOOTLOADER_UNIFLASH_RESP_HEADER_MAGIC_NUMBER         = 0x52554C42 # BLUR

BOOTLOADER_UNIFLASH_OPTYPE_FLASH                     = 0xF0
BOOTLOADER_UNIFLASH_OPTYPE_FLASH_VERIFY              = 0xF1
BOOTLOADER_UNIFLASH_OPTYPE_FLASH_XIP                 = 0xF2
BOOTLOADER_UNIFLASH_OPTYPE_FLASH_VERIFY_XIP          = 0xF3
BOOTLOADER_UNIFLASH_OPTYPE_FLASH_TUNING_DATA         = 0xF4
BOOTLOADER_UNIFLASH_OPTYPE_FLASH_ERASE               = 0xFE
BOOTLOADER_UNIFLASH_OPTYPE_EMMC_FLASH                = 0xF5
BOOTLOADER_UNIFLASH_OPTYPE_EMMC_VERIFY               = 0xF6

optypewords = {
    "flash" : BOOTLOADER_UNIFLASH_OPTYPE_FLASH,
    "flashverify" : BOOTLOADER_UNIFLASH_OPTYPE_FLASH_VERIFY,
    "flash-xip" : BOOTLOADER_UNIFLASH_OPTYPE_FLASH_XIP,
    "flashverify-xip" : BOOTLOADER_UNIFLASH_OPTYPE_FLASH_VERIFY_XIP,
    "flash-phy-tuning-data" : BOOTLOADER_UNIFLASH_OPTYPE_FLASH_TUNING_DATA,
    "erase" : BOOTLOADER_UNIFLASH_OPTYPE_FLASH_ERASE,
    "flash-emmc":BOOTLOADER_UNIFLASH_OPTYPE_EMMC_FLASH,
    "flashverify-emmc":BOOTLOADER_UNIFLASH_OPTYPE_EMMC_VERIFY,
}

TMP_SUFFIX = ".tmp"

g_script_description = '''
MCU+SDK utility script to flash images to device flash.
'''

g_cfg_file_description = '''
Configuration file which has a list of files with
options for flashing. This should be given as option=value pairs
separated by space just like in command line. Only give the long
options in the config file i.e --flash-offset=0x80000 and not -o 0x80000

Example, below can be a line in the config file, myconfig.cfg
--file=am64x-evm/sbl_ospi.release.tiimage --operation=flash --flash-offset=0x0

and then the script can be called:
python usb_dfu_uniflash.py -p <COM port> --cfg=myconfig.cfg
'''

def get_numword(s):
    numword = None
    if(s.startswith('0x')):
        numword = int(s, 16)
    else:
        numword = int(s, 10)
    return numword

# Add the file header with metadata to the file to be sent. Expects a validated linecfg
def create_temp_file(linecfg):
    '''
    File header struct used in the target side :

    typedef struct Bootloader_UniflashFileHeader_s
    {
        uint32_t magicNumber;
        /* BOOTLOADER_UNIFLASH_FILE_HEADER_MAGIC_NUMBER */

        uint32_t operationTypeAndFlags;
        /* LSByte - Operation Type:flash, verify flash or erase */

        uint32_t offset;
        /* Offset to flash, verify flash or erase flash */

        uint32_t eraseSize;
        /* Size of flash to erase */

        uint32_t actualFileSize;
        /* Size of the file sent. This is needed because xmodem returns a padded file size */

        uint32_t rsv1;
        uint32_t rsv2;
        uint32_t rsv3;
        /* Reserved */
    } Bootloader_UniflashFileHeader;

    So we need to use same struct to pack in python
    '''
    file_header_str = '<LLLLLLLL'

    if(linecfg.optype in ("erase", "flash-phy-tuning-data")):
        # No separate file required
        tempfilename = "{}_command".format(linecfg.optype)
    else:
        tempfilename = linecfg.filename + TMP_SUFFIX

    f = open(tempfilename, "wb")

    # Construct the header now, first define the reserved word
    rsv_word = 0xDEADBABE

    # Determine the offset if applicable
    offset_val = rsv_word
    if(linecfg.optype not in ("flash-xip", "flashverify-xip", "flash-phy-tuning-data")):
        offset_val = get_numword(linecfg.offset)

    # Determine the erase size if applicable
    erase_size_val = rsv_word
    if(linecfg.optype in ("erase")):
        erase_size_val = get_numword(linecfg.erase_size)

    # Determine the actual file size if applicable, no original file in case of erase or phy tuning
    actual_file_size = 0
    if(linecfg.optype not in ("erase","flash-phy-tuning-data")):
        actual_file_size = os.path.getsize(linecfg.filename)

    file_header = struct.pack(file_header_str,
                              BOOTLOADER_UNIFLASH_FILE_HEADER_MAGIC_NUMBER,
                              optypewords[linecfg.optype],
                              offset_val,
                              erase_size_val,
                              actual_file_size,
                              rsv_word, rsv_word, rsv_word
                             )

    # Write header to file
    f.write(file_header)

    # No original file in case of erase or phy tuning
    if(linecfg.optype not in ("erase","flash-phy-tuning-data")):
        # copy the original file to this file
        original_file = open(linecfg.filename, "rb")
        f.write(original_file.read())
        original_file.close()

    f.close()

    return tempfilename

# Function to detect dfu device enumeration
def wait_for_enumeration():

    enum_done = False
    ls_dfu = "dfu-util -l"
    # To print banner that waiting for DFU enumeration 
    flag = False
    while enum_done == False:
        subprocess.run(ls_dfu + " > temp_file",shell=True,stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT)
        temp_file = open('temp_file')
        string = temp_file.read()

        if ("Found DFU" in string and "UNKNOWN" not in string ):
            enum_done = True
        else:
            if flag == False: 
                print("------------------------------------------------------")
                print("Waiting for DFU device to be enumerated ....")
                print("------------------------------------------------------")
                # set flag so this will be printer only once 
                flag = True 
            # If not detected then wait sometime before retrying
            time.sleep(0.5)

    temp_file.close()
    os.remove('temp_file')

def dfu_fw_send(filename,intf=0,alt=0,xfer_size=512,isFlashWriter=False):
    status = False
    timetaken = 0
    if not os.path.exists(filename):
        print('[ERROR] File [' + filename + '] not found !!!')
        sys.exit()

    # Wait for sbl flash writer to be detected before starting transaction with SBL.
    wait_for_enumeration()

    # compile command for linux and windows separately
    print("----------------------------------------------------------------------------")
    print("Executing DFU command with alt_setting={0} interface={1} transfer_size={2}".format(alt,intf,xfer_size))
    print("----------------------------------------------------------------------------")
    cmd = "dfu-util -a {0} -i {1} -t {2} -D {3}".format(alt,intf,xfer_size,filename)
    try:
        tstart = time.time()
        dfu_status = os.system(cmd)
        tstop = time.time()
        timetaken = round(tstop-tstart,2)
        if(dfu_status != 0):
            status = False
        else:
            status = True
    except:
        status = False

    if( status is False ) :
        print("")
        print ("[ERROR] DFU fw  send failed, no response OR incorrect response from EVM OR cancelled by user,")
        print ("Power cycle EVM and run this script again !!!")
        sys.exit(1)


    return status,timetaken

def send_file_by_parts(l_cfg):
    orig_f_name = l_cfg.filename
    orig_offset = l_cfg.offset

    alt_setting = l_cfg.alt_setting
    intf_num = l_cfg.intf_num
    xfer_size = l_cfg.xfer_size

    f = open(orig_f_name, "rb")
    f_bytes = f.read()
    f.close()

    num_parts   = int(len(f_bytes) / BOOTLOADER_UNIFLASH_BUF_SIZE)
    remain_size = len(f_bytes) % BOOTLOADER_UNIFLASH_BUF_SIZE
    total_time_taken = 0


    for i in range(0, num_parts):

        start = i*BOOTLOADER_UNIFLASH_BUF_SIZE
        end = start+BOOTLOADER_UNIFLASH_BUF_SIZE

        part_data = f_bytes[start:end]
        part_filename = orig_f_name + ".part{}".format(i+1)
        # make the partial file
        f = open(part_filename, "wb")
        f.write(part_data)
        f.close()

        # temporarily change this to the partial filename
        l_cfg.filename = part_filename
        l_cfg.offset = hex(get_numword(orig_offset) + i*BOOTLOADER_UNIFLASH_BUF_SIZE)

        # send the partial file normally
        tempfilename = create_temp_file(l_cfg)

        status, timetaken = dfu_fw_send(tempfilename,intf_num,alt_setting,xfer_size)

        print("Time Elapsed = {}".format(timetaken))
        total_time_taken += timetaken


        # delete the temporary file
        os.remove(part_filename)
        os.remove(tempfilename)

    # Send the last part, if there were residual bytes
    if(remain_size > 0):
        start = num_parts*BOOTLOADER_UNIFLASH_BUF_SIZE
        # Read till the end of original file
        part_data = bytearray(f_bytes[start:])
        padding = 256 - (remain_size % 256) # page size adjustment
        # there should be a better way :)
        for i in range(0, padding):
            part_data.append(0)
        part_filename = orig_f_name + ".part{}".format(num_parts+1)

        # make the partial file
        f = open(part_filename, "wb")
        f.write(part_data)
        f.close()

        # temporarily change this to the partial filename
        l_cfg.filename = part_filename
        l_cfg.offset = hex(get_numword(orig_offset) + num_parts*BOOTLOADER_UNIFLASH_BUF_SIZE)

        # send the partial file normally
        tempfilename = create_temp_file(l_cfg)
        status, timetaken = dfu_fw_send(tempfilename,intf_num,alt_setting,xfer_size)
        print("Time Elapsed = {}".format(timetaken))
        total_time_taken += timetaken

        # delete the temporary file
        os.remove(part_filename)
        os.remove(tempfilename)

    # revert back the original filename and offset
    l_cfg.filename = orig_f_name
    l_cfg.offset = orig_offset

    return status, total_time_taken

def main(argv):

    config_file = None
    cmd_flash_writer_found = False
    ops_invalid = False

    cmdlinecfg = LineCfg(cfg_src="cmd")

    my_parser = argparse.ArgumentParser(description=g_script_description)
    my_parser.add_argument('-a', '--alt', required=False,type=int, help="The alt setting used for DFU transfer. An alt setting number defines the target memory on SOC. Default value = 0 ")

    my_parser.add_argument('-i', '--interface', required=False,type=int, help=" Selects the interface number used for DFU. Default value = 0 ")
    my_parser.add_argument('-t', '--transfer-size', required=False,type=int, help="Defines the number of bytes that will be transfered per setup transaction. Default transfer_size = 512 Bytes ")
    my_parser.add_argument('-f', '--file', required=False, help="Filename to send for an operation. Not required if using config mode (--cfg)")
    my_parser.add_argument('-o', '--flash-offset', required=False, help="Offset (in hexadecimal format starting with a 0x) at which the flash/verify flash is to be done. Not required if using config mode (--cfg)")
    my_parser.add_argument('--operation', required=False, help='Operation to be done on the file => "flash" or "flashverify" or "erase" or "flash-xip" or "flashverify-xip" or "flash-phy-tuning-data" or "flash-emmc" or "flashverify-emmc". Not required if using config mode (--cfg)')
    my_parser.add_argument('--flash-writer', required=False, help="Special option. This will load the sbl_dfu_uniflash binary which will be booted by ROM. Other arguments are irrelevant and hence ignored when --flash-writer argument is present. Not required if using config mode (--cfg)")
    my_parser.add_argument('--erase-size', required=False, help='Size of flash to erase. Only valid when operation is "erase"')
    my_parser.add_argument('--cfg', required=False, help=g_cfg_file_description)

    args = my_parser.parse_args()

    config_file = args.cfg

    if args.alt is not None:
        cmdlinecfg.alt_setting = args.alt

    if args.interface is not None:
        cmdlinecfg.intf_num = args.interface

    if args.transfer_size is not None:
        cmdlinecfg.xfer_size = args.transfer_size

    cmdlinecfg.filename = args.file
    cmdlinecfg.offset = args.flash_offset
    cmdlinecfg.optype = args.operation
    cmdlinecfg.flashwriter = args.flash_writer
    cmdlinecfg.erase_size = args.erase_size

    if(config_file != None):
        # Check if file exists
        if not os.path.exists(config_file):
            print('[ERROR] Configuration file [' + config_file + '] not found !!!')
            sys.exit()

        print("")
        print("Parsing config file ...")
        filecfg = FileCfg(config_file)
        parse_status = filecfg.parse()

        if(parse_status != 0):
            print(parse_status)
            sys.exit()
        else:
            # No errors, can proceed to flash
            print("Parsing config file ... SUCCESS. Found {} command(s) !!!".format(len(filecfg.cfgs)))
            print("")

            alt_setting = filecfg.alt_setting
            intf_num = filecfg.intf_num
            xfer_size = filecfg.xfer_size

            #if booting flash writer then no temp file required
            if(filecfg.flash_writer_index != None):
                # Found flash writer, flash it
                cfg_flash_writer_file = filecfg.cfgs[filecfg.flash_writer_index].flashwriter
                print("Executing command {} of {} ...".format(1, len(filecfg.cfgs)))
                print("Found flash writer ... sending {}".format(cfg_flash_writer_file))

                # send flash writer via DFU
                status, timetaken = dfu_fw_send(cfg_flash_writer_file,intf_num,alt_setting, xfer_size,isFlashWriter = True)

                bandwidth = (os.path.getsize(cfg_flash_writer_file)/1024)/timetaken
                print("Sent flashwriter {} of size {} bytes in {}s. Bandwidth = {:0.2f}kbps".format(cfg_flash_writer_file, os.path.getsize(cfg_flash_writer_file), timetaken,bandwidth))
                print("")

            # loop through cfgs and lines, skip the process for flashwriter
            for i in range(0, len(filecfg.cfgs)):
                if(i != filecfg.flash_writer_index):
                    line = filecfg.lines[i]
                    linecfg = filecfg.cfgs[i]
                    print("Executing command {} of {} ...".format(i+1, len(filecfg.cfgs)))
                    print("Command arguments : {}".format(line.rstrip('\n')))
                    # Check if the size of application image is larger than buffer size in target side.
                    f_size = 0
                    tempfilename = ""
                    if linecfg.filename is not None:
                        f_size = os.path.getsize(linecfg.filename)

                    if((f_size + BOOTLOADER_UNIFLASH_HEADER_SIZE >= BOOTLOADER_UNIFLASH_BUF_SIZE) and (linecfg.optype in ["flash", "flashverify"])):
                        # Send by parts
                        status, timetaken = send_file_by_parts(linecfg)
                    else:
                        # Send normally
                        tempfilename = create_temp_file(linecfg)

                        status, timetaken = dfu_fw_send(tempfilename,linecfg.intf_num,linecfg.alt_setting,linecfg.xfer_size)

                    orig_filename = linecfg.filename
                    if(linecfg.optype == "erase"):
                        print("Sent flash erase command.")
                    elif(linecfg.optype == "flash-phy-tuning-data"):
                        print("Sent flash phy tuning data in {}s.".format(timetaken))
                    else:
                        bandwidth = (os.path.getsize(orig_filename)/1024)/timetaken
                        print("Sent {} of size {} bytes in {}s. Bandwidth = {:0.2f}kbps".format(orig_filename, os.path.getsize(orig_filename), timetaken,bandwidth))
                    print(status)
                    # Delete the tempfile if it exists
                    if(os.path.exists(tempfilename)):
                        os.remove(tempfilename)

            print("All commands from config file are executed !!!")

    else:
        # Validate the cmdline config
        status = cmdlinecfg.validate()
        if(status != 0):
            print(status)
            if(cmdlinecfg.exit_now == True):
                sys.exit(2)

        # get dfu arguments
        alt_setting = cmdlinecfg.alt_setting
        intf_num = cmdlinecfg.intf_num
        xfer_size = cmdlinecfg.xfer_size

        # If flash writer was found, send it first
        if(cmdlinecfg.found_flashwriter_cmd == True):
            print("Found flash writer ... sending {}".format(cmdlinecfg.flashwriter))
            status, timetaken = dfu_fw_send(cmdlinecfg.flashwriter,intf_num, alt_setting,xfer_size,isFlashWriter=True)
            print("Sent flashwriter {} of size {} bytes in {}s.".format(cmdlinecfg.flashwriter, os.path.getsize(cmdlinecfg.flashwriter), timetaken))
            print("")

        if(cmdlinecfg.ops_invalid == False):
            # Check if the size of application image is larger than buffer size in target side.
            f_size = 0
            if cmdlinecfg.filename is not None:
                f_size = os.path.getsize(cmdlinecfg.filename)

            if((f_size + BOOTLOADER_UNIFLASH_HEADER_SIZE >= BOOTLOADER_UNIFLASH_BUF_SIZE) and (cmdlinecfg.optype in ["flash", "flashverify"])):
                # Send by parts
                status, timetaken = send_file_by_parts(cmdlinecfg )
            else:
                # Send normally
                tempfilename = create_temp_file(cmdlinecfg)
                status, timetaken = dfu_fw_send(tempfilename,intf_num, alt_setting,xfer_size)
                # Delete the tempfile if it exists
                if(os.path.exists(tempfilename)):
                    os.remove(tempfilename)

            orig_filename = cmdlinecfg.filename
            if(cmdlinecfg.optype == "erase"):
                print("Sent flash erase command.")
            elif(cmdlinecfg.optype == "flash-phy-tuning-data"):
                print("Sent flash phy tuning data in {}s.".format(timetaken))
            else:
                bandwidth = (os.path.getsize(orig_filename)/1024)/timetaken
                print("Sent {} of size {} bytes in {}s. Bandwidth = {:0.2f}kbps".format(orig_filename, os.path.getsize(orig_filename), timetaken,bandwidth))
            print(status)

# Class definitions used
class LineCfg():
    def __init__(self, line=None, filename=None, optype=None, offset=None, erase_size=None, flashwriter=None, cfg_src="cfg",alt_setting=0,intf_num=0,xfer_size=512):
        self.line = line
        self.filename = filename
        self.optype = optype
        self.offset = offset
        self.flashwriter = flashwriter
        self.erase_size = erase_size
        self.found_flashwriter_cmd = False
        self.ops_invalid = False
        self.cfg_src = cfg_src
        self.exit_now = False
        self.alt_setting = alt_setting
        self.intf_num = intf_num
        self.xfer_size = xfer_size
        self.found_dfu_cmd = False

    # Takes a string of comma separated key=value pairs and parses into a dictionary
    def parse_to_dict(self, config_string):
        config_dict = dict()
        splitter = shlex.shlex(config_string, posix=True)
        splitter.commenters="#"
        splitter.whitespace = ' '
        splitter.whitespace_split = True

        for key_value_pair in splitter:
            kv = key_value_pair.strip()
            if not kv:
                continue
            kv_t = kv.split('=', 1)
            if(len(kv_t)==1):
                #error, no value
                pass
            else:
                config_dict[kv_t[0]] = kv_t[1]

        return config_dict

    def validate(self):
        status = 0
        optypes = list(optypewords.keys())
        # Check if called by config file parser or direct command line
        if(self.line!=None and self.cfg_src=="cfg"):
            # Called from config_file
            config_dict = self.parse_to_dict(self.line)

            if not config_dict:
                status = "invalid_line"
            else:
                # check for flashwriter
                if "--flash-writer" not in config_dict.keys():
                    # it is okay not to have a flashwriter, in this case
                    # user is expected to have flashed the flashwriter upfront
                    # Check for other arguments
                    # check if file is present
                    if "--transfer-size" in config_dict.keys():
                        self.xfer_size = int(config_dict["--transfer-size"])
                        if((self.xfer_size % 64 != 0) or (self.xfer_size > 512)):
                            status = "[ERROR] Invalid transfer size. It should be multiple of 64 and less than 512b !!!"
                            return status
                        else:
                            self.xfer_size = int(config_dict["--transfer-size"])
                            self.found_dfu_cmd = True
                            status = 0
                            return status

                    if "--alt" in config_dict.keys():
                        self.alt_setting = int(config_dict["--alt"])
                        self.found_dfu_cmd = True
                        status = 0
                        return status

                    if "--interface" in config_dict.keys():
                        self.intf_num = int(config_dict["--interface"])
                        self.found_dfu_cmd = True
                        status = 0
                        return status

                    if "--operation" not in config_dict.keys():
                        status = "[ERROR] No file operation specified !!!"
                        return status
                    else:
                        self.optype = config_dict["--operation"]

                    if(self.optype == "flash" or self.optype == "flashverify" or self.optype == "erase" or self.optype == "flash-emmc" or\
                        self.optype == "flashverify-emmc"):
                        if "--flash-offset" not in config_dict.keys():
                            status = "[ERROR] Operation selected was {}, but no offset provided !!!".format(self.optype)
                            return status
                        else:
                            self.offset = config_dict["--flash-offset"]

                    if(self.optype == "flash" or self.optype == "flashverify" or self.optype == "flash-xip" or self.optype == "flashverify-xip" or\
                        self.optype == "flash-emmc" or self.optype == "flashverify-emmc"):
                        if "--file" not in config_dict.keys():
                            status = "[ERROR] Operation selected was {}, but no filename provided !!!".format(self.optype)
                            return status
                        else:
                            self.filename = config_dict["--file"]

                    if(self.optype == "erase"):
                        if "--erase-size" not in config_dict.keys():
                            status = "[ERROR] Operation selected was {}, but no erase size provided !!!".format(self.optype)
                            return status
                        else:
                            self.erase_size = config_dict["--erase-size"]

                    #No errors in parsing, now validate params to the extent possible

                    if(self.optype not in ("erase", "flash-phy-tuning-data")):
                        try:
                            f = open(self.filename)
                        except FileNotFoundError:
                            status = "[ERROR] File not found !!!"
                            return status

                    if(self.optype not in optypes):
                        status = "[ERROR] Invalid File Operation type !!!"
                        return status

                else:
                    # we have a flash writer argument, other arguments are moot
                    self.flashwriter = config_dict["--flash-writer"]

                    try:
                        f = open(self.flashwriter)
                    except FileNotFoundError:
                        status = "[ERROR] Flashwriter file not found !!!"
                        return status

        elif(self.line==None and self.cfg_src=="cmd"):
            # Called from main, with cmd line arguments
            # We have the filename, optype etc already filled
            if(self.flashwriter != None):
                self.found_flashwriter_cmd = True
            else:
                self.found_flashwriter_cmd = False

            if((self.xfer_size % 64 != 0) or (self.xfer_size > 512)):
                # Invalid argument user should exit
                self.exit_now = True
                status = "[ERROR] Invalid transfer size. It should be multiple of 64 and less than or equal to 512 !!!"
                return status

            if(self.optype == None):
                # mandatory arg. If no flashwriter, caller should exit now
                self.ops_invalid = True
                self.exit_now = not self.found_flashwriter_cmd
                if(self.found_flashwriter_cmd == False):
                    status = "[ERROR] No operation type mentioned !!!"
                    return status
                else:
                    pass
            else:
                if((self.optype == "flash" or self.optype == "flashverify" or self.optype == "erase" or self.optype == "flashverify-emmc") and (self.offset == None)):
                    self.ops_invalid = True
                    self.exit_now = not self.found_flashwriter_cmd
                    # flash/verify flash/erase, but no offset given. exit with help if no flashwriter
                    if(self.found_flashwriter_cmd == False):
                        status = "[ERROR] Operation selected was {}, but no offset was provided !!!".format(self.optype)
                        return status
                    else:
                        pass

                if((self.optype == "flash" or self.optype == "flashverify" or self.optype == "flash-xip" or self.optype == "flashverify-xip" or self.optype == "flashverify-emmc") and (self.filename == None)):
                    self.ops_invalid = True
                    self.exit_now = not self.found_flashwriter_cmd
                    # flash/verify flash/erase, but no filename given. exit with help if no flashwriter
                    if(self.found_flashwriter_cmd == False):
                        status = "[ERROR] Operation selected was {}, but no filename was provided !!!".format(self.optype)
                        return status
                    else:
                        pass

                if(self.optype == "erase" and self.erase_size == None):
                    self.ops_invalid = True
                    self.exit_now = not self.found_flashwriter_cmd
                    # erase command found, but no erase size given. exit with help if no flashwriter
                    if(self.found_flashwriter_cmd == False):
                        status = "[ERROR] Operation selected was {}, but no erase size was provided !!!".format(self.optype)
                        return status
                    else:
                        pass

                if(self.optype not in optypes):
                    self.ops_invalid = True
                    self.exit_now = not self.found_flashwriter_cmd
                    # erase command found, but no erase size given. exit with help if no flashwriter
                    if(self.found_flashwriter_cmd == False):
                        status = "[ERROR] Invalid File Operation type !!!"
                        return status
                    else:
                        pass
                if(self.optype not in ("erase", "flash-phy-tuning-data")):
                    try:
                        f = open(self.filename)
                    except FileNotFoundError:
                        self.ops_invalid = True
                        self.exit_now = not self.found_flashwriter_cmd
                        # erase command found, but no erase size given. exit with help if no flashwriter
                        if(self.found_flashwriter_cmd == False):
                            status = "[ERROR] File not found !!!"
                            return status
                        else:
                            pass
        else:
            # Should not hit this
            status = -1
        return status

class FileCfg():
    def __init__(self, filename=None):
        self.filename = filename
        self.lines = list()
        self.cfgs = list()
        self.errors = list()
        self.flash_writer_index = None
        self.xfer_size = 512
        self.alt_setting = 0
        self.intf_num = 0

    def parse(self):
        f = open(self.filename, "r")
        lines = f.readlines()
        f.close()
        parse_status = 0
        linecount = 0
        found_fw = False
        valid_cfg_count = 0

        for line in lines:
            linecfg = LineCfg(line=line)
            status = linecfg.validate()

            if(status != 0 and status != "invalid_line"):
                self.errors.append("[ERROR] Parsing error found on line {} of {}\n{}\n".format(linecount+1, self.filename, status))
            else:
                if(status != "invalid_line"):
                    if(linecfg.flashwriter != None):
                        if(found_fw == True):
                            # error, another instance of flashwriter found
                            flash_writer_error_msg = "[ERROR] Redefinition of flash writer !!!"
                            self.errors.append("[ERROR] Parsing error found on line {} of {}\n{}\n".format(linecount+1, self.filename, flash_writer_error_msg))
                        else:
                            found_fw = True
                            self.cfgs.append(linecfg)
                            self.lines.append(line.rstrip('\n'))
                            self.flash_writer_index = valid_cfg_count
                            valid_cfg_count += 1

                    elif ( linecfg.found_dfu_cmd == True ):
                        # check what command it is
                        if(linecfg.xfer_size != 512):
                            self.xfer_size = linecfg.xfer_size

                        if(linecfg.alt_setting != 0 ):
                            self.alt_setting = linecfg.alt_setting

                        if(linecfg.intf_num != 0 ):
                            self.intf_num = linecfg.intf_num

                        self.lines.append(line.rstrip('\n'))
                        valid_cfg_count += 1
                    else:
                        self.cfgs.append(linecfg)
                        self.lines.append(line.rstrip('\n'))
                        valid_cfg_count += 1
            linecount += 1

        if(len(self.errors) > 0):
            # Some commands have errors
            parse_status = "Parsing config file ... ERROR. {} error(s).\n\n".format(len(self.errors)) + "\n".join(self.errors)
        else:
            # if no error found then add dfu transfer config found in every line
            for linecfg in self.cfgs:
                linecfg.alt_setting = self.alt_setting
                linecfg.intf_num = self.intf_num
                linecfg.xfer_size = self.xfer_size

        return parse_status

if __name__ == "__main__":

   main(sys.argv[1:])
