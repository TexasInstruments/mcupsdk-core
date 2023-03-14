import os
import sys
import time
import shlex
import struct
import argparse
import getopt

try:
    from tqdm import tqdm
    from pcan.PCANBasic import *
    import ctypes
except ImportError:
    print('[ERROR] Dependant modules not available of PCANBasic.')
    print('')
    sys.exit()

m_objPCANBasic = None
m_DLLFound = False
IsFD = True    # Set it to False if std CAN Protocol to be used.
gSer = None

PcanHandle = PCAN_USBBUS1

BOOTLOADER_CAN_STATUS_LOAD_SUCCESS            = "43 43 55 53"
BOOTLOADER_CAN_STATUS_LOAD_FAIL               = "4C 49 41 46"
BOOTLOADER_CAN_STATUS_APPIMAGE_SIZE_EXCEEDED  = "44 43 58 45"
CAN_PONG = "50 4F 4E 47"
CAN_LST_MSG = "4C 53 54 4D 53 47 41 4B"
CAN_MSG_ACK = "4D 53 47 41 43 4B"

BOOTLOADER_UNIFLASH_BUF_SIZE                         = 1024*1024 # 1 MB This has to be a 256 KB aligned value, because flash writes will be block oriented
BOOTLOADER_UNIFLASH_HEADER_SIZE                      = 32 # 32 B

BOOTLOADER_UNIFLASH_FILE_HEADER_MAGIC_NUMBER         = 0x46554C42 # BLUF
BOOTLOADER_UNIFLASH_RESP_HEADER_MAGIC_NUMBER         = "42 4C 55 52" # BLUR

BOOTLOADER_UNIFLASH_OPTYPE_FLASH                     = 0xF0
BOOTLOADER_UNIFLASH_OPTYPE_FLASH_VERIFY              = 0xF1
BOOTLOADER_UNIFLASH_OPTYPE_FLASH_XIP                 = 0xF2
BOOTLOADER_UNIFLASH_OPTYPE_FLASH_VERIFY_XIP          = 0xF3
BOOTLOADER_UNIFLASH_OPTYPE_FLASH_TUNING_DATA         = 0xF4
BOOTLOADER_UNIFLASH_OPTYPE_FLASH_ERASE               = 0xFE
BOOTLOADER_UNIFLASH_OPTYPE_EMMC_FLASH                = 0xF5
BOOTLOADER_UNIFLASH_OPTYPE_EMMC_VERIFY               = 0xF6

BOOTLOADER_UNIFLASH_STATUSCODE_SUCCESS                = "00 00 00 00"
BOOTLOADER_UNIFLASH_STATUSCODE_MAGIC_ERROR            = "01 00 00 10"
BOOTLOADER_UNIFLASH_STATUSCODE_OPTYPE_ERROR           = "01 00 00 20"
BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_ERROR            = "01 00 00 30"
BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_VERIFY_ERROR     = "01 00 00 40"
BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_ERASE_ERROR      = "01 00 00 50"

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

statuscodes = {
    BOOTLOADER_UNIFLASH_STATUSCODE_SUCCESS : "[STATUS] SUCCESS !!!\n",
    BOOTLOADER_UNIFLASH_STATUSCODE_MAGIC_ERROR : "[STATUS] ERROR: Incorrect magic number in file header !!!\n",
    BOOTLOADER_UNIFLASH_STATUSCODE_OPTYPE_ERROR : "[STATUS] ERROR: Invalid file operation type sent !!!\n",
    BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_ERROR : "[STATUS] ERROR: Flashing failed !!!\n",
    BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_VERIFY_ERROR : "[STATUS] ERROR: Flash verify failed !!!\n",
    BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_ERASE_ERROR : "[STATUS] ERROR: Flash erase failed !!!\n",
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
--file=am263xx-cc/hello_world.release.appimage --operation=flash --flash-offset=0x80000

and then the script can be called:
python can_uniflash.py --cfg=myconfig.cfg

If using command line arguments, following arguments are required:
    --file = Path to appimage
    --flash-offset = # When sending application image, make sure to flash at offset 0x80000 (default) or to whatever offset your bootloader is configured for
    --operation = Operation to perform on flash
'''

def open_can_port(port_open):
    try:
        m_objPCANBasic = PCANBasic()
        m_DLLFound = True
    except :
        print("\n[PCAN ERROR] : Unable to find the library: PCANBasic.dll !")
        m_DLLFound = False
        return
    if(port_open == True):
        return m_objPCANBasic
    if IsFD:
        # Make Sure to set correct timing parameters. Current parameters is as per SDK-CAN driver.
        BitrateFD = b'f_clock_mhz=80, nom_brp=1, nom_tseg1=67, nom_tseg2=12, nom_sjw=12, data_brp=1, data_tseg1=13, data_tseg2=2, data_sjw=1'
        stsResult = m_objPCANBasic.InitializeFD(PcanHandle,BitrateFD)
    else:
        Bitrate = PCAN_BAUD_1M

        stsResult = m_objPCANBasic.Initialize(PcanHandle, Bitrate)

    if stsResult != PCAN_ERROR_OK:
        print("")
        print("\n[PCAN ERROR] : Can not initialize. Please check the defines in the code. Instance not available or may be occupied.\n")
        sys.exit()
    return m_objPCANBasic

def close_pcan_port():
    if m_DLLFound:
        m_objPCANBasic.Uninitialize(PCAN_NONEBUS)

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

# Parse response header sent from EVM
def parse_response_evm(resp_bytes):
    status = None

    if(BOOTLOADER_UNIFLASH_STATUSCODE_SUCCESS in resp_bytes):
        status = statuscodes[BOOTLOADER_UNIFLASH_STATUSCODE_SUCCESS]
    elif(BOOTLOADER_UNIFLASH_STATUSCODE_MAGIC_ERROR in resp_bytes):
        status = statuscodes[BOOTLOADER_UNIFLASH_STATUSCODE_MAGIC_ERROR]
    elif(BOOTLOADER_UNIFLASH_STATUSCODE_OPTYPE_ERROR in resp_bytes):
        status = statuscodes[BOOTLOADER_UNIFLASH_STATUSCODE_OPTYPE_ERROR]
    elif(BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_ERROR in resp_bytes):
        status = statuscodes[BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_ERROR]
    elif(BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_VERIFY_ERROR in resp_bytes):
        status = statuscodes[BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_VERIFY_ERROR]
    elif(BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_ERASE_ERROR in resp_bytes):
        status = statuscodes[BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_ERASE_ERROR]
    else:
        status = "[ERROR] Invalid status code in response !!!"
    return status

def ReadMessages(canobj):
    """
    Function for reading PCAN-Basic messages
    """
    stsResult = PCAN_ERROR_OK
    strTemp = b""
    dataSize = 0

    ## We read at least one time the queue looking for messages. If a message is found, we look again trying to
    ## find more. If the queue is empty or an error occurr, we get out from the dowhile statement.
    while (not (stsResult & PCAN_ERROR_QRCVEMPTY)):
        if IsFD:
            stsResult = canobj.ReadFD(PcanHandle)
            dataSize = 64
        else:
            stsResult = canobj.Read(PcanHandle)
            dataSize = 8

        if stsResult[0] == PCAN_ERROR_OK:
            i=0
            for x in stsResult[1].DATA:
                if(i >= dataSize):
                    break
                else:
                    strTemp += b'%.2X ' % x
                    i = i+1

        if stsResult[0] != PCAN_ERROR_OK and stsResult[0] != PCAN_ERROR_QRCVEMPTY:
            return strTemp

        stsResult = stsResult[0]

    return strTemp

def WriteMessages(canobj, msg):
    '''
    Function for writing PCAN-Basic messages

    Function for writing messages on CAN-FD devices

    Returns:
        A TPCANStatus error code
    '''

    stsResult = PCAN_ERROR_OK
    dataSize = 0

    if IsFD:
        ## Sends a CAN message with extended ID, 64 data bytes, BRS enabled
        msgCanMessage = TPCANMsgFD()
        msgCanMessage.ID = 0xC0
        msgCanMessage.DLC = 15
        msgCanMessage.MSGTYPE = PCAN_MESSAGE_EXTENDED.value | PCAN_MESSAGE_FD.value | PCAN_MESSAGE_BRS.value
        dataSize = 64

        for i in range(min(len(msg),dataSize)):
            msgCanMessage.DATA[i] = msg[i]
            pass

        stsResult = canobj.WriteFD(PcanHandle, msgCanMessage)

    else:
        ## Sends a CAN message with extended ID, 8 data bytes
        msgCanMessage = TPCANMsg()
        msgCanMessage.ID = 0xC0
        msgCanMessage.LEN = 8
        msgCanMessage.MSGTYPE = PCAN_MESSAGE_EXTENDED.value
        dataSize = 8

        for i in range(min(len(msg),dataSize)):
            msgCanMessage.DATA[i] = msg[i]
            pass

        stsResult = canobj.Write(PcanHandle, msgCanMessage)

    return stsResult

# Sends the file to EVM via CAN, receives response from EVM and returns the response status
def send_receive_file(filename, get_response=True,reset=False,run=True,port_open=False):
    status = False
    timetaken = 0
    dataSize = 0
    try:
        bar_filesize = os.path.getsize(filename)
        stream = open(filename, 'rb')
    except FileNotFoundError:
        print('\n[ERROR] File [' + filename + '] not found !!!')
        sys.exit()


    if IsFD:
        dataSize = 63
    else:
        dataSize = 7

    if not os.path.exists(filename):
        print('\n[ERROR] File [' + filename + '] not found !!!')
        sys.exit()

    stream = open(filename, 'rb')
    filesize = os.path.getsize(filename)

    bar = tqdm(total=bar_filesize, unit="bytes", leave=False, desc="Sending {}".format(filename))

    gSer = open_can_port(port_open)

    '''
    Initiating ping between Board and PC.
    '''
    cmd="PING"
    encoded=cmd.encode('ascii')

    status = WriteMessages(gSer, encoded)
    time.sleep(0.005)
    in_data = str(ReadMessages(gSer)).replace("'","",2).replace("b","",1)

    cnt=0
    #"PONG" interpreted in Byte Array
    while(CAN_PONG not in in_data):
        status = WriteMessages(gSer, encoded)
        if(status == 0):
            cnt = cnt+1
        if(cnt == 29):
            print("")
            print ("[ERROR] CAN send failed, no response OR incorrect response from EVM OR cancelled by user,");
            print ("        Power cycle EVM and run this script again !!!");
            sys.exit()
        time.sleep(0.005)
        in_data = str(ReadMessages(gSer)).replace("'","",2).replace("b","",1)


    bar.update(16)
    bar.refresh()
    i=0
    seq=0
    tstart = time.time()

    while(filesize>0):
        if (seq>255):
            seq=0

        if(filesize >= dataSize):
            stream.seek(i,0)
            seq_ba = seq.to_bytes(1, "little")
            bytearr = stream.read(dataSize)
            status = WriteMessages(gSer, seq_ba+bytearr)
            time.sleep(0.005)
            in_data = str(ReadMessages(gSer)).replace("'","",2).replace("b","",1)

            #"MSGACK" interpreted in Byte Array
            if(CAN_MSG_ACK in in_data):
                i = i+dataSize
                filesize = filesize-dataSize
                bar.update(dataSize)
                bar.refresh()
                seq=seq+1

        else:
            stream.seek(i,0)
            seq_ba = seq.to_bytes(1, "little")
            bytearr = stream.read(dataSize)
            status = WriteMessages(gSer, seq_ba+bytearr)
            time.sleep(0.005)
            in_data = str(ReadMessages(gSer)).replace("'","",2).replace("b","",1)

            #"MSGACK" interpreted in Byte Array
            if(CAN_MSG_ACK in in_data):
                i = i+filesize
                stream.seek(filesize)
                bar.update(filesize)
                bar.refresh()
                filesize = 0
                seq=seq+1

    stream.close()

    while(filesize==0):
        cmd="LSTMSG"
        encoded=cmd.encode('ascii')

        WriteMessages(gSer, encoded)
        time.sleep(0.005)
        in_data = str(ReadMessages(gSer)).replace("'","",2).replace("b","",1)

        if(CAN_LST_MSG in in_data):
            filesize = -1

    while(filesize==-1):
        cmd="LSTMSGCF"
        encoded=cmd.encode('ascii')

        WriteMessages(gSer, encoded)
        time.sleep(0.005)

        in_data = str(ReadMessages(gSer)).replace("'","",2).replace("b","",1)

        if(CAN_MSG_ACK in in_data):
            filesize = 0

    resp_status = 0
    tstop = time.time()
    timetaken = timetaken + round(tstop-tstart, 2)

    if( run == True ) :
        run_cmd(gSer)
        resp_header  = str(ReadMessages(gSer)).replace("'","",2).replace("b","",1)
        # Don't do the receive if get_response is False
        if(get_response):
            try:
                while(not resp_header):
                    resp_header  = str(ReadMessages(gSer)).replace("'","",2).replace("b","",1)
                if(BOOTLOADER_UNIFLASH_RESP_HEADER_MAGIC_NUMBER in resp_header):
                    resp_status = (parse_response_evm(resp_header))
                else:
                    print ("\n[ERROR] Incorrect magic number in Response Header !!!\n")
                    resp_status = None
            except:
                resp_status = None
            if( resp_status is None ) :
                print ("[ERROR] CAN recv failed, no response OR incorrect response from EVM OR cancelled by user,\n")
                print ("Power cycle EVM and run this script again !!!")
                sys.exit()

    close_pcan_port()
    bar.close()

    return resp_status, timetaken

def reset_cmd(gSer):
    if IsFD:
        dataSize = 63
    else:
        dataSize = 7
    '''
    Initiating reset between Board and PC.
    '''
    cmd="RESET"
    encoded=cmd.encode('ascii')

    status = WriteMessages(gSer, encoded)
    cnt = 0
    time.sleep(0.005)
    in_data = str(ReadMessages(gSer)).replace("'","",2).replace("b","",1)
    #"PONG" interpreted in Byte Array
    while(CAN_MSG_ACK not in in_data):
        status = WriteMessages(gSer, encoded)
        if(status == 0):
            cnt = cnt+1
        if(cnt == 29):
            print("")
            print ("Reset Failed ...")
            print ("        Power cycle EVM and run this script again !!!")
            sys.exit()
        time.sleep(0.005)
        in_data = str(ReadMessages(gSer)).replace("'","",2).replace("b","",1)

def run_cmd(gSer):
    if IsFD:
        dataSize = 63
    else:
        dataSize = 7
    '''
    Initiating Run CMD between Board and PC.
    '''
    cmd="RUN"
    encoded=cmd.encode('ascii')

    status = WriteMessages(gSer, encoded)
    time.sleep(0.005)
    cnt = 0
    in_data = str(ReadMessages(gSer)).replace("'","",2).replace("b","",1)
    #"ACK" interpreted in Byte Array
    while(CAN_MSG_ACK not in in_data):
        status = WriteMessages(gSer, encoded)
        if(status == 0):
            cnt = cnt+1
        if(cnt == 29):
            print("")
            print ("Reset Failed ...")
            print ("        Power cycle EVM and run this script again !!!")
            sys.exit()
        time.sleep(0.005)
        in_data = str(ReadMessages(gSer)).replace("'","",2).replace("b","",1)

def send_file_by_parts(l_cfg):
    orig_f_name = l_cfg.filename
    orig_offset = l_cfg.offset

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
        if(i == (num_parts-1) and remain_size==0):
            status, timetaken = send_receive_file(tempfilename,True, reset=False, run=True,port_open=True)
        else:
            status, timetaken = send_receive_file(tempfilename,True, reset=True, run=False, port_open=False)

        total_time_taken += timetaken

        # delete the temporary file
        os.remove(part_filename)
        os.remove(tempfilename)

    # Send the last part, if there were residual bytes
    if(remain_size > 0):
        start = num_parts*BOOTLOADER_UNIFLASH_BUF_SIZE
        end = -1 # Read till the end of original file

        part_data = f_bytes[start:end]
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
        status, timetaken = send_receive_file(tempfilename,port_open=True)
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

    if(len(sys.argv) < 2):
        print(g_script_description)
        print(g_cfg_file_description)
        sys.exit()

    cmdlinecfg = LineCfg(cfg_src="cmd")

    my_parser = argparse.ArgumentParser(description=g_script_description)
    my_parser.add_argument('-f', '--file', required=False, help="Filename to send for an operation. Not required if using config mode (--cfg)")
    my_parser.add_argument('-o', '--flash-offset', required=False, help="Offset (in hexadecimal format starting with a 0x) at which the flash/verify flash is to be done. Not required if using config mode (--cfg)")
    my_parser.add_argument('--operation', required=False, help='Operation to be done on the file => "flash" or "flashverify" or "erase" or "flash-xip" or "flashverify-xip" or "flash-phy-tuning-data" or "flash-emmc" or "flashverify-emmc". Not required if using config mode (--cfg)')
    my_parser.add_argument('--erase-size', required=False, help='Size of flash to erase. Only valid when operation is "erase"')
    my_parser.add_argument('--cfg', required=False, help=g_cfg_file_description)

    args = my_parser.parse_args()
    config_file = args.cfg
    cmdlinecfg.filename = args.file
    cmdlinecfg.offset = args.flash_offset
    cmdlinecfg.optype = args.operation
    cmdlinecfg.erase_size = args.erase_size

    if(config_file != None):
        # Check if file exists
        if not os.path.exists(config_file):
            print('\n [ERROR] Configuration file [' + config_file + '] not found !!!\n')
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
            print("\nParsing config file ... SUCCESS. Found {} command(s) !!!".format(len(filecfg.cfgs)))
            print("")

            # loop through cfgs and lines, skip the process for flashwriter
            for i in range(0, len(filecfg.cfgs)):
                if(i != filecfg.flash_writer_index):
                    line = filecfg.lines[i]
                    linecfg = filecfg.cfgs[i]
                    print("\nExecuting command {} of {} ...".format(i+1, len(filecfg.cfgs)))
                    print("Command arguments : {}".format(line.rstrip('\n')))
                    # Check if the size of application image is larger than buffer size in target side.
                    f_size = 0
                    if linecfg.filename is not None:
                        f_size = os.path.getsize(linecfg.filename)

                    tempfilename = create_temp_file(linecfg)
                    status, timetaken = send_receive_file(tempfilename, get_response=True)

                    orig_filename = linecfg.filename
                    if(linecfg.optype == "erase"):
                        print("\nSent flash erase command.")
                    elif(linecfg.optype == "flash-phy-tuning-data"):
                        print("\nSent flash phy tuning data in {}s.".format(timetaken))
                    else:
                        print("Sent {} of size {} bytes in {}s.".format(orig_filename, os.path.getsize(orig_filename), timetaken))
                    print(status)
                    # Delete the tempfile if it exists
                    if(os.path.exists(tempfilename)):
                        os.remove(tempfilename)

            print("\nAll commands from config file are executed !!!\n")
            if("SUCCESS" in status):
                print("Connect to UART in 2 seconds to see logs from UART !!!")

    else:
        # Validate the cmdline config
        status = cmdlinecfg.validate()
        if(status != 0):
            print(status)
            if(cmdlinecfg.exit_now == True):
                sys.exit(2)

        if(cmdlinecfg.ops_invalid == False):
            # Check if the size of application image is larger than buffer size in target side.
            f_size = 0
            if cmdlinecfg.filename is not None:
                f_size = os.path.getsize(cmdlinecfg.filename)

            # Send normally
            tempfilename = create_temp_file(cmdlinecfg)
            status, timetaken = send_receive_file(tempfilename)
            # Delete the tempfile if it exists
            if(os.path.exists(tempfilename)):
                os.remove(tempfilename)

            orig_filename = cmdlinecfg.filename
            if(cmdlinecfg.optype == "erase"):
                print("Sent flash erase command.")
            elif(cmdlinecfg.optype == "flash-phy-tuning-data"):
                print("Sent flash phy tuning data in {}s.".format(timetaken))
            else:
                print("Sent {} of size {} bytes in {}s.".format(orig_filename, os.path.getsize(orig_filename), timetaken))
            print(status)
            if("SUCCESS" in status):
                print("Connect to UART in 2 seconds to see logs from UART !!!")

# Class definitions used
class LineCfg():
    def __init__(self, line=None, filename=None, optype=None, offset=None, erase_size=None, cfg_src="cfg"):
        self.line = line
        self.filename = filename
        self.optype = optype
        self.offset = offset
        self.erase_size = erase_size
        self.found_flashwriter_cmd = False
        self.ops_invalid = False
        self.cfg_src = cfg_src
        self.exit_now = False

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

        elif(self.line==None and self.cfg_src=="cmd"):
            # Called from main, with cmd line arguments
            # We have the filename, optype etc already filled

            self.found_flashwriter_cmd = False

            if(self.optype == None):
                self.ops_invalid = False
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

    def parse(self):
        f = open(self.filename, "r")
        lines = f.readlines()
        f.close()
        parse_status = 0
        linecount = 0
        valid_cfg_count = 0

        for line in lines:
            linecfg = LineCfg(line=line)
            status = linecfg.validate()

            if(status != 0 and status != "invalid_line"):
                self.errors.append("[ERROR] Parsing error found on line {} of {}\n{}\n".format(linecount+1, self.filename, status))
            else:
                if(status != "invalid_line"):
                    self.cfgs.append(linecfg)
                    self.lines.append(line.rstrip('\n'))
                    valid_cfg_count += 1
            linecount += 1

        if(len(self.errors) > 0):
            # Some commands have errors
            parse_status = "Parsing config file ... ERROR. {} error(s).\n\n".format(len(self.errors)) + "\n".join(self.errors)

        return parse_status

if __name__ == "__main__":
    try:
        main(sys.argv[1:])
    except KeyboardInterrupt:
        print("")
        print("[ERROR] CAN send failed, Execution cancelled by the user")