import os
import sys
import getopt
import time
import shlex
import string
import struct

try:
    import serial
    from tqdm import tqdm
    from xmodem import XMODEM, XMODEM1k
except ImportError:
    print('[ERROR] Dependant modules not installed, use below pip command to install them. MAKE sure proxy for pip is setup if needed.')
    print('')
    print('python -m pip install pyserial tqdm xmodem --proxy={http://your proxy server:port or leave blank if no proxy}')
    sys.exit(2)

usage_string = '''
USAGE: python uart_bootloader.py [OPTIONS]

-p, --serial-port=   Serial port to use for the transfer.
                     COM3, COM4 etc if on windows and /dev/ttyUSB1,
                     /dev/ttyUSB2 etc if on linux. Mandatory argument

-b, --bootloader=    Path to the UART Bootloader binary

-f, --file=          Path to the appimage binary

--soc=               SOC type (e.g., am64x_am243x). When set to am64x_am243x, uses segmented transfer.

--cfg=               Path to the configuration file with the entries of the two flags given above, namely bootloader and file.
                     This file can be used as an alternative to passing the two flags given above.
                     Use either the --cfg flag or the --bootloader and --file flags.
'''

BOOTLOADER_UART_STATUS_LOAD_SUCCESS            = 0x53554343
BOOTLOADER_UART_STATUS_LOAD_FAIL               = 0x4641494C
BOOTLOADER_UART_STATUS_APPIMAGE_SIZE_EXCEEDED  = 0x45584344

BOOTLOADER_END_OF_FILES_TRANSFER               = 0x454F4654
BOOTLOADER_END_OF_FILES_TRANSFER_WORD_LENGTH   = 4

BOOTLOADER_UART_MAGIC                          = 0xBF0000BF
BOOTLOADER_UART_FILE_RECEIVE_COMPLETE          = 0xBF000002
BOOTLOADER_UART_SEND_FILE                      = 0xBF000003

mySerPort = "No Serial Port Chosen"
myBaudRate = 115200
ser = serial.Serial(timeout=3)

def open_serial_port():
    ser.baudrate = 115200
    ser.port = mySerPort
    ser.open()

def close_serial_port():
    ser.close()

# Parse response sent from EVM
def parse_response_evm(filename):
    f = open(filename, "rb")
    resp_bytes = f.read(128)
    f.close()

    status = 0

    response = int.from_bytes(resp_bytes[0:4], 'little')

    if(response == BOOTLOADER_UART_STATUS_LOAD_SUCCESS):
        status = "[STATUS] Application load SUCCESS !!!"
    elif(response == BOOTLOADER_UART_STATUS_LOAD_FAIL):
        status = "[STATUS] ERROR: Application load FAILED !!!"
    elif(response == BOOTLOADER_UART_STATUS_APPIMAGE_SIZE_EXCEEDED):
        status = "[STATUS] ERROR: Application load FAILED, file size exceeds LIMIT on the EVM !!!"
    else:
        status = "[STATUS] ERROR: Bad response from EVM !!!"

    return status

# Parse message requests for program segments from EVM
def parse_msg_response(resp_bytes):
    '''
    /* UART protocol request header */
    typedef struct Bootloader_UartProtocolReq_s
    {
        uint64_t magic;
        uint64_t virtMemOffset;
        uint64_t cmd;
        uint64_t len;

    } Bootloader_UartProtocolReq;
    '''

    uart_header_str = "<QQQQ"

    status = -1

    magic, virt_mem_offset, cmd , req_length = struct.unpack(uart_header_str, resp_bytes)

    if(magic == BOOTLOADER_UART_MAGIC):
        status = 0
    else:
        status = -1

    return status, cmd, virt_mem_offset, req_length

# Send the file to EVM
def xmodem_send_file(port, baudrate, timeout, f_stream, show_progress):

    status = None
    open_serial_port();

    f_stream.seek(0, os.SEEK_END)
    bar_filesize = f_stream.tell()
    f_stream.seek(0, os.SEEK_SET)

    if show_progress:
        bar = tqdm(total=bar_filesize, unit="bytes", leave=True, desc="Sending file part ...")
        def getc(size, timeout=1):
            return ser.read(size) or None

        def putc(data, timeout=1):
            bar.update(len(data));
            bar.refresh()
            return ser.write(data)  # note that this ignores the timeout

        modem = XMODEM1k(getc, putc)

        status = modem.send(f_stream, retry=100, quiet=True)

        bar.close()
    else:
        def getc(size, timeout=1):
            return ser.read(size) or None

        def putc(data, timeout=1):
            return ser.write(data)  # note that this ignores the timeout

        modem = XMODEM1k(getc, putc)

        status = modem.send(f_stream, retry=100, quiet=True)

    close_serial_port()

    return status

#Receive the file from EVM
def xmodem_recv_file(port, baudrate, timeout):

    receiverespfilename = "receiverespfile.part"
    recvrespfile = open(receiverespfilename, "wb")

    open_serial_port();

    def getc(size, timeout=1):
        return ser.read(size) or None

    def putc(data, timeout=1):
        return ser.write(data)  # note that this ignores the timeout

    modem = XMODEM1k(getc, putc)
    status = modem.recv(recvrespfile, quiet=True, timeout=1000)

    close_serial_port()

    recvrespfile.close()

    return receiverespfilename

# Sends the mcelf image meta data, program segments as requested by EVM via xmodem
# Receives request from EVM, parse the request frame and send the requested segment
def xmodem_send_segments(filename, serialport, baudrate=115200, timeout=10):
    status = False
    timetaken = 0
    show_progress=True

    total_file_size = os.path.getsize(filename)
    print(f"Total filesize = {total_file_size} bytes")

    #Open the file
    try:
        stream = open(filename, 'rb')
    except FileNotFoundError:
        print('[ERROR] File [' + filename + '] not found !!!');
        sys.exit();

    #Read the file
    file_bytes = stream.read()
    stream.close()

    stop_transaction = False

    respfilename = "resp_cmd.dat"
    resp_status = -1
    offset = -1
    req_length = -1

    #Send the initial metadata of file of around 8kB
    tstart = time.time()

    with open("tempfilepart.part", 'wb') as f_part:
        f_part.write(file_bytes[0:8192])
    part_stream = open("tempfilepart.part", 'rb')
    xmodem_send_file(serialport, baudrate, timeout, part_stream, show_progress)
    part_stream.close()
    os.remove("tempfilepart.part")

    #Receive the request frame from target containing the offset and length of required segment
    while(not stop_transaction):
        resp_file_name = xmodem_recv_file(serialport, baudrate, 100)
        resp_fh = open(resp_file_name, "rb")
        buf = resp_fh.read(32)
        resp_fh.close()
        os.remove(resp_file_name)
        resp_status, cmd, virt_mem_offset, req_length = parse_msg_response(buf)

        if(resp_status==0):
            if(cmd == BOOTLOADER_UART_SEND_FILE):
                if(virt_mem_offset+req_length <= total_file_size):
                    # Create a tempfile with the size requested
                    with open("tempfilepart.part", 'wb') as f_part:
                        f_part.write(file_bytes[virt_mem_offset:virt_mem_offset+req_length])
                else:
                # Error
                   print(f"Request exceeding file size !! Sending remaining bytes ...")

                part_stream = open("tempfilepart.part", 'rb')
                xmodem_send_file(serialport, baudrate, timeout, part_stream, show_progress)
                part_stream.close()
                os.remove("tempfilepart.part")

            elif(cmd == BOOTLOADER_UART_FILE_RECEIVE_COMPLETE):
                resp_status = "[STATUS] Application load SUCCESS !!!"
                stop_transaction = True
            else:
                resp_status = "[STATUS] ERROR: Application load FAILED !!!"
        else:
            pass

    tstop = time.time()
    timetaken = (tstop - tstart)

    return resp_status, timetaken

# Sends the file to EVM via xmodem, receives response from EVM and returns the response status
def xmodem_send_receive_file(filename, serialport, baudrate=115200, get_response=True):
    status = False
    timetaken = 0
    try:
        global mySerPort, myBaudRate
        bar_filesize = os.path.getsize(filename)
        mySerPort = serialport
        myBaudRate = baudrate
        open_serial_port();
        stream = open(filename, 'rb')
    except FileNotFoundError:
        print('[ERROR] File [' + filename + '] not found !!!')
        sys.exit()
    except serial.serialutil.SerialException:
        print('[ERROR] Serial port [' + serialport + '] not found or not accessible !!!')
        sys.exit()

    bar_cursize = 0
    bar = tqdm(total=bar_filesize, unit="bytes", leave=False, desc="Sending {}".format(filename))

    def getc(size, timeout=1):
        return ser.read(size) or None

    def putc(data, timeout=1):
        bar.update(len(data))
        bar.refresh()
        return ser.write(data)  # note that this ignores the timeout

    try:
        modem = XMODEM1k(getc, putc)
        tstart = time.time()
        status = modem.send(stream, quiet=True, timeout=10, retry=10)
        tstop = time.time()
        timetaken = round(tstop-tstart, 2)
    except:
        status = False

    stream.close()

    if( status is False ) :
        print("")
        print ("[ERROR] XMODEM send failed, no response OR incorrect response from EVM OR cancelled by user,");
        print ("        Power cycle EVM and run this script again !!!");
        sys.exit()

    resp_status = 0

    # Don't do the receive if get_response is False
    if(get_response):
        respfilename = "resp_uart_bootloader.dat"
        try:
            respfile = open(respfilename, "wb")
            status = modem.recv(respfile, quiet=True, timeout=20)
            respfile.close()
            resp_status = parse_response_evm(respfilename)
            os.remove(respfilename)
        except:
            status = None
        if( status is None ) :
            print("")
            print ("[ERROR] XMODEM recv failed, no response OR incorrect response from EVM OR cancelled by user,");
            print ("        Power cycle EVM and run this script again !!!");
            sys.exit()

    close_serial_port()

    bar.close()

    return resp_status, timetaken

def main(argv):

    def help() :
        print(usage_string)

    serialport = None
    appimage_file = None
    bootloader_file = None
    config_file = None
    soc = None

    try:
        opts, args = getopt.getopt(argv,"hf:b:p:",["help", "file=","bootloader=","serial-port=","cfg=","soc="])
    except getopt.GetoptError:
        help()
        sys.exit()
    for opt, arg in opts:
        if opt in ("-h", "--help"):
            help()
            sys.exit()
        elif opt in ("-f, --file"):
            appimage_file = arg
        elif opt in ("-b", "--bootloader"):
            bootloader_file = arg
        elif opt in ("-p", "--serial-port"):
            serialport = arg
        elif opt in ("--cfg"):
            config_file = arg
        elif opt in ("--soc"):
            soc = arg

    status = 0

    if(serialport == None):
            status = "[ERROR] Provide a serial port with option -p or --serial-port=, use -h option to see detailed help !!!"
            print(status)
            sys.exit()

    if(config_file != None):
        # Check if file exists
        if not os.path.exists(config_file):
            print('[ERROR] Configuration file [' + config_file + '] not found !!!')
            sys.exit()

        print("")
        print("Parsing config file ...")
        filecfg = FileCfg(config_file)
        parse_status = filecfg.parse()

        if(filecfg.soc_index != None):
            device = filecfg.cfgs[filecfg.soc_index].soc
        else:
            device = None

        if(parse_status != 0):
            print(parse_status)
            sys.exit()

        else:

            print("Parsing config file ... SUCCESS. Found {} command(s) !!!".format(len(filecfg.cfgs)))
            print("")

            if(filecfg.soc_index != None):
                length = len(filecfg.cfgs) - 1
            else:
                length = len(filecfg.cfgs)

            if(filecfg.bootloader_index != None):
                cfg_bootloader_file = filecfg.cfgs[filecfg.bootloader_index].bootloader
                print("Executing command {} of {} ...".format(1, length))
                print("Found the UART bootloader ... sending {}".format(cfg_bootloader_file))
                send_status, timetaken = xmodem_send_receive_file(cfg_bootloader_file, serialport, get_response=False)
                print("Sent bootloader {} of size {} bytes in {}s.".format(cfg_bootloader_file, os.path.getsize(cfg_bootloader_file), timetaken))
                print("")

            else:
                status = "[ERROR] Provide path to the uart bootloader binary with option -b or --bootloader=, use -h option to see detailed help !!!"
                print(status)
                sys.exit()

            # loop through cfgs and lines, skip the process for bootloader
            for i in range(0, len(filecfg.cfgs)):
                if((i != filecfg.bootloader_index) and (i != filecfg.soc_index)):
                    line = filecfg.lines[i]
                    linecfg = filecfg.cfgs[i]
                    orig_filename = linecfg.filename

                    if(filecfg.soc_index != None):
                        curr = i
                    else:
                        curr = i+1

                    print("Executing command {} of {} ...".format(curr, length))
                    print("Command arguments : {}".format(line.rstrip('\n')))

                    if(device != "am64x_am243x"):
                       send_status, timetaken = xmodem_send_receive_file(linecfg.filename, serialport, get_response=True)
                    elif(device == "am64x_am243x"):
                        send_status, timetaken = xmodem_send_segments(linecfg.filename, serialport)

                    print("Sent {} of size {} bytes in {}s.".format(orig_filename, os.path.getsize(orig_filename), timetaken))
                    print(send_status)
                    print("")

            if(length != 1):
                magic_word_filename = "magic_word_file.dat"
                magic_word_file     = open(magic_word_filename,"wb")
                magic_word_bytes    = BOOTLOADER_END_OF_FILES_TRANSFER.to_bytes(BOOTLOADER_END_OF_FILES_TRANSFER_WORD_LENGTH,"big")
                magic_word_file.write(magic_word_bytes)
                magic_word_file.close()
                sendd_status, timetaken = xmodem_send_receive_file(magic_word_filename, serialport, get_response=False)
                print("")
                print(" Sent End Of File Transfer message of size {} bytes in {}s.".format( os.path.getsize(magic_word_filename), timetaken))
                print("")
                os.remove(magic_word_filename)
                print("All commands from config file are executed !!!")
                if("SUCCESS" in send_status):
                    print("Connect to UART in 5 seconds to see logs from UART !!!")

    else:
        # Validate the cmdline config
        if(bootloader_file == None):
            status = "[ERROR] Provide path to the uart bootloader binary with option -b or --bootloader=, use -h option to see detailed help !!!"
            print(status)
            sys.exit()

        if(appimage_file == None):
            status = "[ERROR] Provide path to an appimage binary to be sent via UART with option -f or --file=, , use -h option to see detailed help !!!"
            print(status)
            sys.exit()

        if(status == 0):
            # Check both SBL and appimage files exists
            try:
                appimage_file_handle = open(appimage_file, "r")
            except FileNotFoundError:
                status = '[ERROR] Application file [' + appimage_file + '] not found !!!'
                print(status)
                sys.exit()

            try:
                bootloader_file_handle = open(bootloader_file, "r")
            except FileNotFoundError:
                status = '[ERROR] Bootloader file [' + bootloader_file + '] not found !!!'
                print(status)
                sys.exit()

            # All good, now send files
            print("Sending the UART bootloader {} ...".format(bootloader_file))
            send_status, timetaken = xmodem_send_receive_file(bootloader_file, serialport, get_response=False)
            print("Sent bootloader {} of size {} bytes in {}s.".format(bootloader_file, os.path.getsize(bootloader_file), timetaken))
            print("")

            print("Sending the application {} ...".format(appimage_file))
            if(soc != None and soc == "am64x_am243x"):
                send_status, timetaken = xmodem_send_segments(appimage_file, serialport)
            else:
                send_status, timetaken = xmodem_send_receive_file(appimage_file, serialport, get_response=True)
            print("Sent application {} of size {} bytes in {}s.".format(appimage_file, os.path.getsize(appimage_file), timetaken))
            print(send_status)
            print("")
            if(soc != None and soc == "am64x_am243x"):
                magic_word_filename = "magic_word_file.dat"
                magic_word_file     = open(magic_word_filename,"wb")
                magic_word_bytes    = BOOTLOADER_END_OF_FILES_TRANSFER.to_bytes(BOOTLOADER_END_OF_FILES_TRANSFER_WORD_LENGTH,"big")
                magic_word_file.write(magic_word_bytes)
                magic_word_file.close()
                sendd_status, timetaken = xmodem_send_receive_file(magic_word_filename, serialport, get_response=False)
                print("")
                print(" Sent End Of File Transfer message of size {} bytes in {}s.".format( os.path.getsize(magic_word_filename), timetaken))
                print("")
                os.remove(magic_word_filename)
            if("SUCCESS" in send_status):
                print("Connect to UART in 5 seconds to see logs from UART !!!")

# Class definitions used
class LineCfg():
    def __init__(self, line=None, filename=None, bootloader=None, soc=None):
        self.line = line
        self.filename = filename
        self.bootloader = bootloader
        self.exit_now = False
        self.soc = soc

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
        if(self.line!=None):
            # Called from config_file
            config_dict = self.parse_to_dict(self.line)

            if not config_dict:
                status = "invalid_line"
            else:
                # check for bootloader
                if ("--bootloader" not in config_dict.keys() and "--soc" not in config_dict.keys()):
                    if "--file" not in config_dict.keys():
                        status = "[ERROR] No filename provided !!!"
                        return status
                    else:
                        self.filename = config_dict["--file"]

                    try:
                        f = open(self.filename)
                    except FileNotFoundError:
                        status = "[ERROR] File not found !!!"
                        return status

                elif("--bootloader" in config_dict.keys()):
                    self.bootloader = config_dict["--bootloader"]

                    try:
                        f = open(self.bootloader)
                    except FileNotFoundError:
                        status = "[ERROR] Bootloader file not found !!!"
                        return status

                elif("--soc" in config_dict.keys()):
                    self.soc = config_dict["--soc"]
                    return status
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
        self.soc_index = None
        self.bootloader_index = None

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
                    if(linecfg.bootloader != None):
                        if(found_fw == True):
                            # error, another instance of bootloader found
                            bootloader_error_msg = "[ERROR] Redefinition of Bootloader !!!"
                            self.errors.append("[ERROR] Parsing error found on line {} of {}\n{}\n".format(linecount+1, self.filename, bootloader_error_msg))
                        else:
                            found_fw = True
                            self.cfgs.append(linecfg)
                            self.lines.append(line.rstrip('\n'))
                            self.bootloader_index = valid_cfg_count
                            valid_cfg_count += 1
                    elif(linecfg.soc != None):
                        self.cfgs.append(linecfg)
                        self.lines.append(line.rstrip('\n'))
                        self.soc_index = valid_cfg_count
                        valid_cfg_count += 1
                    else:
                        self.cfgs.append(linecfg)
                        self.lines.append(line.rstrip('\n'))
                        valid_cfg_count += 1
            linecount += 1

        if(len(self.errors) > 0):
            # Some commands have errors
            parse_status = "Parsing config file ... ERROR. {} error(s).\n\n".format(len(self.errors)) + "\n".join(self.errors)

        return parse_status

if __name__ == "__main__":
   main(sys.argv[1:])