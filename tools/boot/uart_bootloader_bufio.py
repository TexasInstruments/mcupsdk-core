import os
import sys
import argparse
import time
import string
import struct
import random

try: 
    import serial
    from tqdm import tqdm
    from xmodem import XMODEM, XMODEM1k
except ImportError:
    print('[ERROR] Dependant modules not installed, use below pip command to install them. MAKE sure proxy for pip is setup if needed.')
    print('');
    print('python -m pip install pyserial tqdm xmodem --proxy={http://your proxy server:port or leave blank if no proxy}');
    sys.exit(2);

usage_string = '''
USAGE: python uart_bootloader.py [OPTIONS]

-p, --serial-port=   Serial port to use for the transfer. 
                     COM3, COM4 etc if on windows and /dev/ttyUSB1,
                     /dev/ttyUSB2 etc if on linux. Mandatory argument

-b, --bootloader=    Path to the UART Bootloader binary

-f, --file=          Path to the appimage binary
'''

BOOTLOADER_UART_STATUS_LOAD_SUCCESS            = 0x53554343
BOOTLOADER_UART_STATUS_LOAD_FAIL               = 0x4641494C
BOOTLOADER_UART_STATUS_APPIMAGE_SIZE_EXCEEDED  = 0x45584344

# BUFFERED IO PROTOCOL DEFINES
BOOTLOADER_BUF_IO_MAGIC                      = 0xBF0000BF
BOOTLOADER_BUF_IO_OK                         = 0xBF000000
BOOTLOADER_BUF_IO_ERR                        = 0xBF000001
BOOTLOADER_BUF_IO_FILE_RECEIVE_COMPLETE      = 0xBF000002
BOOTLOADER_BUF_IO_SEND_FILE                  = 0xBF000003

mySerPort = "No Serial Port Chosen"
myBaudRate = 115200

def open_serial_port(ser_port, baudrate, timeout):
    s = None
    try:
        s = serial.Serial(port=ser_port, baudrate=baudrate, timeout=timeout)
    except serial.serialutil.SerialException:
        print('[ERROR] Serial port [' + ser_port + '] not found or not accessible !!!');
        sys.exit();
    return s

def close_serial_port(s):
    s.close()

def generate_temp_filename(n):
    letters = string.ascii_lowercase
    s = ''.join(random.choice(letters) for i in range(n))
    s += '.tmp'
    return s

# Parse buf io requests from EVM
def parse_response_buf_io(resp_bytes):
    '''
    /* Buffered IO protocol request header */
    typedef struct Bootloader_BufIoProtocolReq_s
    {
        uint32_t magic;
        uint32_t virtMemOffset;
        uint32_t cmd;
        uint32_t len;

    } Bootloader_BufIoProtocolReq;
    '''
    bufio_header_str = "<LLLL"

    status = -1

    magic, virt_mem_offset, cmd , req_length = struct.unpack(bufio_header_str, resp_bytes)

    if(magic == BOOTLOADER_BUF_IO_MAGIC):
        status = 0
    else:
        status = -1

    return status, cmd, virt_mem_offset, req_length

def xmodem_recv_file(port, baudrate, timeout):
    
    respfilename = generate_temp_filename(15)
    respfile = open(respfilename, "wb")

    s = open_serial_port(port, baudrate, timeout)

    def getc(size, timeout=1):
        return s.read(size) or None

    def putc(data, timeout=1):
        return s.write(data)  # note that this ignores the timeout

    modem = XMODEM1k(getc, putc)
    status = modem.recv(respfile, retry=32, quiet=True)

    close_serial_port(s)

    respfile.close()

    return respfilename

def xmodem_send_file(port, baudrate, timeout, f_stream, show_progress=True):

    status = None
    s = open_serial_port(port, baudrate, timeout)

    f_stream.seek(0, os.SEEK_END)
    bar_filesize = f_stream.tell()
    f_stream.seek(0, os.SEEK_SET)

    if show_progress:
        bar = tqdm(total=bar_filesize, unit="bytes", leave=True, desc="Sending file part ...")
        def getc(size, timeout=1):
            return s.read(size) or None

        def putc(data, timeout=1):
            bar.update(len(data));
            bar.refresh()
            return s.write(data)  # note that this ignores the timeout

        modem = XMODEM1k(getc, putc)

        status = modem.send(f_stream, retry=100, quiet=True)

        bar.close()
    else:
        def getc(size, timeout=1):
            return s.read(size) or None

        def putc(data, timeout=1):
            return s.write(data)  # note that this ignores the timeout

        modem = XMODEM1k(getc, putc)

        status = modem.send(f_stream, retry=100, quiet=True)

    close_serial_port(s)
    return status

def uart_receive_bytes(port, baudrate, timeout, sz):
    s = open_serial_port(port, baudrate, timeout)
    buf = s.read(sz)
    close_serial_port(s)
    return buf

def send_file_buf_io(filename, serialport, baudrate=115200, timeout=10, get_response=True, show_progress=True):
    status = False;
    timetaken = 0

    total_file_size = os.path.getsize(filename)
    print(f"Total filesize = {total_file_size} bytes")

    try:
        stream = open(filename, 'rb')
    except FileNotFoundError:
        print('[ERROR] File [' + filename + '] not found !!!');
        sys.exit();

    file_bytes = stream.read()
    stream.close()
    
    stop_transaction = False

    respfilename = "resp_cmd.dat"
    resp_status = -1
    cmd = -1
    virt_mem_offset = -1
    req_length = -1

    print("Waiting for requests from Target ...\n")

    tstart = time.time()
    while(not stop_transaction):
        resp_file_name = xmodem_recv_file(serialport, baudrate, timeout)
        resp_fh = open(resp_file_name, "rb")
        buf = resp_fh.read(16)
        resp_status, cmd, virt_mem_offset, req_length = parse_response_buf_io(buf)
        print(f"CMD = {hex(cmd)}, VIRT MEM OFFSET = {hex(virt_mem_offset)}, REQ LENGTH = {req_length}")

        if(resp_status == 0):
            if(cmd == BOOTLOADER_BUF_IO_SEND_FILE):
                print(f"Received file part request for {req_length} bytes ...")
                if(virt_mem_offset+req_length < total_file_size):
                    # Create a tempfile with the size requested
                    with open("tempfilepart.part", 'wb') as f_part:
                        f_part.write(file_bytes[virt_mem_offset:virt_mem_offset+req_length])
                else:
                    # To save on I/O, target will always request for buffer size number of bytes. Since we have reached end of file,
                    # we can send whatever is remaining
                    print(f"Request exceeding file size !! Sending remaining {total_file_size % req_length} bytes ...")
                    with open("tempfilepart.part", 'wb') as f_part:
                        f_part.write(file_bytes[virt_mem_offset:total_file_size])
                part_stream = open("tempfilepart.part", 'rb')
                xmodem_send_file(serialport, baudrate, timeout, part_stream, show_progress)
                part_stream.close()
                print("Done")
                os.remove("tempfilepart.part")

            elif(cmd == BOOTLOADER_BUF_IO_FILE_RECEIVE_COMPLETE):
                print(f"Transaction complete.")
                stop_transaction = True
            else:
                print("Bad command received!")
        else:
            pass

    tstop = time.time()
    timetaken = (tstop - tstart)

    return resp_status, timetaken

def main(argv):

    def help() :
        print(usage_string);

    my_parser = argparse.ArgumentParser(description="Python script to be used in conjunction with UART bootloader")

    my_parser.add_argument('-f', '--file', help='Path to the appimage binary file to be sent to UART bootloader')
    my_parser.add_argument('-b', '--bootloader', required=True, help='[Mandatory argument] Path to the UART bootloader image.')
    my_parser.add_argument('-p', '--serial-port', required=True, help='[Mandatory argument] Serial port to use for the transfer. COM3, COM4 etc if on windows and /dev/ttyUSB1, /dev/ttyUSB2 etc if on linux.')

    args = my_parser.parse_args()

    is_app_present = False

    # Check both SBL and appimage files exists
    if args.bootloader is not None:
        if not os.path.exists(args.bootloader):
            status = f"[ERROR] Bootloader file [{args.bootloader}] not found !!!"
            print(status);
            sys.exit()

    if args.file is not None:
        if not os.path.exists(args.file):
            status = f"[ERROR] Application file [{args.file}] not found !!!"
            print(status);
            sys.exit()
        else:
            is_app_present = True

    # Send the SBL
    print(f"Sending the UART bootloader {args.bootloader} ...")
    bf = open(args.bootloader, "rb")
    t_start = time.time()
    send_status = xmodem_send_file(args.serial_port, 115200, 10, bf, show_progress=True)
    timetaken = time.time() - t_start
    bf.close()
    print(f"Sent bootloader {args.bootloader} of size {os.path.getsize(args.bootloader)} bytes in {timetaken}s.")

    # Send the application if present
    if is_app_present:
        print(f"Sending the application {args.file} ...")
        send_status, timetaken = send_file_buf_io(args.file, args.serial_port)
        if send_status != -1:
            print(f"Transfer Succesful, completed in {timetaken} seconds")
            print("Connect to UART in 2 seconds to see logs from UART !!!")
            os.system(f"minicom --device {args.serial_port} -b 115200")

if __name__ == "__main__":
   main(sys.argv[1:])
