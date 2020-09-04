import os
import sys
import getopt
import time

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
'''

BOOTLOADER_UART_STATUS_LOAD_SUCCESS            = 0x53554343
BOOTLOADER_UART_STATUS_LOAD_FAIL               = 0x4641494C
BOOTLOADER_UART_STATUS_APPIMAGE_SIZE_EXCEEDED  = 0x45584344

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

    try:
        opts, args = getopt.getopt(argv,"hf:b:p:",["help", "file=","bootloader=","serial-port="])
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

    status = 0

    if(serialport == None):
        status = "[ERROR] Provide a serial port with option -p or --serial-port=, use -h option to see detailed help !!!"
        print(status)
        sys.exit()

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
        send_status, timetaken = xmodem_send_receive_file(appimage_file, serialport, get_response=True)
        print("Sent application {} of size {} bytes in {}s.".format(appimage_file, os.path.getsize(appimage_file), timetaken))
        print(send_status)
        if("SUCCESS" in send_status):
            print("Connect to UART in 5 seconds to see logs from UART !!!")

if __name__ == "__main__":
   main(sys.argv[1:])