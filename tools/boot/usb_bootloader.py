import os
import sys
import getopt
import time
import subprocess

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

-b, --bootloader=    Path to the UART Bootloader binary

-f, --file=          Path to the appimage binary

[optional args]

-i, --interface      USB DFU interface number
                     Default = 0

-a, --alt-setting    USB DFU alternate setting number
                     Default = 0

-t, --transfer-size  USB DFU data size per setup transfer.
                     transfer size should be a multiple of 64 and it should be
                     less or equal to 512
                     Default = 512
'''

mySerPort = "No Serial Port Chosen"
myBaudRate = 115200
ser = serial.Serial(timeout=3)


# Max appimage size supported by SBL DFU bootloader
MAX_APPIMAGE_SIZE = 0x60000

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

# Function to detect dfu device enumeration
def wait_for_enumeration():

    enum_done = False
    ls_dfu = "dfu-util -l"
    while enum_done == False:
        subprocess.run(ls_dfu + " > temp_file",shell=True,stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT)
        temp_file = open('temp_file')
        string = temp_file.read()

        if ("Found DFU" in string and "UNKNOWN" not in string ):
            enum_done = True
        else:
            print("------------------------------------------------------")
            print("Waiting for DFU device to be enumerated ....")
            print("------------------------------------------------------")
            # If not detected then wait sometime before retrying
            time.sleep(0.5)

    temp_file.close()
    os.remove('temp_file')

def dfu_fw_send(filename,intf=0,alt=0,xfer_size=512,reset_req=False):
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

    if(reset_req == False):

        cmd = "dfu-util -a {0} -i {1} -t {2} -D {3}".format(alt,intf,xfer_size,filename)
    else:
        cmd = "dfu-util -a {0} -i {1} -t {2} -R -D {3}".format(alt,intf,xfer_size,filename)

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

def main(argv):

    def help() :
        print(usage_string)

    serialport = None
    appimage_file = None
    bootloader_file = None
    interface = 0
    alt_setting = 0
    transfer_size = 512

    try:
        opts, args = getopt.getopt(argv,"hf:b:i:a:t:",["help", "file=","bootloader=","interface=","alt-setting=","transfer-size="])
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
        elif opt in ("-i", "--interface"):
            try:
                interface = int(arg)
            except ValueError :
                print("[ERROR] --interface , -i should be integer")
                help()
                sys.exit()

        elif opt in ("-a", "--alt-setting"):
            try:
                alt_setting = int(arg)
            except ValueError :
                print("[ERROR] --alt-setting , -a should be integer")
                help()
                sys.exit()
        elif opt in ("-t", "--transfer-size"):
            try:
                transfer_size = int(arg)
            except ValueError :
                print("[ERROR] --transfer-size, -t should be integer")
                help()
                sys.exit()

    status = 0

    if(bootloader_file == None):
        status = "[ERROR] Provide path to the uart bootloader binary with option -b or --bootloader=, use -h option to see detailed help !!!"
        print(status)
        sys.exit()

    if(appimage_file == None):
        status = "[ERROR] Provide path to an appimage binary to be sent via UART with option -f or --file=, , use -h option to see detailed help !!!"
        print(status)
        sys.exit()

    # check valid transfer size
    if(transfer_size != None):
        if(transfer_size % 64 != 0 or transfer_size > 512):
            status = "[ERROR] USB DFU transfer size should be a multiple of 64 and it should be less than 512 use -h option to see detailed help !!!"
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

        # check if app image is of valid size
        app_size = os.path.getsize(appimage_file)
        if(app_size > MAX_APPIMAGE_SIZE ):
            status = "[ERROR] Currently Supported Max appimage size is {0} . Please Use Flash based Boot for appimage size greater than {1}".format(app_size, app_size)
            print(status)
            sys.exit()

        # All good, now send files
        print("----------------------------------------------------------------")
        print("Sending the SBL DFU bootloader {} ...".format(bootloader_file))

        send_status, timetaken = dfu_fw_send(bootloader_file, interface ,alt_setting,transfer_size)

        print("----------------------------------------------------------------")
        print("Sent bootloader {} of size {} bytes in {}s.".format(bootloader_file, os.path.getsize(bootloader_file), timetaken))
        print("----------------------------------------------------------------")
        print("")

        print("----------------------------------------------------------------")
        print("Sending the application {} ...".format(appimage_file))

        send_status, timetaken = dfu_fw_send(appimage_file, interface ,alt_setting,transfer_size,True)

        print("----------------------------------------------------------------")
        print("Sent application {} of size {} bytes in {}s.".format(appimage_file, os.path.getsize(appimage_file), timetaken))
        print("----------------------------------------------------------------")

if __name__ == "__main__":
    main(sys.argv[1:])
