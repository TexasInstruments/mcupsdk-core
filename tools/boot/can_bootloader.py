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

usage_string = '''
USAGE: python can_bootloader.py [OPTIONS]

-f, --file=          Path to the appimage binary
'''

m_objPCANBasic = None
m_DLLFound = False
IsFD = True    # Set it to False if std CAN Protocol to be used.

PcanHandle = PCAN_USBBUS1

BOOTLOADER_CAN_STATUS_LOAD_SUCCESS            = "43 43 55 53"
BOOTLOADER_CAN_STATUS_LOAD_FAIL               = "4C 49 41 46"
BOOTLOADER_CAN_STATUS_APPIMAGE_SIZE_EXCEEDED  = "44 43 58 45"
CAN_PONG = "50 4F 4E 47"
CAN_LST_MSG = "4C 53 54 4D 53 47 41 4B"
CAN_MSG_ACK = "4D 53 47 41 43 4B"

def open_can_port():
    try:
        m_objPCANBasic = PCANBasic()
        m_DLLFound = True
    except :
        print("\n[PCAN ERROR] : Unable to find the library: PCANBasic.dll !")
        m_DLLFound = False
        return

    if IsFD:
        # Make Sure to set correct timing parameters. Current parameters is as per SDK-CAN driver.
        BitrateFD = b'f_clock_mhz=80, nom_brp=1, nom_tseg1=67, nom_tseg2=12, nom_sjw=12, data_brp=1, data_tseg1=13, data_tseg2=2, data_sjw=1'
        stsResult = m_objPCANBasic.InitializeFD(PcanHandle,BitrateFD)
    else:
        Bitrate = PCAN_BAUD_1M

        stsResult = m_objPCANBasic.Initialize(PcanHandle, Bitrate)

    if stsResult != PCAN_ERROR_OK:
        print("\n[PCAN ERROR] : Can not initialize. Please check the defines in the code. Instance not available or may be occupied.")

    return m_objPCANBasic

def close_pcan_port():
    if m_DLLFound:
        m_objPCANBasic.Uninitialize(PCAN_NONEBUS)

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
def send_receive_file(filename, get_response=True,reset=False,run=True):
    status = False
    timetaken = 0
    dataSize = 0
    try:
        bar_filesize = os.path.getsize(filename)
        stream = open(filename, 'rb')
    except FileNotFoundError:
        print('[ERROR] File [' + filename + '] not found !!!')
        sys.exit()


    if IsFD:
        dataSize = 63
    else:
        dataSize = 7

    if not os.path.exists(filename):
        print('[ERROR] File [' + filename + '] not found !!!')
        sys.exit()

    stream = open(filename, 'rb')
    filesize = os.path.getsize(filename)

    bar = tqdm(total=bar_filesize, unit="bytes", leave=False, desc="Sending {}".format(filename))

    ser = open_can_port()

    '''
    Initiating ping between Board and PC.
    '''
    cmd="PING"
    encoded=cmd.encode('ascii')

    status = WriteMessages(ser, encoded)
    time.sleep(0.005)
    in_data = str(ReadMessages(ser)).replace("'","",2).replace("b","",1)

    cnt=0
    #"PONG" interpreted in Byte Array
    while(CAN_PONG not in in_data):
        status = WriteMessages(ser, encoded)
        if(status == 0):
            cnt = cnt+1
        if(cnt == 29):
            print("")
            print ("[ERROR] CAN send failed, no response OR incorrect response from EVM OR cancelled by user,");
            print ("        Power cycle EVM and run this script again !!!");
            sys.exit()
        time.sleep(0.005)
        in_data = str(ReadMessages(ser)).replace("'","",2).replace("b","",1)


    bar.update(16)
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
            status = WriteMessages(ser, seq_ba+bytearr)
            time.sleep(0.005)
            in_data = str(ReadMessages(ser)).replace("'","",2).replace("b","",1)

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
            status = WriteMessages(ser, seq_ba+bytearr)
            time.sleep(0.005)
            in_data = str(ReadMessages(ser)).replace("'","",2).replace("b","",1)

            #"MSGACK" interpreted in Byte Array
            if(CAN_MSG_ACK in in_data):
                i = i+filesize
                stream.seek(filesize)
                bar.update(filesize)
                bar.refresh()
                filesize = 0
                seq=seq+1

    stream.close()
    #tstop = time.time()
    #timetaken = timetaken+round(tstop-tstart, 2)
    #tstart = time.time()

    while(filesize==0):
        cmd="LSTMSG"
        encoded=cmd.encode('ascii')

        WriteMessages(ser, encoded)
        time.sleep(0.005)
        in_data = str(ReadMessages(ser)).replace("'","",2).replace("b","",1)

        if(CAN_LST_MSG in in_data):
            filesize = -1

    while(filesize==-1):
        cmd="LSTMSGCF"
        encoded=cmd.encode('ascii')

        WriteMessages(ser, encoded)
        time.sleep(0.005)

        in_data = str(ReadMessages(ser)).replace("'","",2).replace("b","",1)

        if(CAN_MSG_ACK in in_data):
            filesize = 0

    resp_status = 0
    tstop = time.time()
    timetaken = timetaken + round(tstop-tstart, 2)

    if(run == True):
        run_cmd(ser)

    # Don't do the receive if get_response is False
    if(get_response):
        try:
            while(resp_status==0):
                in_data = str(ReadMessages(ser)).replace("'","",2).replace("b","",1)
                if(BOOTLOADER_CAN_STATUS_LOAD_SUCCESS in in_data):
                    print ("\n[STATUS] BOOTLOADER_CAN_STATUS_LOAD_SUCCESS!!!\n")
                    resp_status = 1
                elif(BOOTLOADER_CAN_STATUS_LOAD_FAIL in in_data):
                    print ("\n[STATUS] BOOTLOADER_CAN_STATUS_LOAD_CPU_FAIL!!!\n")
                    resp_status = -1
                    sys.exit()
                elif(BOOTLOADER_CAN_STATUS_APPIMAGE_SIZE_EXCEEDED in in_data):
                    print ("\n[STATUS] BOOTLOADER_CAN_STATUS_APPIMAGE_SIZE_EXCEEDED!!!\n")
                    resp_status = -1
                    sys.exit()
        except:
            status = None
        if( status is None ) :
            print("")
            print ("Power cycle EVM and run this script again !!!")
            sys.exit()

    close_pcan_port()

    bar.close()

    return resp_status, timetaken

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

def main(argv):

    def help() :
        print(usage_string)

    appimage_file = None
    try:
        opts, args = getopt.getopt(argv,"hf:",["help", "file="])
    except getopt.GetoptError:
        help()
        sys.exit()
    for opt, arg in opts:
        if opt in ("-h", "--help"):
            help()
            sys.exit()
        elif opt in ("-f, --file"):
            appimage_file = arg

    status = 0

    if(appimage_file == None):
        status = "[ERROR] Provide path to an appimage binary to be sent via CAN with option -f or --file=, , use -h option to see detailed help !!!"
        print(status)
        sys.exit()

    if(status == 0):
        # Check appimage files exists
        try:
            appimage_file_handle = open(appimage_file, "r")
        except FileNotFoundError:
            status = '[ERROR] Application file [' + appimage_file + '] not found !!!'
            print(status)
            sys.exit()

        # All good, now send files
        print("Sending the application {} ...".format(appimage_file))
        send_status, timetaken = send_receive_file(appimage_file, get_response=True)
        print("Sent application {} of size {} bytes in {}s.".format(appimage_file, os.path.getsize(appimage_file), timetaken))
        if(send_status == 1):
            print("Connect to UART to see logs from UART !!!")


if __name__ == "__main__":
    try:
        main(sys.argv[1:])
    except KeyboardInterrupt:
        print("")
        print("[ERROR] CAN send failed, Execution cancelled by the user")