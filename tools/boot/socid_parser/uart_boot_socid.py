#!/usr/bin/env python3
import binascii
import struct
import sys
import argparse

# SCRIPT USAGE
# ------------
# usage: uart_boot_socid.py [-h] -d DEVICE (-s STRING | -f FILE)
#   -h, --help            show this help message and exit
# required arguments:
#
#   -d DEVICE, --device   - device for which the soc id will be parsed
#
#   -s STRING, --string   - Soc id reported from UART console
#                           (OR)
#   -f FILE, --file       - file which contains the soc id reported from UART console

g_script_description = '''
MCU+SDK utility script to parse SOC ID
'''

k3_devices = ['am243x', 'am64x']
mcu_devices = ['am263x', 'am273x', 'am263px']

r5InfoStr = 'I'

secInfoStr = 'BBHHH64B64B32B'
pubInfoStr = 'BB2B12B4B4B4B'

hwInfoStr = 'HBBBBH'
hsmPubInfoStr = '12BII'
hsmsecInfoStr = 'IIIII64B64B64B'

devTypedict = {
    '0xabcd0001': 'GP',
    '0xabcd0002': 'TEST',
    '0xabcd0003': 'EMU_FS',
    '0xabcd0004': 'HS_FS',
    '0xabcd0005': 'EMU_SE',
    '0xabcd0006': 'HS_SE',
}

def parse_mcu_soc_info(bin_str):
    # Decode byte array to struct
    try:
        hwInfo = struct.unpack(hwInfoStr, bin_str[0:8])
        r5Info = struct.unpack(r5InfoStr, bin_str[8:12])
        hsmPubInfo = struct.unpack(hsmPubInfoStr, bin_str[12:32])
        hsmSecInfo = struct.unpack(hsmsecInfoStr, bin_str[32:244])
    except:
        print('[ERROR] Invalid SoC Id!! Please check that the copied SoC Id belongs to the given device')
        sys.exit(1)

    # Print SoC HW Info Struct
    print('---------------------------')
    print('SoC ID HW Info:')
    print('---------------------------')
    print("partID               :", hex(hwInfo[0]))
    print("partNumber           :", hex(hwInfo[1]))
    print("PGVer                :", hex(hwInfo[2]))
    print("ROMVer               :", hex(hwInfo[3]))
    print("MetalVer             :", hex(hwInfo[4]))

    # Print R5 ROM Info Struct
    print('---------------------------')
    print('SoC ID R5 ROM Info:')
    print('---------------------------')
    print("r5 ROM Ver           :", hex(r5Info[0]))

    tmpList = list(hsmPubInfo[0:12])
    hexList = [hex(i) for i in tmpList]
    deviceName = ''.join(chr(int(c, 16)) for c in hexList[0:])

    # Print HSM Pub Info Struct
    print('---------------------------')
    print('SoC ID HSM Pub ROM Info:')
    print('---------------------------')
    print("devName              :", deviceName)
    print("devType              :", devTypedict[hex(hsmPubInfo[12])])
    print("hsm ROM Ver          :", hex(hsmPubInfo[13]))

    tmpList = list(hsmSecInfo[5:69])
    tiMPKHash = ''.join('{:02x}'.format(x) for x in tmpList)
    tmpList = list(hsmSecInfo[69:133])
    custMPKHash = ''.join('{:02x}'.format(x) for x in tmpList)
    tmpList = list(hsmSecInfo[133:197])
    uID = ''.join('{:02x}'.format(x) for x in tmpList)

    # Print HSM Sec Info Struct
    print('---------------------------')
    print('SoC ID HSM Sec ROM Info:')
    print('---------------------------')
    # print hsmSecInfo
    print("Prime                :", hex(hsmSecInfo[0]))
    print("Key Count            :", hex(hsmSecInfo[1]))
    print("Key Rev              :", hex(hsmSecInfo[2]))
    print("SWRV SBL             :", hex(hsmSecInfo[3]))
    print("SWRV HSM             :", hex(hsmSecInfo[4]))
    print("TI MPK Hash          :", tiMPKHash)
    print("Cust MPK Hash        :", custMPKHash)
    print("Unique ID            :", uID)

def parser_k3_soc_info(bin_str):
    try:
        numBlocks = list(struct.unpack('I', bin_str[0:4]))[0]
        pubROMInfo = struct.unpack(pubInfoStr, bin_str[4:32])
    except:
        print('[ERROR] Invalid SoC Id!! Please check that the copied SoC Id belongs to the given device')
        sys.exit(1)

    sub_block_id = pubROMInfo[0]
    sub_block_sz = pubROMInfo[1]
    tmpList = list(pubROMInfo[4:15])
    hexList = [hex(i) for i in tmpList]
    device_name = ''.join(chr(int(c, 16)) for c in hexList[0:])
    tmpList = list(pubROMInfo[16:20])
    hexList = [hex(i) for i in tmpList]
    device_type = ''.join(chr(int(c, 16)) for c in hexList[0:])
    dmsc_rom_version = list(pubROMInfo[20:24])
    dmsc_rom_version.reverse()
    r5_rom_version = list(pubROMInfo[24:28])
    r5_rom_version.reverse()

    # numBlocks
    print('-----------------------')
    print('SoC ID Header Info:')
    print('-----------------------')
    print("NumBlocks            :", numBlocks)

    # pubInfo
    print('-----------------------')
    print('SoC ID Public ROM Info:')
    print('-----------------------')
    print("SubBlockId           :", sub_block_id)
    print("SubBlockSize         :", sub_block_sz)
    print("DeviceName           :", device_name)
    print("DeviceType           :", device_type)
    print("DMSC ROM Version     :", dmsc_rom_version)
    print("R5 ROM Version       :", r5_rom_version)

    #secInfo is only applicable for HS devices
    if numBlocks > 1:
        secROMInfo = struct.unpack(secInfoStr, bin_str[32:200])
        sec_sub_block_id = secROMInfo[0]
        sec_sub_block_sz = secROMInfo[1]
        sec_prime = secROMInfo[2]
        sec_keyrev = secROMInfo[3]
        sec_keycount = secROMInfo[4]
        tmpList = list(secROMInfo[5:69])
        ti_mpk_hash = ''.join('{:02x}'.format(x) for x in tmpList)
        tmpList = list(secROMInfo[69:133])
        cust_mpk_hash = ''.join('{:02x}'.format(x) for x in tmpList)
        tmpList = list(secROMInfo[133:167])
        sec_unique_id = ''.join('{:02x}'.format(x) for x in tmpList)

        # secInfo
        print('-----------------------')
        print('SoC ID Secure ROM Info:')
        print('-----------------------')
        #print secROMInfo
        print("Sec SubBlockId       :", sec_sub_block_id)
        print("Sec SubBlockSize     :", sec_sub_block_sz)
        print("Sec Prime            :", sec_prime)
        print("Sec Key Revision     :", sec_keyrev)
        print("Sec Key Count        :", sec_keycount)
        print("Sec TI MPK Hash      :", ti_mpk_hash)
        print("Sec Cust MPK Hash    :", cust_mpk_hash)
        print("Sec Unique ID        :", sec_unique_id)

def modifySocId(str):
    ans  = str
    # If 6 leading 0s are there, remove 1
    if( str[5] == '0' ):
        ans = str[1:]
    return ans

def main(argv):
    parser = argparse.ArgumentParser(description=g_script_description)
    group = parser.add_mutually_exclusive_group(required=True)
    parser.add_argument('-d', '--device', required=True,
                    help='device for which the soc id will be parsed')
    group.add_argument('-s', '--string',
                    default=None, help='Soc id reported from UART console')
    group.add_argument('-f', '--file',
                    default=None ,help='path to the file which contains the soc id reported from UART console')
    args = parser.parse_args()

    device = args.device.lower()
    file = args.file
    str = args.string
    str_arr = []

    if str:
        if device == "am273x" :
            str = modifySocId(str)
        str_arr.append(str)
    elif file:
        try:
            print(file)
            fp = open(file, 'rt')
            str_arr = fp.readlines()
            if device == "am273x" :
                str_arr[0] = modifySocId(str_arr[0])
            fp.close
        except FileNotFoundError:
            print('[ERROR] File not Found !!')
            sys.exit(1)
    else:
        sys.exit(1)

    try:
        # Convert to byte array
        bin_arr = [ binascii.unhexlify(x.rstrip()) for x in str_arr ]
    except binascii.Error:
        print("[ERROR] Odd length String. Please copy the full string upto character 'C' ")
        sys.exit(1)

    bin_str = b"".join(bin_arr)

    if device in k3_devices:
        parser_k3_soc_info(bin_str)
    elif device in mcu_devices:
        parse_mcu_soc_info(bin_str)
    else:
        print('[ERROR] Unsupported Device Type !!')

if __name__ == "__main__":
    main(sys.argv[1:])
