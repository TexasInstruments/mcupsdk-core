import os
import sys
import platform
import json
from collections import deque
import serial.tools.list_ports as sp
from xmodem import XMODEM1k
import time
from random import randint

try:
    import uart_uniflash as uu
except ImportError:
    print("[ERROR] You need to run this from the same directory where uart_uniflash.py is")
    sys.exit()

try:
    from PyQt5 import QtCore, QtGui, QtWidgets
except ImportError:
    print("[ERROR] You need to install PyQt5 for this GUI to work")
    print("")
    if platform.system == "Windows":
        print("You can install PyQt5 by doing python -m pip install pyqt5")
    elif platform.system == "Linux":
        print("If you're on Ubuntu/Debian based distros, you can install PyQt5 by doing sudo apt install python3-pyqt5")
    elif platform.system == "Darwin":
        print("You can install PyQt5 by doing python3 -m pip install pyqt5")
    sys.exit()

gClipBoard = None

class Flasher(QtCore.QObject):
    xferDone     = QtCore.pyqtSignal(int)
    progress     = QtCore.pyqtSignal(int)
    logger       = QtCore.pyqtSignal(str)
    pBarInit     = QtCore.pyqtSignal()
    pBarDisable  = QtCore.pyqtSignal()
    labelSet     = QtCore.pyqtSignal(str)

    def setup(self, cfgFile, serialPort, baudRate=115200):
        self.serialPort = serialPort
        self.baudRate = baudRate
        self.cfgFile = cfgFile

    def xmodemSendReceive(self, fname, getResponse=True):
        # Some handy variables
        self.pBarCurSize = 0
        self.pBarTotalSize = 0

        status = False
        timetaken = 0
        
        # Since file is selected in GUI, there is very feeble chance it is not present. But just to leave no 'path' unchecked :)
        if not os.path.exists(fname):
            self.logger.emit(f'[ERROR] File [{fname}]] not found !!!')
            return -1, -1
        
        self.pBarTotalSize = os.path.getsize(fname)

        stream = open(fname, 'rb')
        ser = uu.open_serial_port(self.serialPort, self.baudRate)

        def getc(size, timeout=1):
            return ser.read(size) or None

        def putc(data, timeout=1):
            self.pBarCurSize += len(data)
            self.progress.emit(round((self.pBarCurSize * 100)/self.pBarTotalSize))
            return ser.write(data)

        self.pBarInit.emit()
        self.labelSet.emit(f'Sending {fname.replace(uu.TMP_SUFFIX, "")}')

        modem = XMODEM1k(getc, putc)
        tstart = time.time()
        status = modem.send(stream, quiet=True, timeout=10, retry=10)
        tstop = time.time()
        timetaken = round(tstop-tstart, 2)

        self.pBarDisable.emit()
        self.labelSet.emit('')
        stream.close()
        
        # Don't do the receive if get_response is False
        resp_status = 0
        if status:
            if getResponse:
                respfilename = "resp.dat"
                try:
                    respfile = open(respfilename, "wb")
                    status = modem.recv(respfile, quiet=True, timeout=2000)
                    respfile.close()
                    resp_status = uu.parse_response_evm(respfilename)
                    os.remove(respfilename)
                except:
                    self.logger.emit("\n[ERROR] XMODEM recv failed, no response OR incorrect response from EVM OR cancelled by user,")
                    self.logger.emit("Power cycle EVM and run this script again !!!")
                    self.xferDone.emit(-2)
        else:
            self.logger.emit("\n[ERROR] XMODEM send failed, no response OR incorrect response from EVM OR cancelled by user,")
            self.logger.emit("Power cycle EVM and run this script again !!!")
            self.xferDone.emit(-3)

        uu.close_serial_port(ser)

        if "ERROR" in str(resp_status):
            self.xferDone.emit(-4)

        return resp_status, timetaken

    def sendFileByParts(self, lCfg):
        orig_f_name = lCfg.filename
        orig_offset = lCfg.offset

        f = open(orig_f_name, "rb")
        f_bytes = f.read()
        f.close()

        num_parts   = int(len(f_bytes) / uu.BOOTLOADER_UNIFLASH_BUF_SIZE)
        remain_size = len(f_bytes) % uu.BOOTLOADER_UNIFLASH_BUF_SIZE
        total_time_taken = 0


        for i in range(0, num_parts):

            start = i*uu.BOOTLOADER_UNIFLASH_BUF_SIZE
            end = start+uu.BOOTLOADER_UNIFLASH_BUF_SIZE

            part_data = f_bytes[start:end]
            part_filename = orig_f_name + f".part{i+1}"
            # make the partial file
            f = open(part_filename, "wb")
            f.write(part_data)
            f.close()

            # temporarily change this to the partial filename
            lCfg.filename = part_filename
            lCfg.offset = hex(uu.get_numword(orig_offset) + i*uu.BOOTLOADER_UNIFLASH_BUF_SIZE)

            # send the partial file normally
            tempfilename = uu.create_temp_file(lCfg)
            status, timetaken = self.xmodemSendReceive(tempfilename)
            total_time_taken += timetaken

            # delete the temporary file
            os.remove(part_filename)
            os.remove(tempfilename)

        # Send the last part, if there were residual bytes
        if(remain_size > 0):
            start = num_parts*uu.BOOTLOADER_UNIFLASH_BUF_SIZE
            end = -1 # Read till the end of original file

            part_data = f_bytes[start:end]
            part_filename = orig_f_name + f".part{num_parts+1}"

            # make the partial file
            f = open(part_filename, "wb")
            f.write(part_data)
            f.close()

            # temporarily change this to the partial filename
            lCfg.filename = part_filename
            lCfg.offset = hex(uu.get_numword(orig_offset) + num_parts*uu.BOOTLOADER_UNIFLASH_BUF_SIZE)

            # send the partial file normally
            tempfilename = uu.create_temp_file(lCfg)
            status, timetaken = self.xmodemSendReceive(tempfilename)
            total_time_taken += timetaken

            # delete the temporary file
            os.remove(part_filename)
            os.remove(tempfilename)

        # revert back the original filename and offset
        lCfg.filename = orig_f_name
        lCfg.offset = orig_offset

        return status, total_time_taken
    
    def run(self):
        fileCfg = uu.FileCfg(self.cfgFile)
        parseStatus = fileCfg.parse()

        if parseStatus != 0:
            self.xferDone.emit(-1) # File parse error
            return None
        else:
            # Proceed to flash
            self.logger.emit(f"[INFO] Parsing config file ... SUCCESS. Found {len(fileCfg.cfgs)} command(s) !!!\n")
            if(fileCfg.flash_writer_index != None):
                # Found flash writer, flash it
                cfg_flash_writer_file = fileCfg.cfgs[fileCfg.flash_writer_index].flashwriter
                self.logger.emit(f"[INFO] Executing command 1 of {len(fileCfg.cfgs)} ...")
                self.logger.emit(f"[INFO] Found flash writer ... sending {cfg_flash_writer_file}")
                status, timetaken = self.xmodemSendReceive(cfg_flash_writer_file, getResponse=False)
                self.logger.emit(f"[INFO] Sent flashwriter {cfg_flash_writer_file} of size {os.path.getsize(cfg_flash_writer_file)} bytes in {timetaken}s.\n") 

            # loop through cfgs and lines, skip the process for flashwriter
            for i in range(0, len(fileCfg.cfgs)):
                if(i != fileCfg.flash_writer_index):
                    line = fileCfg.lines[i]
                    linecfg = fileCfg.cfgs[i]
                    self.logger.emit(f"[INFO] Executing command {i+1} of {len(fileCfg.cfgs)} ...")
                    ll_temp = line.rstrip('\n')
                    self.logger.emit(f"[INFO] Command arguments : {ll_temp}")
                    # Check if the size of application image is larger than buffer size in target side.
                    f_size = 0
                    if linecfg.filename is not None:
                        f_size = os.path.getsize(linecfg.filename)
                    
                    if((f_size + uu.BOOTLOADER_UNIFLASH_HEADER_SIZE >= uu.BOOTLOADER_UNIFLASH_BUF_SIZE) and (linecfg.optype in ["flash", "flashverify"])):
                        # Send by parts
                        self.logger.emit(f"[INFO] Size of the file {linecfg.filename} is more than receive buffer size, sending by parts ...")
                        status, timetaken = self.sendFileByParts(linecfg)
                    else:
                        # Send normally
                        tempfilename = uu.create_temp_file(linecfg)
                        status, timetaken = self.xmodemSendReceive(tempfilename, getResponse=True)

                    orig_filename = linecfg.filename
                    if(linecfg.optype == "erase"):
                        self.logger.emit("Sent flash erase command.")
                    elif(linecfg.optype == "flash-phy-tuning-data"):
                        self.logger.emit(f"Sent flash phy tuning data in {timetaken}s.")
                    else:
                        self.logger.emit(f"Sent {orig_filename} of size {os.path.getsize(orig_filename)} bytes in {timetaken}s.")
                    self.logger.emit(status)
                    # Delete the tempfile if it exists
                    if(os.path.exists(tempfilename)):
                        os.remove(tempfilename)

            self.logger.emit("All commands from config file are executed !!!")
            self.xferDone.emit(0)

class DropDownHistory():
    def __init__(self, histfile, keylist, histlim=10):
        self.histfile = histfile
        self.histlim = histlim
        self.keylist = keylist

        self.histdict = dict()
    
    # Load as dictionary of deques
    def load(self):
        if not os.path.exists(self.histfile):
            # nothing to load, create file with empty lists
            temp_hist_dict = dict()
            for key in self.keylist:
                temp_hist_dict[key] = [' ']
            with open(self.histfile, "w") as histfh:
                json.dump(temp_hist_dict, histfh)
        else:
            pass
        hist_dict = None
        with open(self.histfile, "r") as histfh:
            hist_dict = json.load(histfh)
        for key in hist_dict.keys():
            if key in self.keylist:
                self.histdict[key] = deque(hist_dict[key], maxlen=self.histlim)
    
    # Save as dictionary of lists
    def save(self):
        temp_hist_dict = dict()
        for key in self.histdict.keys():
            temp_hist_dict[key] = list(self.histdict[key])
        with open(self.histfile, "w") as histfh:
            json.dump(temp_hist_dict, histfh)
    
    def add_to_history(self, key, s):
        if s not in self.histdict[key]:
            if key in self.keylist:
                self.histdict[key].appendleft(s)

    def get_hist_items(self, key):
        if key in self.histdict.keys():
            return list(self.histdict[key])
        else:
            return None

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(795, 720)
        MainWindow.setFixedSize(795, 720)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.logTextBox = QtWidgets.QPlainTextEdit(self.centralwidget)
        self.logTextBox.setGeometry(QtCore.QRect(12, 510, 771, 70))
        self.logTextBox.setObjectName("logTextBox")
        self.manualConfigGroupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.manualConfigGroupBox.setEnabled(True)
        self.manualConfigGroupBox.setGeometry(QtCore.QRect(36, 70, 721, 301))
        self.manualConfigGroupBox.setCheckable(True)
        self.manualConfigGroupBox.setChecked(False)
        self.manualConfigGroupBox.setObjectName("manualConfigGroupBox")
        self.bootloaderBrowseButton = QtWidgets.QPushButton(self.manualConfigGroupBox)
        self.bootloaderBrowseButton.setGeometry(QtCore.QRect(414, 84, 89, 25))
        self.bootloaderBrowseButton.setObjectName("bootloaderBrowseButton")
        self.appimageXIPBinLabel = QtWidgets.QLabel(self.manualConfigGroupBox)
        self.appimageXIPBinLabel.setGeometry(QtCore.QRect(14, 184, 151, 17))
        self.appimageXIPBinLabel.setObjectName("appimageXIPBinLabel")
        self.appimageXIPBrowseButton = QtWidgets.QPushButton(self.manualConfigGroupBox)
        self.appimageXIPBrowseButton.setGeometry(QtCore.QRect(414, 184, 89, 25))
        self.appimageXIPBrowseButton.setObjectName("appimageXIPBrowseButton")
        self.flashWrBinLabel = QtWidgets.QLabel(self.manualConfigGroupBox)
        self.flashWrBinLabel.setGeometry(QtCore.QRect(14, 34, 131, 17))
        self.flashWrBinLabel.setObjectName("flashWrBinLabel")
        self.customDataBrowseButton = QtWidgets.QPushButton(self.manualConfigGroupBox)
        self.customDataBrowseButton.setGeometry(QtCore.QRect(414, 234, 89, 25))
        self.customDataBrowseButton.setObjectName("customDataBrowseButton")
        self.bootloaderBinLabel = QtWidgets.QLabel(self.manualConfigGroupBox)
        self.bootloaderBinLabel.setGeometry(QtCore.QRect(14, 84, 131, 17))
        self.bootloaderBinLabel.setObjectName("bootloaderBinLabel")
        self.appimageBinLabel = QtWidgets.QLabel(self.manualConfigGroupBox)
        self.appimageBinLabel.setGeometry(QtCore.QRect(14, 134, 131, 17))
        self.appimageBinLabel.setObjectName("appimageBinLabel")
        self.customDataLabel = QtWidgets.QLabel(self.manualConfigGroupBox)
        self.customDataLabel.setGeometry(QtCore.QRect(14, 234, 151, 17))
        self.customDataLabel.setObjectName("customDataLabel")
        self.flashWrBrowseButton = QtWidgets.QPushButton(self.manualConfigGroupBox)
        self.flashWrBrowseButton.setGeometry(QtCore.QRect(414, 34, 89, 25))
        self.flashWrBrowseButton.setObjectName("flashWrBrowseButton")
        self.appimageBrowseButton = QtWidgets.QPushButton(self.manualConfigGroupBox)
        self.appimageBrowseButton.setGeometry(QtCore.QRect(414, 134, 89, 25))
        self.appimageBrowseButton.setObjectName("appimageBrowseButton")
        self.bootloaderBinaryOffsetLabel = QtWidgets.QLabel(self.manualConfigGroupBox)
        self.bootloaderBinaryOffsetLabel.setGeometry(QtCore.QRect(516, 88, 67, 17))
        self.bootloaderBinaryOffsetLabel.setObjectName("bootloaderBinaryOffsetLabel")
        self.bootloaderBinaryOffsetLineEdit = QtWidgets.QLineEdit(self.manualConfigGroupBox)
        self.bootloaderBinaryOffsetLineEdit.setGeometry(QtCore.QRect(596, 84, 113, 25))
        self.bootloaderBinaryOffsetLineEdit.setObjectName("bootloaderBinaryOffsetLineEdit")
        self.appimageBinaryOffsetLineEdit = QtWidgets.QLineEdit(self.manualConfigGroupBox)
        self.appimageBinaryOffsetLineEdit.setGeometry(QtCore.QRect(594, 132, 113, 25))
        self.appimageBinaryOffsetLineEdit.setObjectName("appimageBinaryOffsetLineEdit")
        self.appimageBinaryLabel = QtWidgets.QLabel(self.manualConfigGroupBox)
        self.appimageBinaryLabel.setGeometry(QtCore.QRect(514, 136, 67, 17))
        self.appimageBinaryLabel.setObjectName("appimageBinaryLabel")
        self.customDataOffsetLineEdit = QtWidgets.QLineEdit(self.manualConfigGroupBox)
        self.customDataOffsetLineEdit.setGeometry(QtCore.QRect(594, 234, 113, 25))
        self.customDataOffsetLineEdit.setText("")
        self.customDataOffsetLineEdit.setObjectName("customDataOffsetLineEdit")
        self.customDataBinaryLabel = QtWidgets.QLabel(self.manualConfigGroupBox)
        self.customDataBinaryLabel.setGeometry(QtCore.QRect(514, 238, 67, 17))
        self.customDataBinaryLabel.setObjectName("customDataBinaryLabel")
        self.flashwriterFileCBox = QtWidgets.QComboBox(self.manualConfigGroupBox)
        self.flashwriterFileCBox.setGeometry(QtCore.QRect(162, 34, 253, 25))
        self.flashwriterFileCBox.setEditable(True)
        self.flashwriterFileCBox.setObjectName("flashwriterFileCBox")
        self.bootloaderBinaryFileCBox = QtWidgets.QComboBox(self.manualConfigGroupBox)
        self.bootloaderBinaryFileCBox.setGeometry(QtCore.QRect(162, 84, 253, 25))
        self.bootloaderBinaryFileCBox.setEditable(True)
        self.bootloaderBinaryFileCBox.setObjectName("bootloaderBinaryFileCBox")
        self.appimageBinCBox = QtWidgets.QComboBox(self.manualConfigGroupBox)
        self.appimageBinCBox.setGeometry(QtCore.QRect(162, 134, 253, 25))
        self.appimageBinCBox.setEditable(True)
        self.appimageBinCBox.setObjectName("appimageBinCBox")
        self.appimageXIPBinCBox = QtWidgets.QComboBox(self.manualConfigGroupBox)
        self.appimageXIPBinCBox.setGeometry(QtCore.QRect(162, 184, 253, 25))
        self.appimageXIPBinCBox.setEditable(True)
        self.appimageXIPBinCBox.setObjectName("appimageXIPBinCBox")
        self.customDataBinaryCBox = QtWidgets.QComboBox(self.manualConfigGroupBox)
        self.customDataBinaryCBox.setGeometry(QtCore.QRect(162, 234, 253, 25))
        self.customDataBinaryCBox.setEditable(True)
        self.customDataBinaryCBox.setObjectName("customDataBinaryCBox")
        self.phyTuningDataCheckBox = QtWidgets.QCheckBox(self.manualConfigGroupBox)
        self.phyTuningDataCheckBox.setGeometry(QtCore.QRect(10, 272, 175, 23))
        self.phyTuningDataCheckBox.setObjectName("phyTuningDataCheckBox")
        self.saveCfgButton = QtWidgets.QPushButton(self.manualConfigGroupBox)
        self.saveCfgButton.setGeometry(QtCore.QRect(594, 272, 113, 25))
        self.saveCfgButton.setCheckable(False)
        self.saveCfgButton.setObjectName("saveCfgButton")
        self.autoConfigGroupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.autoConfigGroupBox.setEnabled(True)
        self.autoConfigGroupBox.setGeometry(QtCore.QRect(38, 386, 721, 71))
        self.autoConfigGroupBox.setCheckable(True)
        self.autoConfigGroupBox.setObjectName("autoConfigGroupBox")
        self.configFileBrowseButton = QtWidgets.QPushButton(self.autoConfigGroupBox)
        self.configFileBrowseButton.setGeometry(QtCore.QRect(416, 34, 89, 25))
        self.configFileBrowseButton.setObjectName("configFileBrowseButton")
        self.configFileLabel = QtWidgets.QLabel(self.autoConfigGroupBox)
        self.configFileLabel.setGeometry(QtCore.QRect(16, 38, 131, 17))
        self.configFileLabel.setObjectName("configFileLabel")
        self.configFileCBox = QtWidgets.QComboBox(self.autoConfigGroupBox)
        self.configFileCBox.setGeometry(QtCore.QRect(164, 34, 253, 25))
        self.configFileCBox.setEditable(True)
        self.configFileCBox.setObjectName("configFileCBox")
        self.flashPushButton = QtWidgets.QPushButton(self.centralwidget)
        self.flashPushButton.setGeometry(QtCore.QRect(322, 462, 113, 45))
        self.flashPushButton.setObjectName("flashPushButton")
        self.mainProgressBar = QtWidgets.QProgressBar(self.centralwidget)
        self.mainProgressBar.setEnabled(True)
        self.mainProgressBar.setGeometry(QtCore.QRect(12, 656, 767, 31))
        self.mainProgressBar.setProperty("value", 0)
        self.mainProgressBar.setTextVisible(True)
        self.mainProgressBar.setObjectName("mainProgressBar")
        self.progressBarLabel = QtWidgets.QLabel(self.centralwidget)
        self.progressBarLabel.setGeometry(QtCore.QRect(14, 624, 763, 17))
        self.progressBarLabel.setText("")
        self.progressBarLabel.setObjectName("progressBarLabel")
        self.clearLogButton = QtWidgets.QPushButton(self.centralwidget)
        self.clearLogButton.setGeometry(QtCore.QRect(606, 578, 89, 25))
        self.clearLogButton.setCheckable(False)
        self.clearLogButton.setObjectName("clearLogButton")
        self.copyLogButton = QtWidgets.QPushButton(self.centralwidget)
        self.copyLogButton.setGeometry(QtCore.QRect(694, 578, 89, 25))
        self.copyLogButton.setCheckable(False)
        self.copyLogButton.setObjectName("copyLogButton")
        self.serialPortCBox = QtWidgets.QComboBox(self.centralwidget)
        self.serialPortCBox.setGeometry(QtCore.QRect(356, 22, 181, 25))
        self.serialPortCBox.setEditable(True)
        self.serialPortCBox.setObjectName("serialPortCBox")
        self.uartSerialPortLabel = QtWidgets.QLabel(self.centralwidget)
        self.uartSerialPortLabel.setGeometry(QtCore.QRect(214, 26, 131, 17))
        self.uartSerialPortLabel.setObjectName("uartSerialPortLabel")
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MCU+SDK UART Uniflash Tool"))
        self.manualConfigGroupBox.setTitle(_translate("MainWindow", "Manual Configuration"))
        self.bootloaderBrowseButton.setText(_translate("MainWindow", "Browse"))
        self.appimageXIPBinLabel.setText(_translate("MainWindow", "Appimage XIP binary"))
        self.appimageXIPBrowseButton.setText(_translate("MainWindow", "Browse"))
        self.flashWrBinLabel.setText(_translate("MainWindow", "Flash Writer Binary"))
        self.customDataBrowseButton.setText(_translate("MainWindow", "Browse"))
        self.bootloaderBinLabel.setText(_translate("MainWindow", "Bootloader Binary"))
        self.appimageBinLabel.setText(_translate("MainWindow", "Appimage binary"))
        self.customDataLabel.setText(_translate("MainWindow", "Custom Data"))
        self.flashWrBrowseButton.setText(_translate("MainWindow", "Browse"))
        self.appimageBrowseButton.setText(_translate("MainWindow", "Browse"))
        self.bootloaderBinaryOffsetLabel.setText(_translate("MainWindow", "Offset"))
        self.bootloaderBinaryOffsetLineEdit.setText(_translate("MainWindow", "0x0000"))
        self.appimageBinaryOffsetLineEdit.setText(_translate("MainWindow", "0x80000"))
        self.appimageBinaryLabel.setText(_translate("MainWindow", "Offset"))
        self.customDataBinaryLabel.setText(_translate("MainWindow", "Offset"))
        self.phyTuningDataCheckBox.setText(_translate("MainWindow", "Flash PHY tuning data "))
        self.saveCfgButton.setText(_translate("MainWindow", "Save CFG"))
        self.autoConfigGroupBox.setTitle(_translate("MainWindow", "From File"))
        self.configFileBrowseButton.setText(_translate("MainWindow", "Browse"))
        self.configFileLabel.setText(_translate("MainWindow", "Configuration File"))
        self.flashPushButton.setText(_translate("MainWindow", "FLASH"))
        self.clearLogButton.setText(_translate("MainWindow", "Clear Log"))
        self.copyLogButton.setText(_translate("MainWindow", "Copy Log"))
        self.uartSerialPortLabel.setText(_translate("MainWindow", "UART Serial Port"))
        self.customUartUniflashSetup()

#############################################################################################################################################################################################################
    def popInfoMbox(self, boxTitle, boxMsg):
        self.infoMbox.setWindowTitle(boxTitle)
        self.infoMbox.setText(boxMsg)
        self.infoMbox.exec_()

    def popErrorMbox(self, boxTitle, boxMsg):
        self.errorMbox.setWindowTitle(boxTitle)
        self.errorMbox.setText(boxMsg)
        self.errorMbox.exec_()
    
    def guiPrintln(self, s):
        self.logData += (s+'\n')
        self.logTextBox.setPlainText(self.logData)
    
    def logClear(self):
        self.logData = ""
        self.logTextBox.setPlainText(self.logData)

    def logCopy(self):
        gClipBoard.setText(self.logData, mode=gClipBoard.Clipboard)

    def flashWrClicked(self):
        # Invoked when flash writer file browse button is clicked
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        fileName, _ = QtWidgets.QFileDialog.getOpenFileName(None, "Select the SBL Uart Uniflash binary", "", "Bootimage Files (*.tiimage);;All Files (*)", options=options)

        if fileName:
            self.flashwriterFileCBox.setCurrentText(fileName)
            self.flashwriterFileCBox.addItem(fileName)
            # Update history
            self.dropDownHist.add_to_history('flashwriter', fileName)
            self.dropDownHist.save()
            self.guiPrintln(f"[INFO] Selected SBL Uart Uniflash binary : {fileName}")

    def bootloaderClicked(self):
        # Invoked when bootloader file browse button is clicked
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        fileName, _ = QtWidgets.QFileDialog.getOpenFileName(None, "Select the OSPI/QSPI bootloader binary", "", "Bootimage Files (*.tiimage);;All Files (*)", options=options)

        if fileName:
            self.bootloaderBinaryFileCBox.setCurrentText(fileName)
            self.bootloaderBinaryFileCBox.addItem(fileName)
            # Update history
            self.dropDownHist.add_to_history('bootloader', fileName)
            self.dropDownHist.save()
            self.guiPrintln(f"[INFO] Selected SBL OSPI/QSPI binary : {fileName}")

    def appimageBinClicked(self):
        # Invoked when appimage file browse button is clicked
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        fileName, _ = QtWidgets.QFileDialog.getOpenFileName(None, "Select the appimage binary", "", "Appimage Files (*.appimage *.appimage.hs);;All Files (*)", options=options)

        if fileName:
            self.appimageBinCBox.setCurrentText(fileName)
            self.appimageBinCBox.addItem(fileName)
            # Update history
            self.dropDownHist.add_to_history('appimage', fileName)
            self.dropDownHist.save()
            self.guiPrintln(f"[INFO] Selected appimage binary : {fileName}")

    def appimageXIPBinClicked(self):
        # Invoked when appimage xip file browse button is clicked
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        fileName, _ = QtWidgets.QFileDialog.getOpenFileName(None, "Select the appimage XIP binary", "", "Appimage XIP Files (*.appimage_xip);;All Files (*)", options=options)

        if fileName:
            self.appimageXIPBinCBox.setCurrentText(fileName)
            self.appimageXIPBinCBox.addItem(fileName)
            # Update history
            self.dropDownHist.add_to_history('appimage_xip', fileName)
            self.dropDownHist.save()
            self.guiPrintln(f"[INFO] Selected appimage XIP binary : {fileName}")

    def customDataClicked(self):
        # Invoked when custom data browse button is clicked
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        fileName, _ = QtWidgets.QFileDialog.getOpenFileName(None, "Select file to be flashed", "", "All Files (*)", options=options)

        if fileName:
            self.customDataBinaryCBox.setCurrentText(fileName)
            self.customDataBinaryCBox.addItem(fileName)
            # Update history
            self.dropDownHist.add_to_history('custom_data', fileName)
            self.dropDownHist.save()
            self.guiPrintln(f"[INFO] Selected custom file to flash: {fileName}")
    
    def configFileBrowseClicked(self):
        # Invoked when custom data browse button is clicked
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        fileName, _ = QtWidgets.QFileDialog.getOpenFileName(None, "Select file to be flashed", "", "CFG Files (*.cfg);;All Files (*)", options=options)

        if fileName:
            self.configFileCBox.setCurrentText(fileName)
            self.configFileCBox.addItem(fileName)
            # Update history
            self.dropDownHist.add_to_history('config_file', fileName)
            self.dropDownHist.save()
            self.guiPrintln(f"[INFO] Selected config file: {fileName}")
    
    def setConfigBoxManual(self):
        self.manualConfigGroupBox.setChecked(not self.autoConfigGroupBox.isChecked())

    def setConfigBoxAuto(self):
        self.autoConfigGroupBox.setChecked(not self.manualConfigGroupBox.isChecked())

    def saveManualConfig(self):
        # Flash writer
        flashwr = self.flashwriterFileCBox.currentText().strip()
        # Bootloader
        bootloader = self.bootloaderBinaryFileCBox.currentText().strip()
        # Appimage
        appimage = self.appimageBinCBox.currentText().strip()
        # Appimage XIP
        appimageXip = self.appimageXIPBinCBox.currentText().strip()
        # Custom data
        customFile = self.customDataBinaryCBox.currentText().strip()
        
        fileData = ""

        if flashwr not in (None, '', ' ', '\n'):
            fileData += "# First point to sbl_uart_uniflash binary, which functions as a server to flash one or more files\n" 
            fileData += f"--flash-writer={flashwr}\n"
            fileData += "\n"
        else:
            # Raise info box, maybe they have already flashed flash writer
            self.popInfoMbox("File Save Info", "You have opted not to include flashwriter in your config file")
        
        if self.phyTuningDataCheckBox.isChecked():
            fileData += "# Program the OSPI PHY tuning attack vector\n"
            fileData += "--operation=flash-phy-tuning-data\n"
            fileData += "\n"
        
        if bootloader not in (None, '', ' ', '\n'):
            if self.bootloaderBinaryOffsetLineEdit.text() in (None, '', ' '):
                # Raise message box error, no offset given to flash
                self.popErrorMbox("File Save Error", "Bootloader to be flashed is selected, but no valid offset given!")
                self.guiPrintln("[INFO] Please provide the offset at which the bootloader is to be flashed")
                return None
            fileData += "# When sending bootloader make sure to flash at offset 0x0. ROM expects bootloader at offset 0x0\n"
            fileData += f"--file={bootloader} --operation=flash --flash-offset={self.bootloaderBinaryOffsetLineEdit.text()}\n"
            fileData += "\n"
        
        if appimage not in (None, '', ' ', '\n'):
            if self.bootloaderBinaryOffsetLineEdit.text() in (None, '', ' '):
                # Raise message box error, no offset given to flash
                self.popErrorMbox("File Save Error", "Appimage to be flashed is selected, but no valid offset given!")
                self.guiPrintln("[INFO] Please provide the offset at which the bootloader is to be flashed")
                return None
            fileData += "# When sending application image, make sure to flash at offset 0x80000 (default) or to whatever offset your bootloader is configured for\n"
            fileData += f"--file={appimage} --operation=flash --flash-offset={self.appimageBinaryOffsetLineEdit.text()}\n"
            fileData += "\n"

        if appimageXip not in (None, '', ' ', '\n'):
            fileData += "# Send the XIP image for this application, no need to specify flash offset since flash offset is specified within the image itself\n"
            fileData += f"--file={appimageXip} --operation=flash-xip\n"
            fileData += "\n"

        if customFile not in (None, '', ' ', '\n'):
            if self.customDataOffsetLineEdit.text() in (None, '', ' '):
                # Raise message box error, no offset given to flash
                self.popErrorMbox("File Save Error", "Custom file to be flashed is selected, but no valid offset given!")
                self.guiPrintln("[INFO] Please provide the offset at which the custom file is to be flashed")
                return None
            fileData += "# # When sending application image, make sure to flash at offset 0x80000 (default) or to whatever offset your bootloader is configured for\n"
            fileData += f"--file={appimage} --operation=flash --flash-offset={self.customDataOffsetLineEdit.text()}\n"
            fileData += "\n"

        # Everything validated, if there is fileData to be written, open file dialog and write to selected file.
        if fileData == '':
            self.popInfoMbox("File Save Info", "Nothing to be saved!")
        else:
            options = QtWidgets.QFileDialog.Options()
            options |= QtWidgets.QFileDialog.DontUseNativeDialog
            fileName, _ = QtWidgets.QFileDialog.getSaveFileName(None, "Select file to save the config to", "", "Config Files (*.cfg);;All Files (*)", options=options)

            fileData += "\n"
            if fileName is not None:
                with open(fileName, "w") as f:
                    f.write(fileData)
                self.guiPrintln(f"[INFO] Saved config to file {fileName}")

    # Signal callbacks
    def labelSetCb(self, s):
        self.progressBarLabel.setText(s)

    def pBarInitCb(self):
        self.mainProgressBar.setHidden(False)
        self.mainProgressBar.setValue(0)

    def pBarDisableCb(self):
        self.mainProgressBar.setHidden(True)

    def loggerCb(self, s):
        self.guiPrintln(s)

    def progressCb(self, val):
        self.mainProgressBar.setValue(val)

    def flashStatusCb(self, val):
        if val == 0:
            # Transfer success
            self.popInfoMbox("Flash Status", "Flashing successfully completed !")
        elif val == -1:
            # Parse Error
            self.popErrorMbox("Flash Status", "Flashing failed: Error in parsing config file !!!")
        elif val == -2:
            # Recv Error
            self.popErrorMbox("Flash Status", "Flashing failed: XMODEM receive error !!!")
        elif val == -3:
            # Send Error
            self.popErrorMbox("Flash Status", "Flashing failed: XMODEM send error !!!")
        elif val == -4:
            # Send Error
            self.popErrorMbox("Flash Status", "Flashing failed: Error response from target. Check logs for details !!!")
        else:
            # Unrecognized val
            self.popErrorMbox("Flash Status", "Unknown error.")
    
    def removeTempFileIfRequired(self):
        if os.path.exists(self.tempFileName):
            os.remove(self.tempFileName)
    
    def flashFromConfigFile(self, fname):
        self.thread = QtCore.QThread()
        self.flasher = Flasher()
        self.flasher.setup(fname, self.serialPortCBox.currentText().strip(), baudRate=115200)
        self.flasher.moveToThread(self.thread)

        self.flasher.labelSet.connect(self.labelSetCb)
        self.flasher.pBarInit.connect(self.pBarInitCb)
        self.flasher.pBarDisable.connect(self.pBarDisableCb)
        self.flasher.logger.connect(self.loggerCb)
        self.flasher.progress.connect(self.progressCb)
        self.flasher.xferDone.connect(self.thread.quit)
        self.flasher.xferDone.connect(self.flasher.deleteLater)
        self.flasher.xferDone.connect(self.flashStatusCb)
        self.thread.finished.connect(self.thread.deleteLater)
        self.thread.started.connect(self.flasher.run)

        self.thread.start()

        # Disable the flash button until flashing is complete
        self.flashPushButton.setEnabled(False)
        self.thread.finished.connect(lambda: self.flashPushButton.setEnabled(True))
        self.thread.finished.connect(lambda: self.removeTempFileIfRequired)

        
    def flash(self):
        # No need to check for serial port as it will be selected from SYSTEM data
        # Check if manual config or config file
        if self.autoConfigGroupBox.isChecked():
            # Config file
            self.guiPrintln("[INFO] Selected config file based flashing, parsing config file ...")
            configFileName = self.configFileCBox.currentText().strip()
            if configFileName in (None, '', ' ', '\n'):
                self.popErrorMbox("Flash Error", "Config file mode selected, but no config file provided !!")
                return None
            else:
                self.flashFromConfigFile(self.configFileCBox.currentText().strip())
        else:
            # Manual config
            flashwr = self.flashwriterFileCBox.currentText().strip()
            bootloader = self.bootloaderBinaryFileCBox.currentText().strip()
            appimage = self.appimageBinCBox.currentText().strip()
            appimageXip = self.appimageXIPBinCBox.currentText().strip()
            customFile = self.customDataBinaryCBox.currentText().strip()

            cfgFileData = ''

            # Check for flash writer
            if flashwr in (None, '', ' ', '\n'):
                self.popInfoMbox("Flash Info", "You have not selected a flash writer binary. In this case it is assumed you already have the flash writer running")
            else:
                cfgFileData += f"--flash-writer={flashwr}\n"

            # Check for tuning data
            if self.phyTuningDataCheckBox.isChecked():
                cfgFileData += "--operation=flash-phy-tuning-data\n"
            
            # Check for bootloader
            if bootloader in (None, '', ' ', '\n'):
                self.popInfoMbox("Flash Info", "You have not selected a bootloader binary. Assuming this is intentional")
            else:
                if self.bootloaderBinaryOffsetLineEdit.text() in (None, '', ' '):
                    # Raise message box error, no offset given to flash
                    self.popErrorMbox("Flash Error", "Custom file to be flashed is selected, but no valid offset given!")
                    self.guiPrintln("[INFO] Please provide the offset at which the custom file is to be flashed")
                    return None
                cfgFileData += f"--file={bootloader} --operation=flash --flash-offset={self.bootloaderBinaryOffsetLineEdit.text()}\n"
            
            # Check for appimage
            if appimage in (None, '', ' ', '\n'):
                # self.popInfoMbox("Flash Info", "You have not selected a appimage binary. Assuming this is intentional")
                pass
            else:
                if self.appimageBinaryOffsetLineEdit.text() in (None, '', ' ', '\n'):
                    # Raise message box error, no offset given to flash
                    self.popErrorMbox("Flash Error", "Appimage file to be flashed is selected, but no valid offset given!")
                    self.guiPrintln("[INFO] Please provide the offset at which the appimage file is to be flashed")
                    return None
                cfgFileData += f"--file={appimage} --operation=flash --flash-offset={self.appimageBinaryOffsetLineEdit.text()}\n"

            # Check for XIP appimage
            if appimageXip in (None, '', ' ', '\n'):
                # self.popInfoMbox("Flash Info", "You have not selected a appimage XIP binary. Assuming this is intentional")
                pass
            else:
                cfgFileData += f"--file={appimageXip} --operation=flash-xip\n"

            # Check for custom file
            if customFile in (None, '', ' ', '\n'):
                # self.popInfoMbox("Flash Info", "You have not selected a custom file binary. Assuming this is intentional")
                pass
            else:
                if self.customDataOffsetLineEdit.text() in (None, '', ' ', '\n'):
                    # Raise message box error, no offset given to flash
                    self.popErrorMbox("Flash Error", "Appimage file to be flashed is selected, but no valid offset given!")
                    self.guiPrintln("[INFO] Please provide the offset at which the appimage file is to be flashed")
                    return None
                cfgFileData += f"--file={customFile} --operation=flash --flash-offset={self.customDataOffsetLineEdit.text()}\n"

            if cfgFileData != '':
                # Write to a temp file and flash using that
                tempfilename = str(os.path.join(os.path.expanduser('~'), f'temp_cfg_{str(randint(22222, 33333))}.cfg'))
                with open(tempfilename, "w") as f:
                    f.write(cfgFileData)
                self.flashFromConfigFile(tempfilename)
                self.tempFileName = tempfilename
            else:
                self.popInfoMbox("Flash Info", "Nothing to be flashed.")

    def customUartUniflashSetup(self):
        # Set Tooltips
        self.bootloaderBinaryOffsetLineEdit.setToolTip("Flash offset at which your bootloader needs to be written.\nUsually 0.")
        self.appimageBinaryOffsetLineEdit.setToolTip("Flash offset at which your application image needs to be written.\nUsually 0x80000 (512 KB offset from start).")
        self.customDataOffsetLineEdit.setToolTip("Flash offset at which your custom data needs to be written.\nGive a valid offset which is aligned with the flash's block size in hex (0x40000) or decimal (262144)")
        self.saveCfgButton.setToolTip("Saves the current manual configuration into a file")
        self.phyTuningDataCheckBox.setToolTip("Check this if you want to flash the OSPI phy tuning data into the flash as well")

        # Set a log data variable
        self.logData = ""

        # Set time and status variables to be updated by IO thread
        self.xmodemTimeSpent = 0
        self.xmodemStatus = False

        # Make log read-only
        self.logTextBox.setReadOnly(True)

        # Setup some message boxes
        self.infoMbox = QtWidgets.QMessageBox()
        self.infoMbox.setIcon(QtWidgets.QMessageBox.Information)
        self.infoMbox.setStandardButtons(QtWidgets.QMessageBox.Ok)

        self.errorMbox = QtWidgets.QMessageBox()
        self.errorMbox.setIcon(QtWidgets.QMessageBox.Critical)
        self.errorMbox.setStandardButtons(QtWidgets.QMessageBox.Ok)

        # Set up drop down history for all the drop downs -> bootloader, flashwriter, appimage, appimage_xip, custom_data, config_file
        keylist = ['flashwriter', 'bootloader', 'appimage', 'appimage_xip', 'custom_data', 'config_file']
        self.dropDownHist = DropDownHistory(str(os.path.join(os.path.expanduser('~'), '.uart_uniflash_drop_down_hist')), keylist)
        self.dropDownHist.load()

        # Set loaded history to the dropdown items
        if self.dropDownHist.get_hist_items('flashwriter') is not None:
            self.flashwriterFileCBox.addItems(self.dropDownHist.get_hist_items('flashwriter'))
        if self.dropDownHist.get_hist_items('bootloader') is not None:
            self.bootloaderBinaryFileCBox.addItems(self.dropDownHist.get_hist_items('bootloader'))
        if self.dropDownHist.get_hist_items('appimage') is not None:
            self.appimageBinCBox.addItems(self.dropDownHist.get_hist_items('appimage'))
        if self.dropDownHist.get_hist_items('appimage_xip') is not None:
            self.appimageXIPBinCBox.addItems(self.dropDownHist.get_hist_items('appimage_xip'))
        if self.dropDownHist.get_hist_items('custom_data') is not None:
            self.customDataBinaryCBox.addItems(self.dropDownHist.get_hist_items('custom_data'))
        if self.dropDownHist.get_hist_items('config_file') is not None:
            self.configFileCBox.addItems(self.dropDownHist.get_hist_items('config_file'))

        # Make the items word wrap
        self.flashwriterFileCBox.view().setWordWrap(True)
        self.bootloaderBinaryFileCBox.view().setWordWrap(True)
        self.appimageBinCBox.view().setWordWrap(True)
        self.appimageXIPBinCBox.view().setWordWrap(True)
        self.customDataBinaryCBox.view().setWordWrap(True)
        self.configFileCBox.view().setWordWrap(True)
        self.serialPortCBox.view().setWordWrap(True)

        # Set file dialog boxes to open when browse button is clicked
        self.bootloaderBrowseButton.clicked.connect(self.bootloaderClicked)
        self.flashWrBrowseButton.clicked.connect(self.flashWrClicked)
        self.appimageBrowseButton.clicked.connect(self.appimageBinClicked)
        self.appimageXIPBrowseButton.clicked.connect(self.appimageXIPBinClicked)
        self.customDataBrowseButton.clicked.connect(self.customDataClicked)
        self.configFileBrowseButton.clicked.connect(self.configFileBrowseClicked)

        # Set clear log and copy log buttons
        self.clearLogButton.clicked.connect(self.logClear)
        self.copyLogButton.clicked.connect(self.logCopy)

        # Make groupboxes mutually exclusive
        self.autoConfigGroupBox.toggled.connect(self.setConfigBoxManual)
        self.manualConfigGroupBox.toggled.connect(self.setConfigBoxAuto)

        # Hide the progress bar. It should be unhided and hided back, only during a transfer
        self.mainProgressBar.setHidden(True)

        # Set serial port list
        self.serialPortCBox.addItems([i.device for i in sp.comports()])

        # Attach save config button to save manual config method
        self.saveCfgButton.clicked.connect(self.saveManualConfig)

        # Attach the flash processing method to flash button
        self.flashPushButton.clicked.connect(self.flash)

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    gClipBoard = app.clipboard()
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

