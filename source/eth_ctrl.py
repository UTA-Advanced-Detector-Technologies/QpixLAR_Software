# interfacing dependcies
from eth_interface import (eth_interface, EthBadAddr, REG, DEFAULT_PACKET_SIZE, ZYBO_FRQ)
import os
import sys
import time

# PyQt GUI things
from PyQt5 import QtCore
from PyQt5.QtWidgets import (QWidget, QPushButton, QCheckBox, QComboBox, QSpinBox, QLabel,
                             QDoubleSpinBox, QProgressBar, QTabWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QStatusBar,
                             QDialog, QDialogButtonBox, QLCDNumber, QFileDialog)
from PyQt5.QtCore import QProcess, QTimer, pyqtSignal
from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import QApplication, QMainWindow, QAction

from qda_reg import QDA_CTRL_REG


class GUI(QMainWindow):

    close_udp = pyqtSignal()

    def __init__(self):
        super(QMainWindow, self).__init__()

        # IO interfaces
        self.eth = eth_interface()
        self.close_udp.connect(self.eth.finish) # closes udp worker thread
        self.version = self.eth.version

        # window setup
        self.setWindowTitle('Ethernet Control')

        # passive triggering
        self._clock = QTimer()
        self._clock.timeout.connect(self.trigger)
        self._lastTrig = -1

        # initialize the sub menus
        self._make_menuBar()

        # create the layouts that are needed for making the GUI pretty
        self.tabW = QTabWidget()
        self.tabW.addTab(self._makeEthlayout(), "ETH")

        self.tabW.addTab(self._makeTestlayout(), "TEST")

        # show the main window
        self.setCentralWidget(self.tabW)
        self.show()

    def _makeEthlayout(self):
        """
        Wrapper function to store all of the Eth widgets into a single layout,
        and finally add it to the main window's QStackLayout
        """

        self._qdbPage = QWidget()
        layout = QGridLayout()

        btn_init = QPushButton()
        btn_init.setText('read ID')
        btn_init.clicked.connect(self.readID)
        layout.addWidget(btn_init, 0, 0)

        btn = QPushButton()
        btn.setText('trigger')
        btn.clicked.connect(lambda: self.trigger(hard=True))
        layout.addWidget(btn, 0, 1)

        btn_gtimeout = QPushButton()
        btn_gtimeout.setText('read reg')
        btn_gtimeout.clicked.connect(self.readReg)
        layout.addWidget(btn_gtimeout, 0, 2)

        btn_writeReg = QPushButton()
        btn_writeReg.setText('write reg')
        btn_writeReg.clicked.connect(self.writeReg)
        layout.addWidget(btn_writeReg, 1, 0)

        self.s_addr = QSpinBox()
        self.s_addr.setRange(0, 32)
        self.s_addr.setValue(22)
        self._laddr = QLabel("addr")
        layout.addWidget(self._laddr, 1, 1)
        layout.addWidget(self.s_addr, 2, 1)

        self.s_addrVal = QSpinBox()
        self.s_addrVal.setRange(0, 4095)
        self.s_addrVal.setValue(2000)
        self._lval = QLabel("val")
        layout.addWidget(self._lval, 1, 2)
        layout.addWidget(self.s_addrVal, 2, 2)

        self._qdbPage.setLayout(layout)
        return self._qdbPage

    def _makeTestlayout(self):
        """
        Helper wrapped to put testing click functions on a separate tab for custom uses
        """
        self._testPage = QWidget()
        layout = QGridLayout()

        scan_dac = QPushButton()
        scan_dac.setText('Scan DAC')
        scan_dac.clicked.connect(self.ScanDAC)
        layout.addWidget(scan_dac, 0, 0)

        btn = QPushButton()
        btn.setText('Reset DAC')
        btn.clicked.connect(self.ResetDAC)
        layout.addWidget(btn, 0, 1)

        dbtn = QPushButton()
        dbtn.setText('Read Debug')
        dbtn.clicked.connect(self.ReadDebug)
        layout.addWidget(dbtn, 1, 0)

        hIntbtn = QPushButton()
        hIntbtn.setText('Hard Int')
        hIntbtn.clicked.connect(self.HardInt)
        layout.addWidget(hIntbtn, 1, 1)

        hIntbtn = QPushButton()
        hIntbtn.setText('Reset ASIC')
        hIntbtn.clicked.connect(self.ResetASIC)
        layout.addWidget(hIntbtn, 2, 0)

        self._testPage.setLayout(layout)
        return self._testPage

    ############################
    ## Test specific Commands ##
    ############################
    def ScanDAC(self):
        """
        Looking for the correct reg load and latch period combinations to set the DAC
        Per Gang: 16 bit word. 12 lowest are DAC. Bit<12> is used for power on and should be 'on'
        """
        # over 2 V spike at 15 load val
        loadVals = [400]
        # latchVals = [v for v in range(50, 401, 20)]
        latchVals = [400]

        ## setting the bits
        print('scanning dac')
        p_on = lambda x: ((x & 0x1) << 12)
        dac_set = lambda x: (x & 0xffff)
        cap_set = lambda x: ((x & 0x3) << 13)
        def dac_setting(dac_val, power_on=True, cap=0x3):
            return p_on(power_on) + dac_set(dac_val) + cap_set(cap)

        def bit_rev(b, rng=0, sz=16):
            """
            Reverse the endian-ness of the bit stream if we want to
            """
            if not(isinstance(rng, int)) or not (isinstance(sz, int)):
                return
            if rng>=int(sz/2):
                return b
            else:
                bp1 = rng
                bp2 = sz - rng - 1
                bv1 = b & (1 << bp1)
                bv2 = b & (1 << bp2)
                # if bv1 == bv2 there's nothing to do
                if bv1 and not bv2: # downsize
                    b = b + (1 << bp2) - (1 << bp1)
                elif bv2 and not bv1: # upsize
                    b = b - (1 << bp2) + (1 << bp1)
                bit_rev(b, rng=rng+1, sz=sz)

        dv = dac_setting(0xaaaa)
        time.sleep(0.5)
        self.ResetDAC()
        self.SetSPILoad(loadVals[0])
        self.SetSPILatch(latchVals[0])
        time.sleep(0.5)
        self.SetDAC(dv)


    def SetDAC(self, val):
        """
        Update value on the DAC
        """
        addr = QDA_CTRL_REG.SPI_DATA.value
        self.eth.regWrite(addr, val)

    def ResetDAC(self):
        """
        Send a high followed by a low pulse to the SPI_SRST port at addr 0x17
        """
        addr = QDA_CTRL_REG.SPI_RESET.value
        val = 1
        self.eth.regWrite(addr, val)

        val = 0
        self.eth.regWrite(addr, val)

    def ResetASIC(self):
        """
        Send Digital reset
        """
        addr = QDA_CTRL_REG.SEND_DEBUG.value
        val = 3
        self.eth.regWrite(addr, val)

    def SetSPILoad(self, val):
        """
        Configure the SPI load value
        """
        addr = QDA_CTRL_REG.SPI_LOAD_PERIOD.value
        self.eth.regWrite(addr, val)

    def SetSPILatch(self, val):
        """
        Configure the SPI latch value
        """
        addr = QDA_CTRL_REG.SPI_LATCH_PERIOD.value
        self.eth.regWrite(addr, val)

    def ReadDebug(self):
        """
        Read the Debug Bits
        """
        dVals = self.eth.regRead(QDA_CTRL_REG.READ_DEBUG.value)
        dTxBusy = (dVals & (1 << 0)) > 0
        dRxError = (dVals & (1 << 1)) > 0
        dRxBusy = (dVals & (1 << 2)) > 0
        dExtFifoFull = (dVals & (1 << 3)) > 0
        dLocFifoFull = (dVals & (1 << 4)) > 0
        dFsmState = (dVals & (0b111 << 5)) >> 5

        print("status of ASIC: ")
        if dTxBusy:
            print("TX BUSY")
        if dRxError:
            print("Rx ERROR")
        if dRxBusy:
            print("RX BUSY")
        if dExtFifoFull:
            print("Ext FIFO Full")
        if dLocFifoFull:
            print("Loc FIFO Full")
        print("FSM State: ", dFsmState)

    def HardInt(self):
        """
        Try to force something out of the ASIC
        """
        print("sending hard int..")
        addr = QDA_CTRL_REG.SEND_DEBUG.value
        self.eth.regWrite(addr, 1)


    ############################
    ## Zybo specific Commands ##
    ############################
    def readID(self):
        """
        Main function to send a trigger to update the two register addresses to read the eeprom ID
        """
        self.trigger(hard=False)
        time.sleep(0.100)
        lowVal = self.readReg(2)
        data = 0xffff_ffff & lowVal
        data = data >> 4
        highVal = self.readReg(3)
        highVal = highVal & 0xff
        highVal = highVal << 28
        data += highVal
        print(f"reg read val: {data:08x}")

    def readReg(self, addr):
        """
        read a specific asic reg
        """
        addr = self.s_addr.value()
        readVal = self.eth.regRead(addr)
        print(f"read addr reg: 0x{addr:06x}")
        print(f"reg read val: {readVal:08x}")
        return readVal

    def writeReg(self):
        """
        write a specific asic reg
        """
        addr = self.s_addr.value()
        val = self.s_addrVal.value()
        print(f"writing addr reg: 0x{addr:06x}")
        print(f"writing val: 0x{val:04x}")
        self.eth.regWrite(addr, val)

    def trigger(self, hard=True):
        """
        Send a basic trigger packet to the board.

        This interrogation will be sent to all ASICs in the array, and memory
        will be recorded into the BRAM within QpixDaqCtrl.vhd.
        """
        if hard:
            print("sending trig")
        addr = REG.TRIG
        self.eth.regWrite(addr, 1)


    def getDMARegisters(self):
        """
        Print the DMA register status, connected to a button
        """
        print("printing DMA Registers:")
        self.eth.PrintDMA()


    def resetDMA(self):
        """
        reset DMA by pinging the correct ctrl register
        """
        print("reseting the DMA!")
        self.eth._resetDMA()


    ###########################
    ## GUI specific Commands ##
    ###########################

    def _make_menuBar(self):
        menubar = self.menuBar()
        menubar.setNativeMenuBar(False)

        # exit action
        exitAct = QAction(QIcon(), '&Exit', self)
        exitAct.setShortcut('Ctrl+Q')
        exitAct.setStatusTip('Exit application')
        exitAct.triggered.connect(self.close)

        # create a way to save the data collected
        saveAct = QAction(QIcon(), '&Save', self)
        saveAct.setShortcut('Ctrl+S')
        saveAct.setStatusTip('Save Data')
        saveAct.triggered.connect(self.SaveAs)

        # add the actions to the menuBar
        fileMenu = menubar.addMenu('File')
        fileMenu.addAction(exitAct)
        fileMenu.addAction(saveAct)


    def SaveAs(self):
        """
        file save
        """
        pass

    def reject(self):
        pass


if __name__ == "__main__":

    app = QApplication(sys.argv)
    window = GUI()
    app.exec_()
