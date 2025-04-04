import sys
import time

# PyQt GUI things
from PyQt5.QtWidgets import (QWidget, QPushButton, QCheckBox, QComboBox, QSpinBox, QLabel,
                             QDoubleSpinBox, QProgressBar, QTabWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QStatusBar,
                             QDialog, QDialogButtonBox, QLCDNumber, QFileDialog)
from PyQt5.QtCore import QProcess, QTimer, pyqtSignal
from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import QApplication, QMainWindow, QAction

from eth_interface import (eth_interface, EthBadAddr, DEFAULT_PACKET_SIZE)

import qpixlar_regmap as REG
from helpers import calcMaskFromCheckboxes

class GUI(QMainWindow):

    close_udp = pyqtSignal()

    def __init__(self):
        super(QMainWindow, self).__init__()

        # IO interfaces
        self.eth = eth_interface()
        if not self.eth.isConnected():
            sys.exit(-1)

        self.close_udp.connect(self.eth.finish) # closes udp worker thread
        self.version = self.eth.version

        # window setup
        self.setWindowTitle('Ethernet Control')

        # initialize the sub menus
        self._make_menuBar()

        # create the layouts that are needed for making the GUI pretty
        self.tabW = QTabWidget()
        self.tabW.addTab(self._makeEthlayout(), "ETH")

        self.tabW.addTab(self._makeCTRLayout(), "CTRL")

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

        btn_read_i2c = QPushButton()
        btn_read_i2c.setText('i2c')
        btn_read_i2c.clicked.connect(lambda x: self.readReg('I2C'))
        layout.addWidget(btn_read_i2c, 0, 0)

        btn_read_qpix = QPushButton()
        btn_read_qpix.setText('QPIX')
        btn_read_qpix.clicked.connect(lambda x: self.readReg('QPIX'))
        layout.addWidget(btn_read_qpix, 0, 2)

        btn_read_ctrl = QPushButton()
        btn_read_ctrl.setText('CTRL')
        btn_read_ctrl.clicked.connect(lambda x: self.readReg('CTRL'))
        layout.addWidget(btn_read_ctrl, 0, 3)

        # control to set SPI1
        self.s_addr = QDoubleSpinBox()
        self.s_addr.setRange(0.0, 1.0)  # Set range for the float value
        self.s_addr.setSingleStep(0.01) 
        self.s_addr.setValue(0.5)
        self.s_addr.valueChanged.connect(self.readSPI)
        self._laddr = QLabel("VCOMP1")
        layout.addWidget(self._laddr, 1, 0)
        layout.addWidget(self.s_addr, 1, 1)

        # control to set SPI2
        self.s_addr2 = QDoubleSpinBox()
        self.s_addr2.setRange(0.0, 1.0)  # Set range for the float value
        self.s_addr2.setSingleStep(0.01) 
        self.s_addr2.setValue(0.5)
        self.s_addr2.valueChanged.connect(self.readSPI2)
        self._laddr = QLabel("VCOMP2")
        layout.addWidget(self._laddr, 1, 2)
        layout.addWidget(self.s_addr2, 1, 3)

        # control to set VCM1
        self.vcm_addr1 = QDoubleSpinBox()
        self.vcm_addr1.setRange(0.0, 1.0)  # Set range for the float value
        self.vcm_addr1.setSingleStep(0.01) 
        self.vcm_addr1.setValue(0.5)
        self.vcm_addr1.valueChanged.connect(self.readI2C_1)
        self._laddr = QLabel("VCM1")
        layout.addWidget(self._laddr, 2, 0)
        layout.addWidget(self.vcm_addr1, 2, 1)

        # control to set VCM2
        self.vcm_addr2 = QDoubleSpinBox()
        self.vcm_addr2.setRange(0.0, 1.0)  # Set range for the float value
        self.vcm_addr2.setSingleStep(0.01) 
        self.vcm_addr2.setValue(0.5)
        self.vcm_addr2.valueChanged.connect(self.readI2C_2)
        self._laddr = QLabel("VCM2")
        layout.addWidget(self._laddr, 2, 2)
        layout.addWidget(self.vcm_addr2, 2, 3)

        # control to set V_LVDS_CM
        self.vcm_addr3 = QDoubleSpinBox()
        self.vcm_addr3.setRange(0.0, 1.0)  # Set range for the float value
        self.vcm_addr3.setSingleStep(0.01) 
        self.vcm_addr3.setValue(0.5)
        self.vcm_addr3.valueChanged.connect(self.readI2C_3)
        self._laddr = QLabel("LVDS_CM")
        layout.addWidget(self._laddr, 3, 0)
        layout.addWidget(self.vcm_addr3, 3, 1)

        self._qdbPage.setLayout(layout)
        return self._qdbPage

    def _makeCTRLayout(self):
        """
        Helper wrapped to put testing click functions on a separate tab for custom uses
        """
        self._testPage = QWidget()
        layout = QGridLayout()

        # 4x4 channel grid of check boxes which control whether or not the LTC
        # chip at the corresponding channel will be shutdown..
        l =  QLabel("LTC Enable")
        layout.addWidget(l, 0, 0)
        self._shutdownMask = []
        for i in range(4):
            for j in range(4):
                a = QCheckBox(f"{4*i+j}")
                a.stateChanged.connect(self.updateShutdownMask)
                self._shutdownMask.append(a)
                layout.addWidget(a, 1+i, j)

        # 4x4 channel grid of check boxes which control whether or not a trigger
        # will occur if a reset is detected at this pixel
        l =  QLabel("Trigger Enable")
        layout.addWidget(l, 5, 0)
        self._triggerMask = []
        for i in range(4):
            for j in range(4):
                a = QCheckBox(f"{4*i+j}")
                a.setChecked(1) # enable all bits by default
                a.stateChanged.connect(self.updateTriggerMask)
                self._triggerMask.append(a)
                layout.addWidget(a, 6+i, j)

        self._testPage.setLayout(layout)
        return self._testPage

    def updateShutdownMask(self):
        """
        update the LTC shutdown pins on a checkbox value change
        """
        mask = calcMaskFromCheckboxes(self._shutdownMask)
        # example sanity checking here
        # print(f"sum is: {mask:04x}")
        self.readCTRL(reg_addr=REG.CTRL_SHDN, val=mask)

    def updateTriggerMask(self):
        """
        update the LTC shutdown pins on a checkbox value change
        """
        mask = calcMaskFromCheckboxes(self._triggerMask)
        # example sanity checking here
        # print(f"sum is: {mask:04x}")
        self.readCTRL(reg_addr=REG.CTRL_MASK, val=mask)

    #############################
    ## Zturn specific Commands ##
    #############################
    def calcSPI(self, spin_box_value):
        """
        helper function to convert double spin box to useful bit value sent to SPI
        """
        addr = 0
        spi_buf = 1 << 14 # buffer DAC output
        addr |= spi_buf

        spi_gain = 1 << 13 # active low gain*2
        addr |= spi_gain

        spi_on = 1 << 12 # active low shut down
        addr |= spi_on

        dval = int(spin_box_value * 1024)
        dac_val = (dval & 0x3ff) << 2
        addr |= dac_val

        return addr

    def calcI2C(self, addr, port, control, dac_value):
        """
        helper function used to construct i2c addr sent to zynq cmd interpreter
        """
        i2c_val = 0

        # select i2c addr
        i2c_val |= (addr & 0xff) << 20

        # select DAC port
        i2c_val |= (port & 0x3) << 16 # DACB | DACA

        # select control bits
        i2c_val |= (control & 0xf) << 12 # PD1 | PD0 | bCLR | bLDAC

        # update dac value, using 12 bit DAC
        dval = int(dac_value * 2048)
        i2c_val |= (dval & 0xfff)

        return i2c_val

    def calcQPIX(self, addr):
        """
        Wrapper function to access qpix interface registers
        """
        pass

    def calcCTRL(self, addr):
        """
        Wrapper function to access CTRL registers
        """
        pass
        
    def readSPI(self):
        """
        fill in addr value to be sent to the appropriate SPI controller
        used to control VCOMP1 for this particular Zturn carrier.
        """
        addr = self.calcSPI(self.s_addr.value())
        readVal = self.eth.regRead(addr, cmd='SPI')
        return readVal

    def readSPI2(self):
        """
        fill in addr value to be sent to the appropriate SPI controller
        used to control VCOMP2 for this particular Zturn carrier.
        """
        addr = self.calcSPI(self.s_addr2.value())
        addr |= 1<<31 # this is how the command interpreter selects between the SPI-DACs
        readVal = self.eth.regRead(addr, cmd='SPI')
        return readVal

    def readI2C_1(self):
        """
        used to control VCM1
        """
        dval = self.vcm_addr1.value()
        reg_addr = self.calcI2C(addr=REG.IIC_SLAVE_ADDR_2, port=2,
                                control=REG.IIC_CTRL_DEFAULT, dac_value=dval)
        readVal = self.eth.regRead(reg_addr, cmd='I2C')
        return readVal

    def readI2C_2(self):
        """
        used to control VCM2
        """
        dval = self.vcm_addr2.value()
        reg_addr = self.calcI2C(addr=REG.IIC_SLAVE_ADDR_2, port=1,
                                control=REG.IIC_CTRL_DEFAULT, dac_value=dval)
        readVal = self.eth.regRead(reg_addr, cmd='I2C')
        return readVal

    def readI2C_3(self):
        """
        used to control LVDS_CM
        """
        dval = self.vcm_addr3.value()
        reg_addr = self.calcI2C(addr=REG.IIC_SLAVE_ADDR_1, port=1,
                                control=REG.IIC_CTRL_DEFAULT, dac_value=dval)
        readVal = self.eth.regRead(reg_addr, cmd='I2C')
        return readVal

    def readCTRL(self,  reg_addr, val):
        """
        Control register IO function
        These commands expect a single reg_addr space as well as up to a 32bit value
        to send to the register of choice
        """
        readVal = self.eth.regWrite(addr=reg_addr, val=val, cmd='CTRL')
        return readVal

    def readReg(self, cmd: str):
        """
        read a specific asic reg
        """
        addr = self.s_addr.value()
        readVal = self.eth.regRead(addr, cmd=cmd)
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

    def __del__(self):
        print("closing the gui..")
        self.close_udp.emit()


if __name__ == "__main__":

    app = QApplication(sys.argv)
    window = GUI()
    app.exec_()
