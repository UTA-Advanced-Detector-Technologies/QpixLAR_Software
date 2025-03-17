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
        self.s_addr = QSpinBox()
        self.s_addr = QDoubleSpinBox()
        self.s_addr.setRange(0.0, 1.0)  # Set range for the float value
        self.s_addr.setSingleStep(0.01) 
        self.s_addr.setValue(0.5)
        self.s_addr.valueChanged.connect(self.readSPI)
        self._laddr = QLabel("VCOMP1")
        layout.addWidget(self._laddr, 1, 0)
        layout.addWidget(self.s_addr, 1, 1)

        # control to set SPI2
        self.s_addr2 = QSpinBox()
        self.s_addr2 = QDoubleSpinBox()
        self.s_addr2.setRange(0.0, 1.0)  # Set range for the float value
        self.s_addr2.setSingleStep(0.01) 
        self.s_addr2.setValue(0.5)
        self.s_addr2.valueChanged.connect(self.readSPI2)
        self._laddr = QLabel("VCOMP2")
        layout.addWidget(self._laddr, 1, 2)
        layout.addWidget(self.s_addr2, 1, 3)

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
        # scan_dac.clicked.connect(self.ScanDAC)
        layout.addWidget(scan_dac, 0, 0)

        btn = QPushButton()
        btn.setText('Reset DAC')
        # btn.clicked.connect(self.ResetDAC)
        layout.addWidget(btn, 0, 1)

        dbtn = QPushButton()
        dbtn.setText('Read Debug')
        # dbtn.clicked.connect(self.ReadDebug)
        layout.addWidget(dbtn, 1, 0)

        hIntbtn = QPushButton()
        hIntbtn.setText('Hard Int')
        # hIntbtn.clicked.connect(self.HardInt)
        layout.addWidget(hIntbtn, 1, 1)

        hIntbtn = QPushButton()
        hIntbtn.setText('Reset ASIC')
        # hIntbtn.clicked.connect(self.ResetASIC)
        layout.addWidget(hIntbtn, 2, 0)

        self._testPage.setLayout(layout)
        return self._testPage

    ############################
    ## Zybo specific Commands ##
    ############################
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

    def readSPI(self):
        """
        fill in addr value to be sent to the appropriate SPI controller
        """
        addr = self.calcSPI(self.s_addr.value())
        readVal = self.eth.regRead(addr, cmd='SPI')
        return readVal

    def readSPI2(self):
        """
        fill in addr value to be sent to the appropriate SPI controller
        """
        addr = self.calcSPI(self.s_addr2.value())
        addr |= 1<<31 # this is how the command interpreter selects between the SPI-DACs
        readVal = self.eth.regRead(addr, cmd='SPI')
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


if __name__ == "__main__":

    app = QApplication(sys.argv)
    window = GUI()
    app.exec_()
