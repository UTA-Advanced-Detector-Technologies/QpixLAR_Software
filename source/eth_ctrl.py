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
import helpers as helper
from helpers import WidgetNames as WN

QPIX_SER_TIME = 0.050 # 50 ms

class GUI(QMainWindow):

    close_udp = pyqtSignal()

    def __init__(self):
        super(QMainWindow, self).__init__()

        # IO interfaces
        self.eth = eth_interface()
        # if not self.eth.isConnected():
        #     sys.exit(-1)

        self.close_udp.connect(self.eth.finish) # closes udp worker thread
        self.version = self.eth.version

        # window setup
        self.setWindowTitle('Ethernet Control')

        # initialize the sub menus
        self._make_menuBar()

        # create the layouts that are needed for making the GUI pretty
        self.tabW = QTabWidget()
        self.tabW.addTab(self._makeEthlayout(), "ETH")
        self.tabW.addTab(self._makeMaskLayout(), "Mask Ctrl")
        self.tabW.addTab(self._makeCTRLayout(), "Standard Ctrl")
        self.tabW.addTab(self._makeCTRLayout(1), "C-Gain Ctrl")
        self.tabW.addTab(self._makeCalibrationsLayout(), "Calibrations")

        self._assignDefaultPadCtrl(0)
        self._assignDefaultPadCtrl(1)

        # show the main window
        self.setCentralWidget(self.tabW)
        self.show()

        # default calibration time in seconds
        self.cal_time = 1.0

        self._toggleForceEnable.setChecked(1)
        self.updateShutdownMask()
        self.updateTriggerMask()

        # define initial vcm's for i2c when connected
        self.vcm_addr1.setValue(0.967)
        self.vcm_addr2.setValue(0.95)

        # vcomps set here
        self.s_addr.setValue(0.720)
        self.s_addr2.setValue(0.875)

        self.InitQpix()


    def _makeEthlayout(self):
        """
        Wrapper function to store all of the Eth widgets into a single layout,
        and finally add it to the main window's QStackLayout
        """

        self._qdbPage = QWidget()
        layout = QGridLayout()

        self.check_kick = QCheckBox('Kick Start')
        self.check_kick.stateChanged.connect(lambda x: self.qpix_kickstart())
        layout.addWidget(self.check_kick, 0, 0)

        self.startup = QPushButton()
        self.startup.setText('Startup')
        self.startup.clicked.connect(lambda x: self.qpix_startup())
        layout.addWidget(self.startup, 0, 1)


        btn_read_qpix = QPushButton()
        btn_read_qpix.setText('Reset QPIX')
        btn_read_qpix.clicked.connect(lambda x: self.ResetQpix())
        layout.addWidget(btn_read_qpix, 0, 2)

        # btn_read_ctrl = QPushButton()
        # btn_read_ctrl.setText('CTRL')
        # btn_read_ctrl.clicked.connect(lambda x: self.readReg('CTRL'))
        # layout.addWidget(btn_read_ctrl, 0, 3)

        # control to set SPI1
        self.s_addr = QDoubleSpinBox()
        self.s_addr.setRange(0.0, 1.0)  # Set range for the float value
        self.s_addr.setSingleStep(0.001) 
        self.s_addr.setDecimals(3)
        self.s_addr.setValue(0.5)
        self.s_addr.valueChanged.connect(self.readSPI)
        self._laddr = QLabel("VCOMP1")
        layout.addWidget(self._laddr, 1, 0)
        layout.addWidget(self.s_addr, 1, 1)

        # control to set SPI2
        self.s_addr2 = QDoubleSpinBox()
        self.s_addr2.setRange(0.0, 1.0)  # Set range for the float value
        self.s_addr2.setSingleStep(0.001) 
        self.s_addr2.setDecimals(3)
        self.s_addr2.setValue(0.5)
        self.s_addr2.valueChanged.connect(self.readSPI2)
        self._laddr = QLabel("VCOMP2")
        layout.addWidget(self._laddr, 1, 2)
        layout.addWidget(self.s_addr2, 1, 3)

        # control to set VCM1
        self.vcm_addr1 = QDoubleSpinBox()
        self.vcm_addr1.setRange(0.0, 1.0)  # Set range for the float value
        self.vcm_addr1.setSingleStep(0.001) 
        self.vcm_addr1.setDecimals(3)
        self.vcm_addr1.setValue(0.5)
        self.vcm_addr1.valueChanged.connect(self.readI2C_1)
        self._laddr = QLabel("VCM1")
        layout.addWidget(self._laddr, 2, 0)
        layout.addWidget(self.vcm_addr1, 2, 1)

        # control to set VCM2
        self.vcm_addr2 = QDoubleSpinBox()
        self.vcm_addr2.setRange(0.0, 1.0)  # Set range for the float value
        self.vcm_addr2.setSingleStep(0.001) 
        self.vcm_addr2.setDecimals(3)
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

    def _makeMaskLayout(self):
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

        self._toggleShutdownMasks = QCheckBox(f"enable all")
        self._toggleShutdownMasks.stateChanged.connect(self.toggleShutdownMasks)
        layout.addWidget(self._toggleShutdownMasks, 0, 1)
        
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

        self._toggleTriggerMasks = QCheckBox(f"enable all")
        self._toggleTriggerMasks.stateChanged.connect(self.toggleTriggerMask)
        layout.addWidget(self._toggleTriggerMasks, 5, 1)

        self._toggleForceEnable = QCheckBox(f"force enable")
        self._toggleForceEnable.stateChanged.connect(self.toggleForceEnable)
        layout.addWidget(self._toggleForceEnable, 5, 2)

        self._testPage.setLayout(layout)
        return self._testPage

    def _makeCalibrationsLayout(self):

        self._testPage = QWidget()
        layout = QGridLayout()

        self.check_calib = QPushButton('Calibrate')
        self.check_calib.clicked.connect(lambda x: self.QpixCalibration())
        layout.addWidget(self.check_calib, 0, 0)
        
        self.check_clock = QCheckBox('Clock ON')
        self.check_clock.stateChanged.connect(lambda x: self.QpixClockState())
        layout.addWidget(self.check_clock, 0, 1)

        self.check_serialIntReset1 = QPushButton('Reset Serial Interface Standard')
        self.check_serialIntReset1.clicked.connect(lambda x: self.QpixSerialReset(interface_num=1))
        layout.addWidget(self.check_serialIntReset1, 2, 0)

        self.check_serialIntReset2 = QPushButton('Reset Serial Interface C-Gain')
        self.check_serialIntReset2.clicked.connect(lambda x: self.QpixSerialReset(interface_num=2))
        layout.addWidget(self.check_serialIntReset2, 2, 1)

        self.check_integratorReset1 = QPushButton('Reset Integrator Standard')
        self.check_integratorReset1.clicked.connect(lambda x: self.QpixIntegratorReset(pads=[1]))
        layout.addWidget(self.check_integratorReset1, 3, 0)

        self.check_integratorReset2 = QPushButton('Reset Integrator C-Gain')
        self.check_integratorReset2.clicked.connect(lambda x: self.QpixIntegratorReset(pads=[2]))
        layout.addWidget(self.check_integratorReset2, 3, 1)

        self._testPage.setLayout(layout)
        return self._testPage

    def _makeCTRLayout(self, num_pad=0):
        """
        Wrapper function that exposes the dataword controls for one of the qpix
        asic pad control registers

        Changing any of the values here will change the value of the pad ctrl register
        immediately
        ARGS:
            num_pad : int select the pad number, either 1 or 2
        """
        assert num_pad in [0,1], f"pad number must be equal to 1 or 2 not {num_pad}"
        self._testPage = QWidget()

        # ensure that we have some initialization of the serial
        # config for both of the ports
        if not hasattr(self, 'pads_ctrl'):
            self.pads_ctrl = [helper.SerialConfig(), helper.SerialConfig()]
        pad_ctrl = self.pads_ctrl[num_pad-1]

        layout = QGridLayout()

        def addBox(layout, num_pad, name, i):
            self._BoolBoxes[num_pad][name] = QCheckBox(name)
            a = self._BoolBoxes[num_pad][name]
            layout.addWidget(a, i, 0)

        # bool flags from the pad_ctrl
        if not hasattr(self, '_BoolBoxes'):
            self._BoolBoxes = [{}, {}]
        addBox(layout, num_pad, WN.CLK_SRC.value, 0)
        addBox(layout, num_pad, WN.DBL_BAR.value, 1)
        addBox(layout, num_pad, WN.RNG_OSC.value, 2)
        addBox(layout, num_pad, WN.LVDS_DR.value, 3)
        addBox(layout, num_pad, WN.EN_CALB.value, 4)

        # replen_cur control
        def addBoxArray(layout, num_pad, name, i, sz):
            hbox_layout = QHBoxLayout()
            self._repLenBoxes[num_pad][name.value] = [QCheckBox(f"{name.value}{k}") for k in range(sz)]
            for box in self._repLenBoxes[num_pad][name.value]:
                hbox_layout.addWidget(box)
            placeholder_widget = QWidget()
            placeholder_widget.setLayout(hbox_layout)
            layout.addWidget(placeholder_widget, i, 1)

        if not hasattr(self, '_repLenBoxes'):
            self._repLenBoxes = [{}, {}]
        addBoxArray(layout, num_pad, WN.REP_LEN, 0, 5)
        addBoxArray(layout, num_pad, WN.CUR_LEN, 1, 3)
        addBoxArray(layout, num_pad, WN.CUR_CMP, 2, 3)
        addBoxArray(layout, num_pad, WN.CUR_AMP, 3, 3)
        addBoxArray(layout, num_pad, WN.CUR_INT, 4, 3)
        addBoxArray(layout, num_pad, WN.ENABLE,  5, 8)
        addBoxArray(layout, num_pad, WN.RING_OS, 6, 2)

        self._testPage.setLayout(layout)
        return self._testPage

    def updatePadCtrlReg(self, num_pad):
        """
        when any of these values change update the corresponding pad control
        """
        assert num_pad in [0,1], f"update pad ctrl be equal to 1 or 2 not {num_pad}"
        ser_word = self.pads_ctrl[num_pad]

        # helper to extract bit values from boxes based on key name
        def makeBit(boxDict, name, reverse_sz=0):
            boxes = boxDict[name.value]
            val = [1<<i if box.isChecked() else 0 for i, box in enumerate(boxes)]
            bits = sum(val)
            # replen doesn't need to be reversed, but all other bits do
            if reverse_sz>0:
                bits = helper.reverse_bits(bits, reverse_sz)
            return bits

        # update bit ranges
        lenBoxes = self._repLenBoxes[num_pad]
        ser_word.replen_cur = makeBit(lenBoxes, WN.REP_LEN)
        ser_word.curReplen = makeBit(lenBoxes, WN.CUR_LEN, 3)
        ser_word.curCmp = makeBit(lenBoxes, WN.CUR_CMP, 3)
        ser_word.curAmp = makeBit(lenBoxes, WN.CUR_AMP, 3)
        ser_word.curInt = makeBit(lenBoxes, WN.CUR_INT, 3)
        ser_word.enable = makeBit(lenBoxes, WN.ENABLE, 8)
        ser_word.ringOsc_f = makeBit(lenBoxes, WN.RING_OS, 2)

        # update bools
        boolBoxes = self._BoolBoxes[num_pad]
        ser_word.clk_source_ro = boolBoxes[WN.CLK_SRC.value].isChecked()
        ser_word.dbl_bar = boolBoxes[WN.DBL_BAR.value].isChecked()
        ser_word.enableRingOsc = boolBoxes[WN.RNG_OSC.value].isChecked()
        ser_word.LVDS_drvr = boolBoxes[WN.LVDS_DR.value].isChecked()
        ser_word.enableCalB = boolBoxes[WN.EN_CALB.value].isChecked()

        # update and send
        data = helper.make_serial_word(ser_word)
        print(f"{data:08x}")
        self.pads_ctrl[num_pad] = ser_word
        self.QpixSerial(num_pad+1, data)
    
    def _assignDefaultPadCtrl(self, num_pad):
        """
        helper function to use defaults from SerialConfig default constructor
        to set initial values for the appropriate check boxes.

        This function is the behavioral equivalent of the inverse of 'self.updatePadCtrlReg'
        """
        assert num_pad in [0,1], f"update pad ctrl be equal to 1 or 2 not {num_pad}"
        ser_word = self.pads_ctrl[num_pad]

        # helper to extract bit values from boxes based on key name
        def setChecked(boxDict, name, bits, reverse_sz=0):
            boxes = boxDict[name.value]
            if reverse_sz>0:
                bits = helper.reverse_bits(bits, reverse_sz)
            _ = [box.setChecked(1) if 1<<i & bits else box.setChecked(0) 
                                   for i, box in enumerate(boxes)]

        # update bit ranges
        lenBoxes = self._repLenBoxes[num_pad]
        setChecked(lenBoxes, WN.REP_LEN, ser_word.replen_cur)
        setChecked(lenBoxes, WN.CUR_LEN, ser_word.curReplen, 3)
        setChecked(lenBoxes, WN.CUR_CMP, ser_word.curCmp, 3)
        setChecked(lenBoxes, WN.CUR_AMP, ser_word.curAmp, 3)
        setChecked(lenBoxes, WN.CUR_INT, ser_word.curInt, 3)
        setChecked(lenBoxes, WN.ENABLE, ser_word.enable, 8)
        setChecked(lenBoxes, WN.RING_OS, ser_word.ringOsc_f, 2)
        # after all default assignments, now we can connect boxes
        for name, boxes in lenBoxes.items():
            for box in boxes:
                box.stateChanged.connect(lambda x: self.updatePadCtrlReg(num_pad))

        # update bools
        boolBoxes = self._BoolBoxes[num_pad]
        boolBoxes[WN.CLK_SRC.value].setChecked(ser_word.clk_source_ro)
        boolBoxes[WN.DBL_BAR.value].setChecked(ser_word.dbl_bar)
        boolBoxes[WN.RNG_OSC.value].setChecked(ser_word.enableRingOsc)
        boolBoxes[WN.LVDS_DR.value].setChecked(ser_word.LVDS_drvr)
        boolBoxes[WN.EN_CALB.value].setChecked(ser_word.enableCalB)
        # after all default assignments, now we can connect boxes
        for name, box in boolBoxes.items():
            box.stateChanged.connect(lambda x: self.updatePadCtrlReg(num_pad))

        # send
        data = helper.make_serial_word(ser_word)
        print(f"data = {data:08x}")

    def updateShutdownMask(self):
        """
        update the LTC shutdown pins on a checkbox value change
        """
        mask = calcMaskFromCheckboxes(self._shutdownMask)
        # example sanity checking here
        # print(f"sum is: {mask:04x}")
        self.readCTRL(reg_addr=REG.CTRL_SHDN, val=mask)

    def toggleShutdownMasks(self):
        """
        helper function from a checkbox that will enable or disable all of the
        ltc pins and either enable or disable it
        """
        isChecked = self._toggleShutdownMasks.isChecked()
        _ = [a.setChecked(isChecked) for a in self._shutdownMask ]

    def toggleTriggerMask(self):
        """
        helper function from a checkbox that will enable or disable all of the
        ASIC channels connected to this the zturn
        """
        isChecked = self._toggleTriggerMasks.isChecked()
        _ = [a.setChecked(isChecked) for a in self._triggerMask ]

    def toggleForceEnable(self):
        """
        helper function which will set the force enable bit to the firmware register.
        this will allow all sampling and resets to come through, regardless of the sampling
        window set by the opaque 'sample select' bit
        """
        isChecked = self._toggleForceEnable.isChecked()
        self.readCTRL(reg_addr=REG.CTRL_FORCE_VALID, val=int(isChecked))

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

    def _sendQpix(self, addr, val):
        """
        wrapper function which implements the equivalent of the os.system('poke <addr> <val>')
        from the Penn Version.
        ARGS:
            addr : implemented register offset
            val  : register value to be set at addr
        Returns:
            ack value from the typcial transaction with the zynq
        """
        readVal = self.eth.regWrite(addr=addr, val=val, cmd='QPIX')
        return readVal

    def _getQpix(self, addr, val):
        """
        wrapper function which implements the equivalent of the os.system('peek <addr>')
        from the Penn Version.
        ARGS:
            addr : implemented register offset
        Returns:
            val  : 32 bit register value seen at the register
        """
        addr |= 1<<31 # must set read bit high for the embedded firmware to recognize this as a read
        readVal = self.eth.regRead(addr=addr, cmd='QPIX')
        return readVal

    def InitQpix(self):
        """
        Wrapper function to set qpix registers to a known starting state on GUI
        boot
        """
        print("Qpix Init?")

        addr, val = helper.get_system_reset()
        self._sendQpix(addr, val)

        # REG7 Config - Sample Window Width
        addr, val = helper.set_system_window_width()
        self._sendQpix(addr, val)

        # REG8 Config
        addr, val = helper.set_system_reset_width()
        self._sendQpix(addr, val)

        # REG9 Config
        addr, val = helper.set_deltaT_delay()
        self._sendQpix(addr, val)

        # REGA Config
        addr, val = helper.set_deltaT_select()
        self._sendQpix(addr, val)

        # begin system calibration
        # addr, val = helper.set_system_calibration()
        # self._sendQpix(addr, val)

        addr, val = helper.set_system_clear()
        self._sendQpix(addr, val)

        # TODO Kalindi verify that updating serial interface turns on spy point here
        self.QpixSerial(1, 0x55B680CE)
        return

        # TODO verify when the serial configuration registers should be tuned
        return
        # on boot re-load the default configuration registers
        self.updatePadCtrlReg(0)
        self.updatePadCtrlReg(1)

        # reset integrators on boot
        self.QpixIntegratorReset()

        # control register inits
        self.updateShutdownMask()
        self.updateTriggerMask()

    def QpixSerial(self, interface_num, data_word):
        """
        An improved (useful) Implementation of Serial_Interface.py
        """
        ctrl_addr, data_addr = helper.get_serial_addrs(interface_num)
        print(f"QSerial: 0x{data_word:08x}")
        self._sendQpix(data_addr, data_word)

        # lambda helper
        load = lambda x: helper.set_ctrl(x)

        # shift register load, piso requirement
        time.sleep(QPIX_SER_TIME)
        self._sendQpix(ctrl_addr, load(REG.QPAD_CTRL_load))
        time.sleep(QPIX_SER_TIME)
        self._sendQpix(ctrl_addr, 0)
        time.sleep(QPIX_SER_TIME)

        self._sendQpix(ctrl_addr, load(REG.QPAD_CTRL_xmit))
        time.sleep(QPIX_SER_TIME)

        self._sendQpix(ctrl_addr, load(REG.QPAD_CTRL_load_data))
        time.sleep(QPIX_SER_TIME)
        self._sendQpix(ctrl_addr, 0)

    def QpixCalibration(self):
        """
        Implementation of Calibrate.py
        """
        ctrl_addr = REG.REG0  # control register 
        load = lambda x: helper.set_ctrl(x)

        print(f"waiting for time={self.cal_time}", end=".. ")
        self._sendQpix(ctrl_addr, load(REG.QCTRL_calibrate))

        time.sleep(self.cal_time)
        self._sendQpix(ctrl_addr, 0)
        print("calibration complete!")

    def QpixClockState(self, isOn=True):
        """
        Implementation of Clock_50MHz.py
        ARGS:
            isOn : if isOn is true, will turn on the pad's clocks, otherwise they off
        """
        ctrl_addr = REG.REG0
        load = lambda x: helper.set_ctrl(x)
    
        if isOn:
            val = load(REG.QCTRL_opad_clk_rpen1) | load(REG.QCTRL_opad_clk_rpen2)
        else:
            val = 0
        time.sleep(QPIX_SER_TIME)
        self._sendQpix(ctrl_addr, val)

    def QpixSerialReset(self, interface_num):
        """
        Implementation of Serial_Interface_rst.py
        """
        ctrl_addr, data_addr = helper.get_serial_addrs(interface_num)
        load = lambda x: helper.set_ctrl(x)

        # TODO confirm that this should set bit 9 first, then bit 8
        val = load(REG.QPAD_CTRL_opad_selDefData)
        self._sendQpix(ctrl_addr, val)
        time.sleep(QPIX_SER_TIME)
        
        # send 100us pulse on opad_loadData
        val |= load(REG.QPAD_CTRL_load_data)
        self._sendQpix(ctrl_addr, val)  
        time.sleep(QPIX_SER_TIME)

        # clear respective ctrl_addr
        self._sendQpix(ctrl_addr, 0)
        time.sleep(QPIX_SER_TIME)

    def ResetQpix(self):
        print("reseting the qpix system")
        addr, val = helper.get_system_reset()
        self._sendQpix(addr, val)

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

    ########################
    ## Wrapper C Commands ##
    ########################
    def qpix_kickstart(self):
        """
        send the equivalent of the qpix kickstart script
        """
        if self.check_kick.isChecked():
            addr, val = REG.REG0, (1<<REG.QCTRL_opad_startup1) | (1<<REG.QCTRL_opad_startup2)
            self._sendQpix(addr, val)
        else:
            addr, val = helper.set_system_clear()
            self._sendQpix(addr, val)

    def qpix_enable_cal(self):
        """
        send the equivalent of the qpix kickstart script
        """
        if self.check_kick.isChecked():
            addr, val = REG.REG0, (1<<REG.QCTRL_opad_startup1) | (1<<REG.QCTRL_opad_startup2)
            self._sendQpix(addr, val)
        else:
            addr, val = helper.set_system_clear()
            self._sendQpix(addr, val)

    def qpix_startup(self):
        """
        send the equivalent of the qpix startup script
        """
        addr, val = helper.get_system_reset()
        self._sendQpix(addr, val)
        time.sleep(0.1)
        addr, val = helper.set_system_clear()
        self._sendQpix(addr, val)

    def QpixIntegratorReset(self, pads=[1,2]):
        """
        implements the equivalent off Integrator_Rst_Fix.py 
        """
        addr, val = helper.get_integrator_pads(pads)
        self._sendQpix(addr, val)
        time.sleep(0.1)

        addr = REG.REG0
        load = lambda x: helper.set_ctrl(x)
        val = load(REG.QCTRL_opad_clk_rpen1) | load(REG.QCTRL_opad_clk_rpen2)
        self._sendQpix(addr, val)

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

    def closeEvent(self, event):
        print("close the gui..")
        self.close_udp.emit()
