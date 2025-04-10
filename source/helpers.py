## store some helper function methods used to calculate bit masking or other useful things here.
import numpy as np
import struct
import qpixlar_regmap as REG

ZTURN_FCLK0 = 200e6
ZTURN_FCLK1 = 50e6

def calcMaskFromCheckboxes(checkBoxList):
    s = [ 1<<i if checkBoxList[i].isChecked() else 0 for i in range(len(checkBoxList))]
    mask = sum(s)
    return mask

def readEvtFromBytes(qbytearray):
    """
    wrapper function used by the UDPWorker to unpack the binary stream sent from
    the zynq
    ARGS: 
        qbytearray : QByteArray read from the QUDPSocket
    RETURNS: 
        the columns that are sent into the output h5 file.
        mask      - 16 bit mask which represent whether or not a particular channel was in reset
        timestamp - 64 bit value indicating the current count of the zynq clock
        metadata  - 32 bit meta data value from the datastream plus the associated QUDPWorker
    """
    b = bytes(qbytearray)
    if len(b) < 12:
        mask      = 0
        timestamp = 0
    else:
        mask      = struct.unpack('<H', b[8:10])[0]
        timestamp = struct.unpack('<Q', b[0:8])[0]
    metadata  = 42

    print("byte array: ", b)
    print(f"mask={mask:04x}, timestamp={timestamp*5e-9:.2e}, metadata={metadata}")
    return mask, timestamp, metadata

##################################################
#########   Implemented QPix Helpers   ###########
##################################################
# each qpix function should return an addr and a value 
# which performs a specified function. the return values of 
# these functions can be sent into the "_sendQpix" method
# of the eth_ctrl.py GUI class.
def get_system_reset():
    return REG.REG0, 0x01

def set_system_calibration():
    return REG.REG0, 1 << 4

def set_system_clear():
    return REG.REG0, 0

def set_system_window_width(t_us=25):
    """
    give a time in microseconds to set the reset window width
    """
    return REG.REG8, int((t_us*1e-6) * ZTURN_FCLK1)

def get_serial_addrs(interface_num=1):
    """
    wrapper function to help prevent the shameful implmentation done by Penn..
    NOTE: under no circumstances should running python code (after 7 years
    of developments) require an external excel sheet to run, whereby a user 
    copies the produced magic numbers to control the DUT.
    """
    assert interface_num == 1 or interface_num == 2, "must select interface 1 or 2"

    # either pad1 or pad2 implementation
    if interface_num == 1:
        ctrl_addr = REG.REG1
        data_addr = REG.REG2
    else:
        ctrl_addr = REG.REG3
        data_addr = REG.REG4
    return ctrl_addr, data_addr

def make_serial_word(replen_cur=0b01011, curReplen=0b010, curCmp=0b0011, curAmp=0b011, curInt=0b011, 
                     enable=0x03, clk_source_ro=True, dbl_bar=False, enableRingOsc=True, ringOsc_f=0b10,
                     LVDS_drvr=True, enableCalB=True):
    """
    wrapper function to help prevent the shameful implmentation done by Penn..
    NOTE: under no circumstances should running python code (after 5+ years
    of development) require an external excel sheet to run, whereby a user 
    copies produced magic numbers into a string to control the DUT..

    We should all take this opportunity to reflect on the need to take pride in
    our work.
    """
    data_word = 0

    repl_top = (replen_cur & (0xf << 1)) >> 1
    repl_bot = replen_cur & 0b1

    # build the data word
    data_word |= repl_top
    data_word |= (curReplen & 0x7)    << 4
    data_word |= (curCmp    & 0x7)    << 7
    data_word |= (curAmp    & 0x7)    << 9 
    data_word |= (curInt    & 0x7)    << 13
    data_word |= (enable    & 0xff)   << 16
    data_word |= (repl_bot        )   << 24
    data_word |= (int(clk_source_ro)) << 25
    data_word |= (int(dbl_bar))       << 26
    data_word |= (ringOsc_f & 0x3)    << 27
    data_word |= (int(enableRingOsc)) << 29
    data_word |= (int(LVDS_drvr))     << 30
    data_word |= (int(enableCalB))    << 31

    return data_word

def set_system_reset_width(t_us=25):
    """
    reset register is an double duty register, which stores
    the 
    """
    return REG.REG9, int((t_us*1e-6) * ZTURN_FCLK1)
    
def set_deltaT_delay(t_us=10, enable=False):
    """
    reset register is an double duty register, which stores
    the 
    ARGS:
        t_us   : microsecond wait time, default to 10 us
        enable : if enabled then bit 32 is enabled on this register
                 this bit is tired to sample_select in the firmware
    """
    delay_time = int((t_us*1e-6) * ZTURN_FCLK1)
    reg_val = delay_time | (1<<31) if enable else delay_time
    return REG.REG9, reg_val
    
def set_deltaT_select(enable=False):
    """
    reset register is an double duty register, which stores
    the 
    ARGS:
        enable : this bit is tired to deltaT_select in the firmware
    """
    reg_val = 1 if enable else 0
    return REG.REGA, reg_val

def set_ctrl(offset):
    """
    wrapper function to set a single bit high useing one of the REG offset values
    These are used to set values in the QCTRL registers as well as the QPAD_CTRL
    registers.
    """
    return 1<<offset