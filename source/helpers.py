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
    