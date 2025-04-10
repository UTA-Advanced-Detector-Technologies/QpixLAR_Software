## store some helper function methods used to calculate bit masking or other useful things here.
import numpy as np
import struct
import qpixlar_regmap as REG

ZTURN_FCLK0 = 200e6
ZTURN_FCLK1 = 50e6

def reverse_bits(num, bit_size):
    """
    reverse bit value of an integer num, considered to have number of bits equal
    to bit_size
    """
    # Convert the number to a binary string with fixed length
    bit_str = f"{num:0{bit_size}b}"  # Formats number as a bit_size-bit binary string
    reversed_bit_str = bit_str[::-1]  # Reverse the bit order
    
    # Convert back to an integer
    return int(reversed_bit_str, 2)

class SerialConfig():
    """
    Wrapper class to manage the injection of the serial configuration
    for each of the pads in the qpix ASIC.
    These default values are taken from the qpixserialinterface.xlsx
    """
    def __init__(self, calibration=False):
        # bit reversal for replen_cur handled in algorithm
        self.replen_cur    = 0b10101 if not calibration else 0b10101
        self.curReplen     = reverse_bits(0b010, 3) if not calibration else reverse_bits(0b011, 3)
        self.curCmp        = reverse_bits(0b011, 3) if not calibration else reverse_bits(0b011, 3)
        self.curAmp        = reverse_bits(0b011, 3) if not calibration else reverse_bits(0b011, 3)
        self.curInt        = reverse_bits(0b011, 3) if not calibration else reverse_bits(0b011, 3)
        self.enable        = reverse_bits(0x03,  8) if not calibration else reverse_bits(0b011, 3)
        self.clk_source_ro = True if not calibration else True
        self.dbl_bar       = False if not calibration else True
        self.enableRingOsc = True if not calibration else True
        self.ringOsc_f     = reverse_bits(0b10,  8) if not calibration else reverse_bits(0b00,  8)
        self.LVDS_drvr     = True if not calibration else True
        self.enableCalB    = True if not calibration else False

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

def make_serial_word(calibration=False):
    """
    wrapper function to help prevent the shameful implmentation done by Penn..
    NOTE: under no circumstances should running python code (after 5+ years
    of development) require an external excel sheet to run, whereby a user 
    copies produced magic numbers into a string to control the DUT..

    We should all take this opportunity to reflect on the need to take pride in
    our work.
    """
    a = SerialConfig(calibration=calibration)
    data_word = 0

    # this is a fun feature
    val = (a.replen_cur & (0xf << 1))
    repl_top = reverse_bits((val >> 1), 4)
    repl_bot =  a.replen_cur & 0b1

    # build the data word
    data_word |= repl_top               << (31 -  3)
    data_word |= (a.curReplen & 0x7)    << (31 -  6)
    data_word |= (a.curCmp    & 0x7)    << (31 -  9)
    data_word |= (a.curAmp    & 0x7)    << (31 - 12)
    data_word |= (a.curInt    & 0x7)    << (31 - 15)
    data_word |= (a.enable    & 0xff)   << (31 - 23)
    data_word |= (repl_bot          )   << (31 - 24)
    data_word |= (int(a.clk_source_ro)) << (31 - 25)
    data_word |= (int(a.dbl_bar))       << (31 - 26)
    data_word |= (a.ringOsc_f & 0x3)    << (31 - 28)
    data_word |= (int(a.enableRingOsc)) << (31 - 29)
    data_word |= (int(a.LVDS_drvr))     << (31 - 30)
    data_word |= (int(a.enableCalB))    << (31 - 31)

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


def main():
    """
    Test generic functions easily here by running this through a debugger.
    """
    data_word = make_serial_word()
    print(f"data_word = {data_word:08x}")


if __name__ == "__main__":
    main()