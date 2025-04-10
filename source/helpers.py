## store some helper function methods used to calculate bit masking or other useful things here.
import numpy as np
import struct

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