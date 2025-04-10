# ASCII 0C2I with endian-ness
I2C_PACKET = 0x00433249
# ASCII 0IPS with endian-ness
SPI_PACKET = 0x00495053
# ASCII XIPQ with endian-ness
QPIX_PACKET = 0x58495051
# ASCII LRTC with endian-ness
CTRL_PACKET = 0x4C525443


# i2c controls
IIC_SLAVE_ADDR_1 = 0x0C
IIC_SLAVE_ADDR_2 = 0x0D

# generally want PD0 = PD1 = 0, and bCLR = bLDAC = 1, or 0x3
IIC_CTRL_DEFAULT = 0x2 

# total number of reg_0..N qpix registers
QPIX_NUM_REGS = 9

# define good and packet packets for return types on reg IO
GOOD_PACKET = 0xc0decafe
BAD_PACKET = 0xabadadd5

#########################
## Qpix Register Addrs ##
#########################
# Note, there are 9 unique QPix register addresses implemented at Penn,
# However, some of these are unused in this implementation because of an optimization 
# in the data readout. The list of registers below represent a mapping between 
# the magic number system used at Penn towards a more standard approach.
QPIX_OFFSET = 0x000
REG0        = QPIX_OFFSET + 0x0 # ctrl reg
REG1        = QPIX_OFFSET + 0x1 # data1 ctrl
REG2        = QPIX_OFFSET + 0x2 # data1
REG3        = QPIX_OFFSET + 0x3 # data2 ctrl
REG4        = QPIX_OFFSET + 0x4 # data2
REG5        = QPIX_OFFSET + 0x5 # trigger control - UNUSED
REG6        = QPIX_OFFSET + 0x6 # fifo control    - UNUSED
REG7        = QPIX_OFFSET + 0x7 # sample control
REG8        = QPIX_OFFSET + 0x8 # reset control
REG9        = QPIX_OFFSET + 0x9 # sample control2
REGA        = QPIX_OFFSET + 0xA # deltaT select

#########################
## CTRL Register Addrs ##
#########################
# special control values to select ctrl registers
CTRL_SHDN = 0x4C520000
CTRL_MASK = 0x4C520001
CTRL_PLEN = 0x4C520010