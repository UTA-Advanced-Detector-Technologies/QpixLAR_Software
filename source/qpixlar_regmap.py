# ASCII 0C2I with endian-ness
I2C_PACKET = 0x00433249
# ASCII 0IPS with endian-ness
SPI_PACKET = 0x00495053
# ASCII XIPQ with endian-ness
QPIX_PACKET = 0x58495051
# ASCII LRTC with endian-ness
CTRL_PACKET = 0x4C525443

# special control values to select ctrl registers
CTRL_SHDN = 0x4C520000
CTRL_MASK = 0x4C520001
CTRL_PLEN = 0x4C520010

# total number of reg_0..N qpix registers
QPIX_NUM_REGS = 9

# define good and packet packets for return types on reg IO
GOOD_PACKET = 0xc0decafe
BAD_PACKET = 0xabadadd5