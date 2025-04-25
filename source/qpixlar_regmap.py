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

FPGA_ID_ADDR = 0x1018

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
REG8        = QPIX_OFFSET + 0x8 # reset width control
REG9        = QPIX_OFFSET + 0x9 # sample control2
REGA        = QPIX_OFFSET + 0xA # deltaT select

##################################
## Qpix Control Register Offset ##
##################################
QCTRL_sys_rst        = 0
QCTRL_opad_ext_POR   = 1
QCTRL_pulse_rst_ext1 = 2
QCTRL_pulse_rst_ext2 = 3
QCTRL_calibrate      = 4
QCTRL_rst_ext1       = 5
QCTRL_rst_ext2       = 6
QCTRL_rst_and_trg    = 7
QCTRL_opad_ctrl1     = 8
QCTRL_opad_ctrl2     = 9
QCTRL_opad_cal_ctrl1 = 10
QCTRL_opad_cal_ctrl2 = 11
QCTRL_opad_pul_ctrl1 = 12
QCTRL_opad_pul_ctrl2 = 13
QCTRL_opad_clk_rpen1 = 16
QCTRL_opad_clk_rpen2 = 17
QCTRL_opad_startup1  = 24
QCTRL_opad_startup2  = 25

####################################
## Qpix Pad Control Offset Values ##
####################################
# note that these registers exist for both
# the standard and cgain channels.
QPAD_CTRL_rst               = 0
QPAD_CTRL_load              = 1
QPAD_CTRL_xmit              = 2
QPAD_CTRL_opad_serialOutCnt = 4
QPAD_CTRL_pulse_opad_CLKin2 = 5
QPAD_CTRL_read_data         = 6
QPAD_CTRL_load_data         = 8
QPAD_CTRL_opad_selDefData   = 9  

#########################
## CTRL Register Addrs ##
#########################
# special control values to select ctrl registers
CTRL_SHDN        = 0x4C520000
CTRL_MASK        = 0x4C520001
CTRL_PLEN        = 0x4C520010
CTRL_FPGA_ID     = 0x4C520011
CTRL_FORCE_VALID = 0x4C520100