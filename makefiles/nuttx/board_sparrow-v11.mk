#
# Board-specific definitions for the PX4FMUv2
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = CORTEXM4F
CONFIG_BOARD			 = SPARROW_V11

include $(PX4_MK_DIR)/toolchain_gnu-arm-eabi.mk
