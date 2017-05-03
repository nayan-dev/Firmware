#
# Makefile for the px4fmu-v2_APM configuration
#
include $(SKETCHBOOK)/mk/PX4/px4_common.mk
UAVCAN_STM32_NUM_IFACES = 1
MODULES		+= drivers/lsm303d
MODULES		+= drivers/l3gd20
MODULES		+= drivers/mpu9250
MODULES		+= drivers/boards/sparrow-v10
MODULES		+= drivers/pwm_input
MODULES         += modules/uavcan
MODULES         += lib/mathlib
#MODULES		+= drivers/px4io
MODULES		+= drivers/px4flow
MODULES		+= drivers/oreoled
