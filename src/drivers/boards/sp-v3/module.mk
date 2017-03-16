#
# Board-specific startup code for the PX4FMUv2
#

SRCS		 = spv3_can.c \
		   spv3_init.c \
		   spv3_timer_config.c \
		   spv3_spi.c \
		   spv3_usb.c \
		   spv3_led.c

MAXOPTIMIZATION	 = -Os
