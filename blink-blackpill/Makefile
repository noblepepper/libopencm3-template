
NAME = $(notdir $(CURDIR))
PROJECT = $(NAME)
BUILD_DIR = bin

SHARED_DIR = ../common
INCLUDE_DIR = ../include
CFILES = blink.c setup.c util.c 
CFILES += api.c
AFILES += api-asm.S

# TODO - you will need to edit these two lines!
DEVICE=stm32f411ce
OOCD_FILE = board/stm32f4discovery.cfg

# You shouldn't have to edit anything below here.
VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
INCLUDES += $(patsubst %,-I%, . $(INCLUDE_DIR))
OPENCM3_DIR=../libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ../rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk
