NAME = $(notdir $(CURDIR))
PROJECT = $(NAME)
BUILD_DIR = bin

SHARED_DIR = ../common
INCLUDE_DIR = ../include
SRC_FILES := $(notdir $(wildcard $(SHARED_DIR)/*.c))
$(info SRC_FILES =  $(SRC_FILES))
#CFILES = main.c setup.c util.c max31865.c
CFILES = main.c $(SRC_FILES) 
#CFILES += api.c
AFILES += api-asm.S
LDLIBS += -lm
LDFLAGS += -Wl,--print-memory-usage
TGT_LDFLAGS += -u_printf_float

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
