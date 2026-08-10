TARGET = DPT.multimode

BEARAMIN_ROOT ?= ../..
DPT_PLATFORM_ROOT ?= $(abspath $(BEARAMIN_ROOT)/../dpt)
BUILD_DIR ?= $(BEARAMIN_ROOT)/.build/dpt/DPT.multimode
$(shell mkdir -p $(dir $(BUILD_DIR)))

CPP_SOURCES = DPT.multimode.cpp \
$(DPT_PLATFORM_ROOT)/lib/daisy_dpt.cpp \
$(DPT_PLATFORM_ROOT)/lib/dev/DAC7554.cpp \
rings_fm_voice.cpp \
rings_part.cpp \
rings_resonator.cpp \
rings_resources.cpp \
rings_string.cpp \
stmlib_random.cpp \
stmlib_units.cpp

C_INCLUDES += -I$(BEARAMIN_ROOT)/third_party/mutable
C_INCLUDES += -I$(DPT_PLATFORM_ROOT)/lib

USE_FATFS = 1
APP_TYPE = BOOT_QSPI

LIBDAISY_DIR = $(DPT_PLATFORM_ROOT)/libDaisy/
DAISYSP_DIR = $(DPT_PLATFORM_ROOT)/DaisySP/
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
