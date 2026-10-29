SDK_ROOT ?= ../../lab_hd_1_mipi_hdmi/FDwork/anlu
CHIP := ph1p35
DOWNLOAD := tcm
OUTPUT_DEV := USART1
COMPILE_PREFIX ?= riscv64-unknown-elf-
PLAT_DIR := $(SDK_ROOT)/platform

APP_CUR_DIR := $(abspath .)
APP_SRC_DIR := $(APP_CUR_DIR)
APP_INC_DIR := $(APP_CUR_DIR)
APP_OBJ_DIR := $(APP_TAR_DIR)/obj
APP_TAR_DIR ?= $(APP_CUR_DIR)/build
TARGET_PATH ?= $(APP_CUR_DIR)

TARGET := ph1p35_digit_uart
CFLAGS += -Os -g -Wall -Wextra
LINKER_SCRIPT := $(APP_CUR_DIR)/lds/gcc_ph1p35_tcm.ld
LD_LIBS += $(APP_TAR_DIR)/lib/libalbsp.a
LD_LIBS += $(APP_TAR_DIR)/lib/libplat.a
LD_LIBS += $(APP_TAR_DIR)/lib/libsemihost.a

include $(SDK_ROOT)/tools/make/rules.mk
