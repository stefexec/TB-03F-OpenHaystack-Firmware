TEL_CHIP := -DCHIP_TYPE=CHIP_TYPE_8258
TEL_PATH := SDK/
LIBS :=  -llt_8258

PROJECT_NAME := FindMy

# 设置下载固件的串口号
DOWNLOAD_PORT := /dev/ttyUSB0

PROJECT_PATH := .
OUT_PATH :=$(PROJECT_PATH)/out

ifneq ($(TEL_PATH)/make/makefile, $(wildcard $(TEL_PATH)/make/makefile))
$(error "Please check if the TEL_PATH is correct")
endif

#include the SDK makefile
-include $(TEL_PATH)/make/makefile
