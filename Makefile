#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

IDF_PATH = /mnt/h/esp/esp-idf-last-release

PROJECT_NAME := EspFrame

CFLAGS := -DDEBUG_CURL -DMINIZ_NO_TIME -DESP32 -mlongcalls 
CXXFLAGS := -DDEBUG_CURL -DMINIZ_NO_TIME -DESP32 -DZUPPLY_OS_UNIX -DLINUX -Wno-reorder -std=c++14 -mlongcalls 

include $(IDF_PATH)/make/project.mk

