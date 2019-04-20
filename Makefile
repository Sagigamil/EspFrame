#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := EspFrame

CFLAGS := -DMINIZ_NO_TIME -DESP32
CPPFLAGS := -DMINIZ_NO_TIME -DESP32

include $(IDF_PATH)/make/project.mk

