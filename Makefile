#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := paxcounter
IDF_PATH := $(abspath .)/components/esp-idf/
EXTRA_COMPONENT_DIRS := $(abspath .)/src 

CFLAGS :=  -include "$(abspath .)/src/paxcounter.conf" \
           -include "$(abspath .)/src/hal/wipy.h"

CPPFLAGS := $(CFLAGS)

CXXFLAGS := $(CFLAGS)

include $(IDF_PATH)/make/project.mk

