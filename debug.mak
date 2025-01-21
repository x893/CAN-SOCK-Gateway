BINARYDIR := build

#Toolchain
CC := $(TOOLCHAIN_ROOT)gcc
CXX := $(TOOLCHAIN_ROOT)g++
LD := $(CXX)
AR := $(TOOLCHAIN_ROOT)ar
OBJCOPY := $(TOOLCHAIN_ROOT)objcopy

#Additional flags
PREPROCESSOR_MACROS := DEBUG=1
INCLUDE_DIRS := 
LIBRARY_DIRS := 
LIBRARY_NAMES := pthread
MACOS_FRAMEWORKS := 

CFLAGS := -ggdb -ffunction-sections -O0
CXXFLAGS := -ggdb -ffunction-sections -O0
ASFLAGS := 
LDFLAGS := -Wl,-gc-sections
COMMONFLAGS := 
LINKER_SCRIPT := 

START_GROUP := -Wl,--start-group
END_GROUP := -Wl,--end-group

#Additional options detected from testing the toolchain
IS_LINUX_PROJECT := 1
