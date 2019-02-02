#Generated by VisualGDB (http://visualgdb.com)
#DO NOT EDIT THIS FILE MANUALLY UNLESS YOU ABSOLUTELY NEED TO
#USE VISUALGDB PROJECT PROPERTIES DIALOG INSTEAD

BINARYDIR := Release

#Toolchain
CC := C:/SysGCC/raspberry/bin/arm-linux-gnueabihf-gcc.exe
CXX := C:/SysGCC/raspberry/bin/arm-linux-gnueabihf-g++.exe
LD := $(CXX)
AR := C:/SysGCC/raspberry/bin/arm-linux-gnueabihf-ar.exe
OBJCOPY := C:/SysGCC/raspberry/bin/arm-linux-gnueabihf-objcopy.exe

#Additional flags
PREPROCESSOR_MACROS := NDEBUG=1 RELEASE=1
INCLUDE_DIRS := C:\SysGCC\raspberry\arm-linux-gnueabihf\sysroot\usr\local\include\opencv4
LIBRARY_DIRS := 
LIBRARY_NAMES := opencv_core opencv_imgproc opencv_video opencv_videoio opencv_features2d opencv_highgui opencv_objdetect
ADDITIONAL_LINKER_INPUTS := 
MACOS_FRAMEWORKS := 
LINUX_PACKAGES := 

CFLAGS := -ggdb -ffunction-sections -O3
CXXFLAGS := -ggdb -ffunction-sections -O3
ASFLAGS := 
LDFLAGS := -Wl,-gc-sections
COMMONFLAGS := 
LINKER_SCRIPT := 

START_GROUP := -Wl,--start-group
END_GROUP := -Wl,--end-group

#Additional options detected from testing the toolchain
USE_DEL_TO_CLEAN := 1
CP_NOT_AVAILABLE := 1
IS_LINUX_PROJECT := 1
