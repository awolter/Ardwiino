# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/sanjay/Code/Ardwiino

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sanjay/Code/Ardwiino/buildDebug

# Utility rule file for ardwiino-a-micro-atmega32u4-8000000-rf.

# Include any custom commands dependencies for this target.
include CMakeFiles/ardwiino-a-micro-atmega32u4-8000000-rf.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/ardwiino-a-micro-atmega32u4-8000000-rf.dir/progress.make

ardwiino-a-micro-atmega32u4-8000000-rf: CMakeFiles/ardwiino-a-micro-atmega32u4-8000000-rf.dir/build.make
	cd /home/sanjay/Code/Ardwiino/src/avr/micro/rf && make OBJDIR=obj/ardwiino-a-micro-atmega32u4-8000000-rf F_USB=8000000 F_CPU=8000000 ARDUINO_MODEL_PID= ARDWIINO_BOARD=a-micro EXTRA=-rf TARGET=/home/sanjay/Code/Ardwiino/buildDebug/firmware/ardwiino-a-micro-atmega32u4-8000000-rf MCU=atmega32u4 VARIANT=micro
.PHONY : ardwiino-a-micro-atmega32u4-8000000-rf

# Rule to build all files generated by this target.
CMakeFiles/ardwiino-a-micro-atmega32u4-8000000-rf.dir/build: ardwiino-a-micro-atmega32u4-8000000-rf
.PHONY : CMakeFiles/ardwiino-a-micro-atmega32u4-8000000-rf.dir/build

CMakeFiles/ardwiino-a-micro-atmega32u4-8000000-rf.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ardwiino-a-micro-atmega32u4-8000000-rf.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ardwiino-a-micro-atmega32u4-8000000-rf.dir/clean

CMakeFiles/ardwiino-a-micro-atmega32u4-8000000-rf.dir/depend:
	cd /home/sanjay/Code/Ardwiino/buildDebug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sanjay/Code/Ardwiino /home/sanjay/Code/Ardwiino /home/sanjay/Code/Ardwiino/buildDebug /home/sanjay/Code/Ardwiino/buildDebug /home/sanjay/Code/Ardwiino/buildDebug/CMakeFiles/ardwiino-a-micro-atmega32u4-8000000-rf.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ardwiino-a-micro-atmega32u4-8000000-rf.dir/depend

