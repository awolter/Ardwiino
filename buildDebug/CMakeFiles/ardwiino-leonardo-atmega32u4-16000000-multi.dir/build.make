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

# Utility rule file for ardwiino-leonardo-atmega32u4-16000000-multi.

# Include any custom commands dependencies for this target.
include CMakeFiles/ardwiino-leonardo-atmega32u4-16000000-multi.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/ardwiino-leonardo-atmega32u4-16000000-multi.dir/progress.make

ardwiino-leonardo-atmega32u4-16000000-multi: CMakeFiles/ardwiino-leonardo-atmega32u4-16000000-multi.dir/build.make
	cd /home/sanjay/Code/Ardwiino/src/avr/micro/main && make OBJDIR=obj/ardwiino-leonardo-atmega32u4-16000000-multi F_USB=16000000 F_CPU=16000000 ARDUINO_MODEL_PID= ARDWIINO_BOARD=leonardo EXTRA=-multi TARGET=/home/sanjay/Code/Ardwiino/buildDebug/firmware/ardwiino-leonardo-atmega32u4-16000000-multi MCU=atmega32u4 VARIANT=micro
.PHONY : ardwiino-leonardo-atmega32u4-16000000-multi

# Rule to build all files generated by this target.
CMakeFiles/ardwiino-leonardo-atmega32u4-16000000-multi.dir/build: ardwiino-leonardo-atmega32u4-16000000-multi
.PHONY : CMakeFiles/ardwiino-leonardo-atmega32u4-16000000-multi.dir/build

CMakeFiles/ardwiino-leonardo-atmega32u4-16000000-multi.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ardwiino-leonardo-atmega32u4-16000000-multi.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ardwiino-leonardo-atmega32u4-16000000-multi.dir/clean

CMakeFiles/ardwiino-leonardo-atmega32u4-16000000-multi.dir/depend:
	cd /home/sanjay/Code/Ardwiino/buildDebug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sanjay/Code/Ardwiino /home/sanjay/Code/Ardwiino /home/sanjay/Code/Ardwiino/buildDebug /home/sanjay/Code/Ardwiino/buildDebug /home/sanjay/Code/Ardwiino/buildDebug/CMakeFiles/ardwiino-leonardo-atmega32u4-16000000-multi.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ardwiino-leonardo-atmega32u4-16000000-multi.dir/depend

