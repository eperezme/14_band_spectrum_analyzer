# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake.exe

# The command to remove a file.
RM = /usr/bin/cmake.exe -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /cygdrive/c/Users/Familia/Desktop/14_band_spectrum_analyzer-master

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /cygdrive/c/Users/Familia/Desktop/14_band_spectrum_analyzer-master/cmake-build-megaatmega2560

# Utility rule file for PLATFORMIO_BUILD_VERBOSE.

# Include the progress variables for this target.
include CMakeFiles/PLATFORMIO_BUILD_VERBOSE.dir/progress.make

CMakeFiles/PLATFORMIO_BUILD_VERBOSE:
	cd /cygdrive/c/Users/Familia/Desktop/14_band_spectrum_analyzer-master && "C:\\Users\\Familia/AppData/Local/Programs/Python/Python38/Scripts/platformio.exe" -f -c clion run --verbose -emegaatmega2560

PLATFORMIO_BUILD_VERBOSE: CMakeFiles/PLATFORMIO_BUILD_VERBOSE
PLATFORMIO_BUILD_VERBOSE: CMakeFiles/PLATFORMIO_BUILD_VERBOSE.dir/build.make

.PHONY : PLATFORMIO_BUILD_VERBOSE

# Rule to build all files generated by this target.
CMakeFiles/PLATFORMIO_BUILD_VERBOSE.dir/build: PLATFORMIO_BUILD_VERBOSE

.PHONY : CMakeFiles/PLATFORMIO_BUILD_VERBOSE.dir/build

CMakeFiles/PLATFORMIO_BUILD_VERBOSE.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/PLATFORMIO_BUILD_VERBOSE.dir/cmake_clean.cmake
.PHONY : CMakeFiles/PLATFORMIO_BUILD_VERBOSE.dir/clean

CMakeFiles/PLATFORMIO_BUILD_VERBOSE.dir/depend:
	cd /cygdrive/c/Users/Familia/Desktop/14_band_spectrum_analyzer-master/cmake-build-megaatmega2560 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /cygdrive/c/Users/Familia/Desktop/14_band_spectrum_analyzer-master /cygdrive/c/Users/Familia/Desktop/14_band_spectrum_analyzer-master /cygdrive/c/Users/Familia/Desktop/14_band_spectrum_analyzer-master/cmake-build-megaatmega2560 /cygdrive/c/Users/Familia/Desktop/14_band_spectrum_analyzer-master/cmake-build-megaatmega2560 /cygdrive/c/Users/Familia/Desktop/14_band_spectrum_analyzer-master/cmake-build-megaatmega2560/CMakeFiles/PLATFORMIO_BUILD_VERBOSE.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/PLATFORMIO_BUILD_VERBOSE.dir/depend

