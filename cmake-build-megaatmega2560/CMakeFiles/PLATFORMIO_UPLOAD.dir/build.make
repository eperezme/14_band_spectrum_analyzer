# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = "/Users/eduard/Library/Application Support/JetBrains/Toolbox/apps/CLion/ch-0/193.6015.37/CLion.app/Contents/bin/cmake/mac/bin/cmake"

# The command to remove a file.
RM = "/Users/eduard/Library/Application Support/JetBrains/Toolbox/apps/CLion/ch-0/193.6015.37/CLion.app/Contents/bin/cmake/mac/bin/cmake" -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/eduard/CLion_Projects/14_band_spectrum_analyzer/V2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/eduard/CLion_Projects/14_band_spectrum_analyzer/V2/cmake-build-megaatmega2560

# Utility rule file for PLATFORMIO_UPLOAD.

# Include the progress variables for this target.
include CMakeFiles/PLATFORMIO_UPLOAD.dir/progress.make

CMakeFiles/PLATFORMIO_UPLOAD:
	cd /Users/eduard/CLion_Projects/14_band_spectrum_analyzer/V2 && /usr/local/bin/platformio -f -c clion run --target upload -emegaatmega2560

PLATFORMIO_UPLOAD: CMakeFiles/PLATFORMIO_UPLOAD
PLATFORMIO_UPLOAD: CMakeFiles/PLATFORMIO_UPLOAD.dir/build.make

.PHONY : PLATFORMIO_UPLOAD

# Rule to build all files generated by this target.
CMakeFiles/PLATFORMIO_UPLOAD.dir/build: PLATFORMIO_UPLOAD

.PHONY : CMakeFiles/PLATFORMIO_UPLOAD.dir/build

CMakeFiles/PLATFORMIO_UPLOAD.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/PLATFORMIO_UPLOAD.dir/cmake_clean.cmake
.PHONY : CMakeFiles/PLATFORMIO_UPLOAD.dir/clean

CMakeFiles/PLATFORMIO_UPLOAD.dir/depend:
	cd /Users/eduard/CLion_Projects/14_band_spectrum_analyzer/V2/cmake-build-megaatmega2560 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/eduard/CLion_Projects/14_band_spectrum_analyzer/V2 /Users/eduard/CLion_Projects/14_band_spectrum_analyzer/V2 /Users/eduard/CLion_Projects/14_band_spectrum_analyzer/V2/cmake-build-megaatmega2560 /Users/eduard/CLion_Projects/14_band_spectrum_analyzer/V2/cmake-build-megaatmega2560 /Users/eduard/CLion_Projects/14_band_spectrum_analyzer/V2/cmake-build-megaatmega2560/CMakeFiles/PLATFORMIO_UPLOAD.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/PLATFORMIO_UPLOAD.dir/depend

