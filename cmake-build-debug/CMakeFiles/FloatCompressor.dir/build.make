# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.8

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
CMAKE_COMMAND = /Applications/CLion.app/Contents/bin/cmake/bin/cmake

# The command to remove a file.
RM = /Applications/CLion.app/Contents/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/jeff.niu/FloatCompressor

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/jeff.niu/FloatCompressor/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/FloatCompressor.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/FloatCompressor.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/FloatCompressor.dir/flags.make

CMakeFiles/FloatCompressor.dir/main.cpp.o: CMakeFiles/FloatCompressor.dir/flags.make
CMakeFiles/FloatCompressor.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/jeff.niu/FloatCompressor/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/FloatCompressor.dir/main.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/FloatCompressor.dir/main.cpp.o -c /Users/jeff.niu/FloatCompressor/main.cpp

CMakeFiles/FloatCompressor.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/FloatCompressor.dir/main.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/jeff.niu/FloatCompressor/main.cpp > CMakeFiles/FloatCompressor.dir/main.cpp.i

CMakeFiles/FloatCompressor.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/FloatCompressor.dir/main.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/jeff.niu/FloatCompressor/main.cpp -o CMakeFiles/FloatCompressor.dir/main.cpp.s

CMakeFiles/FloatCompressor.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/FloatCompressor.dir/main.cpp.o.requires

CMakeFiles/FloatCompressor.dir/main.cpp.o.provides: CMakeFiles/FloatCompressor.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/FloatCompressor.dir/build.make CMakeFiles/FloatCompressor.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/FloatCompressor.dir/main.cpp.o.provides

CMakeFiles/FloatCompressor.dir/main.cpp.o.provides.build: CMakeFiles/FloatCompressor.dir/main.cpp.o


# Object files for target FloatCompressor
FloatCompressor_OBJECTS = \
"CMakeFiles/FloatCompressor.dir/main.cpp.o"

# External object files for target FloatCompressor
FloatCompressor_EXTERNAL_OBJECTS =

FloatCompressor: CMakeFiles/FloatCompressor.dir/main.cpp.o
FloatCompressor: CMakeFiles/FloatCompressor.dir/build.make
FloatCompressor: CMakeFiles/FloatCompressor.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/jeff.niu/FloatCompressor/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable FloatCompressor"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/FloatCompressor.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/FloatCompressor.dir/build: FloatCompressor

.PHONY : CMakeFiles/FloatCompressor.dir/build

CMakeFiles/FloatCompressor.dir/requires: CMakeFiles/FloatCompressor.dir/main.cpp.o.requires

.PHONY : CMakeFiles/FloatCompressor.dir/requires

CMakeFiles/FloatCompressor.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/FloatCompressor.dir/cmake_clean.cmake
.PHONY : CMakeFiles/FloatCompressor.dir/clean

CMakeFiles/FloatCompressor.dir/depend:
	cd /Users/jeff.niu/FloatCompressor/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/jeff.niu/FloatCompressor /Users/jeff.niu/FloatCompressor /Users/jeff.niu/FloatCompressor/cmake-build-debug /Users/jeff.niu/FloatCompressor/cmake-build-debug /Users/jeff.niu/FloatCompressor/cmake-build-debug/CMakeFiles/FloatCompressor.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/FloatCompressor.dir/depend

