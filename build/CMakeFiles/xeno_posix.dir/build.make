# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/az/Documents/git-STSMC/STSMC

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/az/Documents/git-STSMC/STSMC/build

# Include any dependencies generated for this target.
include CMakeFiles/xeno_posix.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/xeno_posix.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/xeno_posix.dir/flags.make

CMakeFiles/xeno_posix.dir/main.cpp.o: CMakeFiles/xeno_posix.dir/flags.make
CMakeFiles/xeno_posix.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/az/Documents/git-STSMC/STSMC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/xeno_posix.dir/main.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/xeno_posix.dir/main.cpp.o -c /home/az/Documents/git-STSMC/STSMC/main.cpp

CMakeFiles/xeno_posix.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xeno_posix.dir/main.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/az/Documents/git-STSMC/STSMC/main.cpp > CMakeFiles/xeno_posix.dir/main.cpp.i

CMakeFiles/xeno_posix.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xeno_posix.dir/main.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/az/Documents/git-STSMC/STSMC/main.cpp -o CMakeFiles/xeno_posix.dir/main.cpp.s

CMakeFiles/xeno_posix.dir/source/Nano.cpp.o: CMakeFiles/xeno_posix.dir/flags.make
CMakeFiles/xeno_posix.dir/source/Nano.cpp.o: ../source/Nano.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/az/Documents/git-STSMC/STSMC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/xeno_posix.dir/source/Nano.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/xeno_posix.dir/source/Nano.cpp.o -c /home/az/Documents/git-STSMC/STSMC/source/Nano.cpp

CMakeFiles/xeno_posix.dir/source/Nano.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xeno_posix.dir/source/Nano.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/az/Documents/git-STSMC/STSMC/source/Nano.cpp > CMakeFiles/xeno_posix.dir/source/Nano.cpp.i

CMakeFiles/xeno_posix.dir/source/Nano.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xeno_posix.dir/source/Nano.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/az/Documents/git-STSMC/STSMC/source/Nano.cpp -o CMakeFiles/xeno_posix.dir/source/Nano.cpp.s

# Object files for target xeno_posix
xeno_posix_OBJECTS = \
"CMakeFiles/xeno_posix.dir/main.cpp.o" \
"CMakeFiles/xeno_posix.dir/source/Nano.cpp.o"

# External object files for target xeno_posix
xeno_posix_EXTERNAL_OBJECTS =

xeno_posix: CMakeFiles/xeno_posix.dir/main.cpp.o
xeno_posix: CMakeFiles/xeno_posix.dir/source/Nano.cpp.o
xeno_posix: CMakeFiles/xeno_posix.dir/build.make
xeno_posix: /usr/local/lib/libethercat.so
xeno_posix: CMakeFiles/xeno_posix.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/az/Documents/git-STSMC/STSMC/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable xeno_posix"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/xeno_posix.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/xeno_posix.dir/build: xeno_posix

.PHONY : CMakeFiles/xeno_posix.dir/build

CMakeFiles/xeno_posix.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/xeno_posix.dir/cmake_clean.cmake
.PHONY : CMakeFiles/xeno_posix.dir/clean

CMakeFiles/xeno_posix.dir/depend:
	cd /home/az/Documents/git-STSMC/STSMC/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/az/Documents/git-STSMC/STSMC /home/az/Documents/git-STSMC/STSMC /home/az/Documents/git-STSMC/STSMC/build /home/az/Documents/git-STSMC/STSMC/build /home/az/Documents/git-STSMC/STSMC/build/CMakeFiles/xeno_posix.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/xeno_posix.dir/depend

