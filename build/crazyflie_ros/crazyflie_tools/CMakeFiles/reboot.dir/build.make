# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jan/apurva_joshi/crazygames_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jan/apurva_joshi/crazygames_ws/build

# Include any dependencies generated for this target.
include crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/depend.make

# Include the progress variables for this target.
include crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/progress.make

# Include the compile flags for this target's objects.
include crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/flags.make

crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/src/reboot.cpp.o: crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/flags.make
crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/src/reboot.cpp.o: /home/jan/apurva_joshi/crazygames_ws/src/crazyflie_ros/crazyflie_tools/src/reboot.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/jan/apurva_joshi/crazygames_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/src/reboot.cpp.o"
	cd /home/jan/apurva_joshi/crazygames_ws/build/crazyflie_ros/crazyflie_tools && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/reboot.dir/src/reboot.cpp.o -c /home/jan/apurva_joshi/crazygames_ws/src/crazyflie_ros/crazyflie_tools/src/reboot.cpp

crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/src/reboot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/reboot.dir/src/reboot.cpp.i"
	cd /home/jan/apurva_joshi/crazygames_ws/build/crazyflie_ros/crazyflie_tools && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/jan/apurva_joshi/crazygames_ws/src/crazyflie_ros/crazyflie_tools/src/reboot.cpp > CMakeFiles/reboot.dir/src/reboot.cpp.i

crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/src/reboot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/reboot.dir/src/reboot.cpp.s"
	cd /home/jan/apurva_joshi/crazygames_ws/build/crazyflie_ros/crazyflie_tools && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/jan/apurva_joshi/crazygames_ws/src/crazyflie_ros/crazyflie_tools/src/reboot.cpp -o CMakeFiles/reboot.dir/src/reboot.cpp.s

crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/src/reboot.cpp.o.requires:
.PHONY : crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/src/reboot.cpp.o.requires

crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/src/reboot.cpp.o.provides: crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/src/reboot.cpp.o.requires
	$(MAKE) -f crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/build.make crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/src/reboot.cpp.o.provides.build
.PHONY : crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/src/reboot.cpp.o.provides

crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/src/reboot.cpp.o.provides.build: crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/src/reboot.cpp.o

# Object files for target reboot
reboot_OBJECTS = \
"CMakeFiles/reboot.dir/src/reboot.cpp.o"

# External object files for target reboot
reboot_EXTERNAL_OBJECTS =

/home/jan/apurva_joshi/crazygames_ws/devel/lib/crazyflie_tools/reboot: crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/src/reboot.cpp.o
/home/jan/apurva_joshi/crazygames_ws/devel/lib/crazyflie_tools/reboot: crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/build.make
/home/jan/apurva_joshi/crazygames_ws/devel/lib/crazyflie_tools/reboot: /home/jan/apurva_joshi/crazygames_ws/devel/lib/libcrazyflie_cpp.so
/home/jan/apurva_joshi/crazygames_ws/devel/lib/crazyflie_tools/reboot: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/jan/apurva_joshi/crazygames_ws/devel/lib/crazyflie_tools/reboot: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
/home/jan/apurva_joshi/crazygames_ws/devel/lib/crazyflie_tools/reboot: crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/jan/apurva_joshi/crazygames_ws/devel/lib/crazyflie_tools/reboot"
	cd /home/jan/apurva_joshi/crazygames_ws/build/crazyflie_ros/crazyflie_tools && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/reboot.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/build: /home/jan/apurva_joshi/crazygames_ws/devel/lib/crazyflie_tools/reboot
.PHONY : crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/build

crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/requires: crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/src/reboot.cpp.o.requires
.PHONY : crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/requires

crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/clean:
	cd /home/jan/apurva_joshi/crazygames_ws/build/crazyflie_ros/crazyflie_tools && $(CMAKE_COMMAND) -P CMakeFiles/reboot.dir/cmake_clean.cmake
.PHONY : crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/clean

crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/depend:
	cd /home/jan/apurva_joshi/crazygames_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jan/apurva_joshi/crazygames_ws/src /home/jan/apurva_joshi/crazygames_ws/src/crazyflie_ros/crazyflie_tools /home/jan/apurva_joshi/crazygames_ws/build /home/jan/apurva_joshi/crazygames_ws/build/crazyflie_ros/crazyflie_tools /home/jan/apurva_joshi/crazygames_ws/build/crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : crazyflie_ros/crazyflie_tools/CMakeFiles/reboot.dir/depend

