# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.21

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
CMAKE_SOURCE_DIR = /home/fbartelt/Documents/UFMG/Planejamento/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/fbartelt/Documents/UFMG/Planejamento/catkin_ws/build

# Include any dependencies generated for this target.
include gtest/CMakeFiles/gmock_main.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include gtest/CMakeFiles/gmock_main.dir/compiler_depend.make

# Include the progress variables for this target.
include gtest/CMakeFiles/gmock_main.dir/progress.make

# Include the compile flags for this target's objects.
include gtest/CMakeFiles/gmock_main.dir/flags.make

gtest/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o: gtest/CMakeFiles/gmock_main.dir/flags.make
gtest/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o: /usr/src/gmock/src/gmock_main.cc
gtest/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o: gtest/CMakeFiles/gmock_main.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/fbartelt/Documents/UFMG/Planejamento/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object gtest/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o"
	cd /home/fbartelt/Documents/UFMG/Planejamento/catkin_ws/build/gtest && /sbin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT gtest/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o -MF CMakeFiles/gmock_main.dir/src/gmock_main.cc.o.d -o CMakeFiles/gmock_main.dir/src/gmock_main.cc.o -c /usr/src/gmock/src/gmock_main.cc

gtest/CMakeFiles/gmock_main.dir/src/gmock_main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gmock_main.dir/src/gmock_main.cc.i"
	cd /home/fbartelt/Documents/UFMG/Planejamento/catkin_ws/build/gtest && /sbin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /usr/src/gmock/src/gmock_main.cc > CMakeFiles/gmock_main.dir/src/gmock_main.cc.i

gtest/CMakeFiles/gmock_main.dir/src/gmock_main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gmock_main.dir/src/gmock_main.cc.s"
	cd /home/fbartelt/Documents/UFMG/Planejamento/catkin_ws/build/gtest && /sbin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /usr/src/gmock/src/gmock_main.cc -o CMakeFiles/gmock_main.dir/src/gmock_main.cc.s

# Object files for target gmock_main
gmock_main_OBJECTS = \
"CMakeFiles/gmock_main.dir/src/gmock_main.cc.o"

# External object files for target gmock_main
gmock_main_EXTERNAL_OBJECTS =

gtest/lib/libgmock_main.so.1.11.0: gtest/CMakeFiles/gmock_main.dir/src/gmock_main.cc.o
gtest/lib/libgmock_main.so.1.11.0: gtest/CMakeFiles/gmock_main.dir/build.make
gtest/lib/libgmock_main.so.1.11.0: gtest/lib/libgmock.so.1.11.0
gtest/lib/libgmock_main.so.1.11.0: gtest/lib/libgtest.so.1.11.0
gtest/lib/libgmock_main.so.1.11.0: gtest/CMakeFiles/gmock_main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/fbartelt/Documents/UFMG/Planejamento/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library lib/libgmock_main.so"
	cd /home/fbartelt/Documents/UFMG/Planejamento/catkin_ws/build/gtest && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gmock_main.dir/link.txt --verbose=$(VERBOSE)
	cd /home/fbartelt/Documents/UFMG/Planejamento/catkin_ws/build/gtest && $(CMAKE_COMMAND) -E cmake_symlink_library lib/libgmock_main.so.1.11.0 lib/libgmock_main.so.1.11.0 lib/libgmock_main.so

gtest/lib/libgmock_main.so: gtest/lib/libgmock_main.so.1.11.0
	@$(CMAKE_COMMAND) -E touch_nocreate gtest/lib/libgmock_main.so

# Rule to build all files generated by this target.
gtest/CMakeFiles/gmock_main.dir/build: gtest/lib/libgmock_main.so
.PHONY : gtest/CMakeFiles/gmock_main.dir/build

gtest/CMakeFiles/gmock_main.dir/clean:
	cd /home/fbartelt/Documents/UFMG/Planejamento/catkin_ws/build/gtest && $(CMAKE_COMMAND) -P CMakeFiles/gmock_main.dir/cmake_clean.cmake
.PHONY : gtest/CMakeFiles/gmock_main.dir/clean

gtest/CMakeFiles/gmock_main.dir/depend:
	cd /home/fbartelt/Documents/UFMG/Planejamento/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/fbartelt/Documents/UFMG/Planejamento/catkin_ws/src /usr/src/gmock /home/fbartelt/Documents/UFMG/Planejamento/catkin_ws/build /home/fbartelt/Documents/UFMG/Planejamento/catkin_ws/build/gtest /home/fbartelt/Documents/UFMG/Planejamento/catkin_ws/build/gtest/CMakeFiles/gmock_main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : gtest/CMakeFiles/gmock_main.dir/depend

