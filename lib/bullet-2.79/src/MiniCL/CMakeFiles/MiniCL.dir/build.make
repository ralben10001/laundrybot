# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/sanguinus/hojonathanho-bulletsim-56b8d9e

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sanguinus/hojonathanho-bulletsim-56b8d9e

# Include any dependencies generated for this target.
include lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/depend.make

# Include the progress variables for this target.
include lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/progress.make

# Include the compile flags for this target's objects.
include lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/flags.make

lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCL.o: lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/flags.make
lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCL.o: lib/bullet-2.79/src/MiniCL/MiniCL.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sanguinus/hojonathanho-bulletsim-56b8d9e/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCL.o"
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/src/MiniCL && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/MiniCL.dir/MiniCL.o -c /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/src/MiniCL/MiniCL.cpp

lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCL.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MiniCL.dir/MiniCL.i"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CXX_CREATE_PREPROCESSED_SOURCE

lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCL.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MiniCL.dir/MiniCL.s"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CXX_CREATE_ASSEMBLY_SOURCE

lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCL.o.requires:
.PHONY : lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCL.o.requires

lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCL.o.provides: lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCL.o.requires
	$(MAKE) -f lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/build.make lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCL.o.provides.build
.PHONY : lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCL.o.provides

lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCL.o.provides.build: lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCL.o
.PHONY : lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCL.o.provides.build

lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTaskScheduler.o: lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/flags.make
lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTaskScheduler.o: lib/bullet-2.79/src/MiniCL/MiniCLTaskScheduler.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sanguinus/hojonathanho-bulletsim-56b8d9e/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTaskScheduler.o"
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/src/MiniCL && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/MiniCL.dir/MiniCLTaskScheduler.o -c /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/src/MiniCL/MiniCLTaskScheduler.cpp

lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTaskScheduler.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MiniCL.dir/MiniCLTaskScheduler.i"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CXX_CREATE_PREPROCESSED_SOURCE

lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTaskScheduler.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MiniCL.dir/MiniCLTaskScheduler.s"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CXX_CREATE_ASSEMBLY_SOURCE

lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTaskScheduler.o.requires:
.PHONY : lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTaskScheduler.o.requires

lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTaskScheduler.o.provides: lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTaskScheduler.o.requires
	$(MAKE) -f lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/build.make lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTaskScheduler.o.provides.build
.PHONY : lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTaskScheduler.o.provides

lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTaskScheduler.o.provides.build: lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTaskScheduler.o
.PHONY : lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTaskScheduler.o.provides.build

lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTask/MiniCLTask.o: lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/flags.make
lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTask/MiniCLTask.o: lib/bullet-2.79/src/MiniCL/MiniCLTask/MiniCLTask.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sanguinus/hojonathanho-bulletsim-56b8d9e/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTask/MiniCLTask.o"
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/src/MiniCL && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/MiniCL.dir/MiniCLTask/MiniCLTask.o -c /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/src/MiniCL/MiniCLTask/MiniCLTask.cpp

lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTask/MiniCLTask.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MiniCL.dir/MiniCLTask/MiniCLTask.i"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CXX_CREATE_PREPROCESSED_SOURCE

lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTask/MiniCLTask.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MiniCL.dir/MiniCLTask/MiniCLTask.s"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CXX_CREATE_ASSEMBLY_SOURCE

lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTask/MiniCLTask.o.requires:
.PHONY : lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTask/MiniCLTask.o.requires

lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTask/MiniCLTask.o.provides: lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTask/MiniCLTask.o.requires
	$(MAKE) -f lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/build.make lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTask/MiniCLTask.o.provides.build
.PHONY : lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTask/MiniCLTask.o.provides

lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTask/MiniCLTask.o.provides.build: lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTask/MiniCLTask.o
.PHONY : lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTask/MiniCLTask.o.provides.build

# Object files for target MiniCL
MiniCL_OBJECTS = \
"CMakeFiles/MiniCL.dir/MiniCL.o" \
"CMakeFiles/MiniCL.dir/MiniCLTaskScheduler.o" \
"CMakeFiles/MiniCL.dir/MiniCLTask/MiniCLTask.o"

# External object files for target MiniCL
MiniCL_EXTERNAL_OBJECTS =

lib/bullet-2.79/src/MiniCL/libMiniCL.a: lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCL.o
lib/bullet-2.79/src/MiniCL/libMiniCL.a: lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTaskScheduler.o
lib/bullet-2.79/src/MiniCL/libMiniCL.a: lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTask/MiniCLTask.o
lib/bullet-2.79/src/MiniCL/libMiniCL.a: lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/build.make
lib/bullet-2.79/src/MiniCL/libMiniCL.a: lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library libMiniCL.a"
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/src/MiniCL && $(CMAKE_COMMAND) -P CMakeFiles/MiniCL.dir/cmake_clean_target.cmake
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/src/MiniCL && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/MiniCL.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/build: lib/bullet-2.79/src/MiniCL/libMiniCL.a
.PHONY : lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/build

lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/requires: lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCL.o.requires
lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/requires: lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTaskScheduler.o.requires
lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/requires: lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/MiniCLTask/MiniCLTask.o.requires
.PHONY : lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/requires

lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/clean:
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/src/MiniCL && $(CMAKE_COMMAND) -P CMakeFiles/MiniCL.dir/cmake_clean.cmake
.PHONY : lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/clean

lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/depend:
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sanguinus/hojonathanho-bulletsim-56b8d9e /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/src/MiniCL /home/sanguinus/hojonathanho-bulletsim-56b8d9e /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/src/MiniCL /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lib/bullet-2.79/src/MiniCL/CMakeFiles/MiniCL.dir/depend

