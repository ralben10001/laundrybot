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
include lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/depend.make

# Include the progress variables for this target.
include lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/progress.make

# Include the compile flags for this target's objects.
include lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/flags.make

lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.o: lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/flags.make
lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.o: lib/bullet-2.79/Demos/SoftDemo/main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sanguinus/hojonathanho-bulletsim-56b8d9e/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.o"
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/Demos/SoftDemo && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/AppSoftBodyDemo.dir/main.o -c /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/Demos/SoftDemo/main.cpp

lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AppSoftBodyDemo.dir/main.i"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CXX_CREATE_PREPROCESSED_SOURCE

lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AppSoftBodyDemo.dir/main.s"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CXX_CREATE_ASSEMBLY_SOURCE

lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.o.requires:
.PHONY : lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.o.requires

lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.o.provides: lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.o.requires
	$(MAKE) -f lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/build.make lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.o.provides.build
.PHONY : lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.o.provides

lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.o.provides.build: lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.o
.PHONY : lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.o.provides.build

lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o: lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/flags.make
lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o: lib/bullet-2.79/Demos/SoftDemo/SoftDemo.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sanguinus/hojonathanho-bulletsim-56b8d9e/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o"
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/Demos/SoftDemo && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o -c /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/Demos/SoftDemo/SoftDemo.cpp

lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.i"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CXX_CREATE_PREPROCESSED_SOURCE

lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.s"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CXX_CREATE_ASSEMBLY_SOURCE

lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o.requires:
.PHONY : lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o.requires

lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o.provides: lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o.requires
	$(MAKE) -f lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/build.make lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o.provides.build
.PHONY : lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o.provides

lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o.provides.build: lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o
.PHONY : lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o.provides.build

# Object files for target AppSoftBodyDemo
AppSoftBodyDemo_OBJECTS = \
"CMakeFiles/AppSoftBodyDemo.dir/main.o" \
"CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o"

# External object files for target AppSoftBodyDemo
AppSoftBodyDemo_EXTERNAL_OBJECTS =

lib/bullet-2.79/Demos/SoftDemo/AppSoftBodyDemo: lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.o
lib/bullet-2.79/Demos/SoftDemo/AppSoftBodyDemo: lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o
lib/bullet-2.79/Demos/SoftDemo/AppSoftBodyDemo: lib/bullet-2.79/Demos/OpenGL/libOpenGLSupport.a
lib/bullet-2.79/Demos/SoftDemo/AppSoftBodyDemo: lib/bullet-2.79/src/BulletSoftBody/libBulletSoftBody.a
lib/bullet-2.79/Demos/SoftDemo/AppSoftBodyDemo: lib/bullet-2.79/src/BulletDynamics/libBulletDynamics.a
lib/bullet-2.79/Demos/SoftDemo/AppSoftBodyDemo: lib/bullet-2.79/src/BulletCollision/libBulletCollision.a
lib/bullet-2.79/Demos/SoftDemo/AppSoftBodyDemo: lib/bullet-2.79/src/LinearMath/libLinearMath.a
lib/bullet-2.79/Demos/SoftDemo/AppSoftBodyDemo: /usr/lib/libglut.so
lib/bullet-2.79/Demos/SoftDemo/AppSoftBodyDemo: /usr/lib/libGL.so
lib/bullet-2.79/Demos/SoftDemo/AppSoftBodyDemo: /usr/lib/libGLU.so
lib/bullet-2.79/Demos/SoftDemo/AppSoftBodyDemo: lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/build.make
lib/bullet-2.79/Demos/SoftDemo/AppSoftBodyDemo: lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable AppSoftBodyDemo"
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/Demos/SoftDemo && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/AppSoftBodyDemo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/build: lib/bullet-2.79/Demos/SoftDemo/AppSoftBodyDemo
.PHONY : lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/build

lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/requires: lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/main.o.requires
lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/requires: lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/SoftDemo.o.requires
.PHONY : lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/requires

lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/clean:
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/Demos/SoftDemo && $(CMAKE_COMMAND) -P CMakeFiles/AppSoftBodyDemo.dir/cmake_clean.cmake
.PHONY : lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/clean

lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/depend:
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sanguinus/hojonathanho-bulletsim-56b8d9e /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/Demos/SoftDemo /home/sanguinus/hojonathanho-bulletsim-56b8d9e /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/Demos/SoftDemo /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lib/bullet-2.79/Demos/SoftDemo/CMakeFiles/AppSoftBodyDemo.dir/depend

