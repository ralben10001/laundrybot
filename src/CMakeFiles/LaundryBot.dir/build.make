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
include src/CMakeFiles/LaundryBot.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/LaundryBot.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/LaundryBot.dir/flags.make

src/CMakeFiles/LaundryBot.dir/LaundryBot.cpp.o: src/CMakeFiles/LaundryBot.dir/flags.make
src/CMakeFiles/LaundryBot.dir/LaundryBot.cpp.o: src/LaundryBot.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sanguinus/hojonathanho-bulletsim-56b8d9e/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/LaundryBot.dir/LaundryBot.cpp.o"
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/LaundryBot.dir/LaundryBot.cpp.o -c /home/sanguinus/hojonathanho-bulletsim-56b8d9e/src/LaundryBot.cpp

src/CMakeFiles/LaundryBot.dir/LaundryBot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LaundryBot.dir/LaundryBot.cpp.i"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CXX_CREATE_PREPROCESSED_SOURCE

src/CMakeFiles/LaundryBot.dir/LaundryBot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LaundryBot.dir/LaundryBot.cpp.s"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CXX_CREATE_ASSEMBLY_SOURCE

src/CMakeFiles/LaundryBot.dir/LaundryBot.cpp.o.requires:
.PHONY : src/CMakeFiles/LaundryBot.dir/LaundryBot.cpp.o.requires

src/CMakeFiles/LaundryBot.dir/LaundryBot.cpp.o.provides: src/CMakeFiles/LaundryBot.dir/LaundryBot.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/LaundryBot.dir/build.make src/CMakeFiles/LaundryBot.dir/LaundryBot.cpp.o.provides.build
.PHONY : src/CMakeFiles/LaundryBot.dir/LaundryBot.cpp.o.provides

src/CMakeFiles/LaundryBot.dir/LaundryBot.cpp.o.provides.build: src/CMakeFiles/LaundryBot.dir/LaundryBot.cpp.o
.PHONY : src/CMakeFiles/LaundryBot.dir/LaundryBot.cpp.o.provides.build

src/CMakeFiles/LaundryBot.dir/SoftDemo.cpp.o: src/CMakeFiles/LaundryBot.dir/flags.make
src/CMakeFiles/LaundryBot.dir/SoftDemo.cpp.o: src/SoftDemo.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sanguinus/hojonathanho-bulletsim-56b8d9e/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/LaundryBot.dir/SoftDemo.cpp.o"
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/LaundryBot.dir/SoftDemo.cpp.o -c /home/sanguinus/hojonathanho-bulletsim-56b8d9e/src/SoftDemo.cpp

src/CMakeFiles/LaundryBot.dir/SoftDemo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LaundryBot.dir/SoftDemo.cpp.i"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CXX_CREATE_PREPROCESSED_SOURCE

src/CMakeFiles/LaundryBot.dir/SoftDemo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LaundryBot.dir/SoftDemo.cpp.s"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CXX_CREATE_ASSEMBLY_SOURCE

src/CMakeFiles/LaundryBot.dir/SoftDemo.cpp.o.requires:
.PHONY : src/CMakeFiles/LaundryBot.dir/SoftDemo.cpp.o.requires

src/CMakeFiles/LaundryBot.dir/SoftDemo.cpp.o.provides: src/CMakeFiles/LaundryBot.dir/SoftDemo.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/LaundryBot.dir/build.make src/CMakeFiles/LaundryBot.dir/SoftDemo.cpp.o.provides.build
.PHONY : src/CMakeFiles/LaundryBot.dir/SoftDemo.cpp.o.provides

src/CMakeFiles/LaundryBot.dir/SoftDemo.cpp.o.provides.build: src/CMakeFiles/LaundryBot.dir/SoftDemo.cpp.o
.PHONY : src/CMakeFiles/LaundryBot.dir/SoftDemo.cpp.o.provides.build

# Object files for target LaundryBot
LaundryBot_OBJECTS = \
"CMakeFiles/LaundryBot.dir/LaundryBot.cpp.o" \
"CMakeFiles/LaundryBot.dir/SoftDemo.cpp.o"

# External object files for target LaundryBot
LaundryBot_EXTERNAL_OBJECTS =

src/LaundryBot: src/CMakeFiles/LaundryBot.dir/LaundryBot.cpp.o
src/LaundryBot: src/CMakeFiles/LaundryBot.dir/SoftDemo.cpp.o
src/LaundryBot: lib/bullet-2.79/Demos/OpenGL/libOpenGLSupport.a
src/LaundryBot: lib/bullet-2.79/src/BulletSoftBody/libBulletSoftBody.a
src/LaundryBot: lib/bullet-2.79/src/BulletDynamics/libBulletDynamics.a
src/LaundryBot: lib/bullet-2.79/src/BulletCollision/libBulletCollision.a
src/LaundryBot: lib/bullet-2.79/src/LinearMath/libLinearMath.a
src/LaundryBot: /usr/lib/libglut.so
src/LaundryBot: /usr/lib/libGL.so
src/LaundryBot: /usr/lib/libGLU.so
src/LaundryBot: src/libsimulation.a
src/LaundryBot: lib/bullet-2.79/Demos/OpenGL/libOpenGLSupport.a
src/LaundryBot: lib/bullet-2.79/src/BulletSoftBody/libBulletSoftBody.a
src/LaundryBot: lib/bullet-2.79/src/BulletDynamics/libBulletDynamics.a
src/LaundryBot: lib/bullet-2.79/src/BulletCollision/libBulletCollision.a
src/LaundryBot: lib/bullet-2.79/src/LinearMath/libLinearMath.a
src/LaundryBot: /usr/lib/libglut.so
src/LaundryBot: /usr/lib/libGL.so
src/LaundryBot: /usr/lib/libGLU.so
src/LaundryBot: lib/haptics/libhaptics.a
src/LaundryBot: lib/osgBullet-2.0/libosgBullet.a
src/LaundryBot: lib/osgWorks-2.0/libosgWorks.a
src/LaundryBot: /usr/local/lib/libOpenThreads.so
src/LaundryBot: /usr/local/lib/libosg.so
src/LaundryBot: /usr/local/lib/libosgDB.so
src/LaundryBot: /usr/local/lib/libosgUtil.so
src/LaundryBot: /usr/local/lib/libosgViewer.so
src/LaundryBot: src/CMakeFiles/LaundryBot.dir/build.make
src/LaundryBot: src/CMakeFiles/LaundryBot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable LaundryBot"
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/LaundryBot.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/LaundryBot.dir/build: src/LaundryBot
.PHONY : src/CMakeFiles/LaundryBot.dir/build

src/CMakeFiles/LaundryBot.dir/requires: src/CMakeFiles/LaundryBot.dir/LaundryBot.cpp.o.requires
src/CMakeFiles/LaundryBot.dir/requires: src/CMakeFiles/LaundryBot.dir/SoftDemo.cpp.o.requires
.PHONY : src/CMakeFiles/LaundryBot.dir/requires

src/CMakeFiles/LaundryBot.dir/clean:
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e/src && $(CMAKE_COMMAND) -P CMakeFiles/LaundryBot.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/LaundryBot.dir/clean

src/CMakeFiles/LaundryBot.dir/depend:
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sanguinus/hojonathanho-bulletsim-56b8d9e /home/sanguinus/hojonathanho-bulletsim-56b8d9e/src /home/sanguinus/hojonathanho-bulletsim-56b8d9e /home/sanguinus/hojonathanho-bulletsim-56b8d9e/src /home/sanguinus/hojonathanho-bulletsim-56b8d9e/src/CMakeFiles/LaundryBot.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/LaundryBot.dir/depend

