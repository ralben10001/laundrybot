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
include src/CMakeFiles/softbody_test.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/softbody_test.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/softbody_test.dir/flags.make

src/CMakeFiles/softbody_test.dir/softbody_test.cpp.o: src/CMakeFiles/softbody_test.dir/flags.make
src/CMakeFiles/softbody_test.dir/softbody_test.cpp.o: src/softbody_test.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sanguinus/hojonathanho-bulletsim-56b8d9e/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/softbody_test.dir/softbody_test.cpp.o"
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/softbody_test.dir/softbody_test.cpp.o -c /home/sanguinus/hojonathanho-bulletsim-56b8d9e/src/softbody_test.cpp

src/CMakeFiles/softbody_test.dir/softbody_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/softbody_test.dir/softbody_test.cpp.i"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CXX_CREATE_PREPROCESSED_SOURCE

src/CMakeFiles/softbody_test.dir/softbody_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/softbody_test.dir/softbody_test.cpp.s"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CXX_CREATE_ASSEMBLY_SOURCE

src/CMakeFiles/softbody_test.dir/softbody_test.cpp.o.requires:
.PHONY : src/CMakeFiles/softbody_test.dir/softbody_test.cpp.o.requires

src/CMakeFiles/softbody_test.dir/softbody_test.cpp.o.provides: src/CMakeFiles/softbody_test.dir/softbody_test.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/softbody_test.dir/build.make src/CMakeFiles/softbody_test.dir/softbody_test.cpp.o.provides.build
.PHONY : src/CMakeFiles/softbody_test.dir/softbody_test.cpp.o.provides

src/CMakeFiles/softbody_test.dir/softbody_test.cpp.o.provides.build: src/CMakeFiles/softbody_test.dir/softbody_test.cpp.o
.PHONY : src/CMakeFiles/softbody_test.dir/softbody_test.cpp.o.provides.build

# Object files for target softbody_test
softbody_test_OBJECTS = \
"CMakeFiles/softbody_test.dir/softbody_test.cpp.o"

# External object files for target softbody_test
softbody_test_EXTERNAL_OBJECTS =

src/softbody_test: src/CMakeFiles/softbody_test.dir/softbody_test.cpp.o
src/softbody_test: src/libsimulation.a
src/softbody_test: lib/haptics/libhaptics.a
src/softbody_test: lib/osgBullet-2.0/libosgBullet.a
src/softbody_test: lib/osgWorks-2.0/libosgWorks.a
src/softbody_test: /usr/lib/libglut.so
src/softbody_test: /usr/lib/libGL.so
src/softbody_test: /usr/lib/libGLU.so
src/softbody_test: /usr/local/lib/libOpenThreads.so
src/softbody_test: /usr/local/lib/libosg.so
src/softbody_test: /usr/local/lib/libosgDB.so
src/softbody_test: /usr/local/lib/libosgUtil.so
src/softbody_test: /usr/local/lib/libosgViewer.so
src/softbody_test: lib/bullet-2.79/src/BulletSoftBody/libBulletSoftBody.a
src/softbody_test: lib/bullet-2.79/src/BulletDynamics/libBulletDynamics.a
src/softbody_test: lib/bullet-2.79/src/BulletCollision/libBulletCollision.a
src/softbody_test: lib/bullet-2.79/src/LinearMath/libLinearMath.a
src/softbody_test: lib/bullet-2.79/Demos/OpenGL/libOpenGLSupport.a
src/softbody_test: src/CMakeFiles/softbody_test.dir/build.make
src/softbody_test: src/CMakeFiles/softbody_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable softbody_test"
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/softbody_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/softbody_test.dir/build: src/softbody_test
.PHONY : src/CMakeFiles/softbody_test.dir/build

src/CMakeFiles/softbody_test.dir/requires: src/CMakeFiles/softbody_test.dir/softbody_test.cpp.o.requires
.PHONY : src/CMakeFiles/softbody_test.dir/requires

src/CMakeFiles/softbody_test.dir/clean:
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e/src && $(CMAKE_COMMAND) -P CMakeFiles/softbody_test.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/softbody_test.dir/clean

src/CMakeFiles/softbody_test.dir/depend:
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sanguinus/hojonathanho-bulletsim-56b8d9e /home/sanguinus/hojonathanho-bulletsim-56b8d9e/src /home/sanguinus/hojonathanho-bulletsim-56b8d9e /home/sanguinus/hojonathanho-bulletsim-56b8d9e/src /home/sanguinus/hojonathanho-bulletsim-56b8d9e/src/CMakeFiles/softbody_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/softbody_test.dir/depend

