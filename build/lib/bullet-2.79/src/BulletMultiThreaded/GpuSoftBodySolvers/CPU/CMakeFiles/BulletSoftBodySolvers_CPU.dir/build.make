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
CMAKE_BINARY_DIR = /home/sanguinus/hojonathanho-bulletsim-56b8d9e/build

# Include any dependencies generated for this target.
include lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/depend.make

# Include the progress variables for this target.
include lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/progress.make

# Include the compile flags for this target's objects.
include lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/flags.make

lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/btSoftBodySolver_CPU.o: lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/flags.make
lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/btSoftBodySolver_CPU.o: ../lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/btSoftBodySolver_CPU.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sanguinus/hojonathanho-bulletsim-56b8d9e/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/btSoftBodySolver_CPU.o"
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e/build/lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/BulletSoftBodySolvers_CPU.dir/btSoftBodySolver_CPU.o -c /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/btSoftBodySolver_CPU.cpp

lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/btSoftBodySolver_CPU.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BulletSoftBodySolvers_CPU.dir/btSoftBodySolver_CPU.i"
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e/build/lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/btSoftBodySolver_CPU.cpp > CMakeFiles/BulletSoftBodySolvers_CPU.dir/btSoftBodySolver_CPU.i

lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/btSoftBodySolver_CPU.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BulletSoftBodySolvers_CPU.dir/btSoftBodySolver_CPU.s"
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e/build/lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/btSoftBodySolver_CPU.cpp -o CMakeFiles/BulletSoftBodySolvers_CPU.dir/btSoftBodySolver_CPU.s

lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/btSoftBodySolver_CPU.o.requires:
.PHONY : lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/btSoftBodySolver_CPU.o.requires

lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/btSoftBodySolver_CPU.o.provides: lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/btSoftBodySolver_CPU.o.requires
	$(MAKE) -f lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/build.make lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/btSoftBodySolver_CPU.o.provides.build
.PHONY : lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/btSoftBodySolver_CPU.o.provides

lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/btSoftBodySolver_CPU.o.provides.build: lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/btSoftBodySolver_CPU.o
.PHONY : lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/btSoftBodySolver_CPU.o.provides.build

# Object files for target BulletSoftBodySolvers_CPU
BulletSoftBodySolvers_CPU_OBJECTS = \
"CMakeFiles/BulletSoftBodySolvers_CPU.dir/btSoftBodySolver_CPU.o"

# External object files for target BulletSoftBodySolvers_CPU
BulletSoftBodySolvers_CPU_EXTERNAL_OBJECTS =

lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/libBulletSoftBodySolvers_CPU.a: lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/btSoftBodySolver_CPU.o
lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/libBulletSoftBodySolvers_CPU.a: lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/build.make
lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/libBulletSoftBodySolvers_CPU.a: lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library libBulletSoftBodySolvers_CPU.a"
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e/build/lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU && $(CMAKE_COMMAND) -P CMakeFiles/BulletSoftBodySolvers_CPU.dir/cmake_clean_target.cmake
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e/build/lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/BulletSoftBodySolvers_CPU.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/build: lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/libBulletSoftBodySolvers_CPU.a
.PHONY : lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/build

lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/requires: lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/btSoftBodySolver_CPU.o.requires
.PHONY : lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/requires

lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/clean:
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e/build/lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU && $(CMAKE_COMMAND) -P CMakeFiles/BulletSoftBodySolvers_CPU.dir/cmake_clean.cmake
.PHONY : lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/clean

lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/depend:
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sanguinus/hojonathanho-bulletsim-56b8d9e /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU /home/sanguinus/hojonathanho-bulletsim-56b8d9e/build /home/sanguinus/hojonathanho-bulletsim-56b8d9e/build/lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU /home/sanguinus/hojonathanho-bulletsim-56b8d9e/build/lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/CPU/CMakeFiles/BulletSoftBodySolvers_CPU.dir/depend

