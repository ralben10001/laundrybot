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
include lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/depend.make

# Include the progress variables for this target.
include lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/progress.make

# Include the compile flags for this target's objects.
include lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/flags.make

lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/__/btSoftBodySolver_OpenCL.o: lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/flags.make
lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/__/btSoftBodySolver_OpenCL.o: lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/btSoftBodySolver_OpenCL.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/sanguinus/hojonathanho-bulletsim-56b8d9e/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/__/btSoftBodySolver_OpenCL.o"
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/__/btSoftBodySolver_OpenCL.o -c /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/btSoftBodySolver_OpenCL.cpp

lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/__/btSoftBodySolver_OpenCL.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/__/btSoftBodySolver_OpenCL.i"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CXX_CREATE_PREPROCESSED_SOURCE

lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/__/btSoftBodySolver_OpenCL.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/__/btSoftBodySolver_OpenCL.s"
	$(CMAKE_COMMAND) -E cmake_unimplemented_variable CMAKE_CXX_CREATE_ASSEMBLY_SOURCE

lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/__/btSoftBodySolver_OpenCL.o.requires:
.PHONY : lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/__/btSoftBodySolver_OpenCL.o.requires

lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/__/btSoftBodySolver_OpenCL.o.provides: lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/__/btSoftBodySolver_OpenCL.o.requires
	$(MAKE) -f lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/build.make lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/__/btSoftBodySolver_OpenCL.o.provides.build
.PHONY : lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/__/btSoftBodySolver_OpenCL.o.provides

lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/__/btSoftBodySolver_OpenCL.o.provides.build: lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/__/btSoftBodySolver_OpenCL.o
.PHONY : lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/__/btSoftBodySolver_OpenCL.o.provides.build

# Object files for target BulletSoftBodySolvers_OpenCL_Mini
BulletSoftBodySolvers_OpenCL_Mini_OBJECTS = \
"CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/__/btSoftBodySolver_OpenCL.o"

# External object files for target BulletSoftBodySolvers_OpenCL_Mini
BulletSoftBodySolvers_OpenCL_Mini_EXTERNAL_OBJECTS =

lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/libBulletSoftBodySolvers_OpenCL_Mini.a: lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/__/btSoftBodySolver_OpenCL.o
lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/libBulletSoftBodySolvers_OpenCL_Mini.a: lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/build.make
lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/libBulletSoftBodySolvers_OpenCL_Mini.a: lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library libBulletSoftBodySolvers_OpenCL_Mini.a"
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL && $(CMAKE_COMMAND) -P CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/cmake_clean_target.cmake
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/build: lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/libBulletSoftBodySolvers_OpenCL_Mini.a
.PHONY : lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/build

lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/requires: lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/__/btSoftBodySolver_OpenCL.o.requires
.PHONY : lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/requires

lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/clean:
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL && $(CMAKE_COMMAND) -P CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/cmake_clean.cmake
.PHONY : lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/clean

lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/depend:
	cd /home/sanguinus/hojonathanho-bulletsim-56b8d9e && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sanguinus/hojonathanho-bulletsim-56b8d9e /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL /home/sanguinus/hojonathanho-bulletsim-56b8d9e /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL /home/sanguinus/hojonathanho-bulletsim-56b8d9e/lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lib/bullet-2.79/src/BulletMultiThreaded/GpuSoftBodySolvers/OpenCL/MiniCL/CMakeFiles/BulletSoftBodySolvers_OpenCL_Mini.dir/depend

