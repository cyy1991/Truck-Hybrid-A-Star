# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/ubuntu/hybrid_a_star

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/hybrid_a_star/build

# Include any dependencies generated for this target.
include CMakeFiles/grid_a_star_shared.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/grid_a_star_shared.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/grid_a_star_shared.dir/flags.make

CMakeFiles/grid_a_star_shared.dir/src/grid_a_star.cpp.o: CMakeFiles/grid_a_star_shared.dir/flags.make
CMakeFiles/grid_a_star_shared.dir/src/grid_a_star.cpp.o: ../src/grid_a_star.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/hybrid_a_star/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/grid_a_star_shared.dir/src/grid_a_star.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/grid_a_star_shared.dir/src/grid_a_star.cpp.o -c /home/ubuntu/hybrid_a_star/src/grid_a_star.cpp

CMakeFiles/grid_a_star_shared.dir/src/grid_a_star.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/grid_a_star_shared.dir/src/grid_a_star.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/hybrid_a_star/src/grid_a_star.cpp > CMakeFiles/grid_a_star_shared.dir/src/grid_a_star.cpp.i

CMakeFiles/grid_a_star_shared.dir/src/grid_a_star.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/grid_a_star_shared.dir/src/grid_a_star.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/hybrid_a_star/src/grid_a_star.cpp -o CMakeFiles/grid_a_star_shared.dir/src/grid_a_star.cpp.s

CMakeFiles/grid_a_star_shared.dir/src/grid_a_star.cpp.o.requires:

.PHONY : CMakeFiles/grid_a_star_shared.dir/src/grid_a_star.cpp.o.requires

CMakeFiles/grid_a_star_shared.dir/src/grid_a_star.cpp.o.provides: CMakeFiles/grid_a_star_shared.dir/src/grid_a_star.cpp.o.requires
	$(MAKE) -f CMakeFiles/grid_a_star_shared.dir/build.make CMakeFiles/grid_a_star_shared.dir/src/grid_a_star.cpp.o.provides.build
.PHONY : CMakeFiles/grid_a_star_shared.dir/src/grid_a_star.cpp.o.provides

CMakeFiles/grid_a_star_shared.dir/src/grid_a_star.cpp.o.provides.build: CMakeFiles/grid_a_star_shared.dir/src/grid_a_star.cpp.o


# Object files for target grid_a_star_shared
grid_a_star_shared_OBJECTS = \
"CMakeFiles/grid_a_star_shared.dir/src/grid_a_star.cpp.o"

# External object files for target grid_a_star_shared
grid_a_star_shared_EXTERNAL_OBJECTS =

../lib/libgrid_a_star_shared.so: CMakeFiles/grid_a_star_shared.dir/src/grid_a_star.cpp.o
../lib/libgrid_a_star_shared.so: CMakeFiles/grid_a_star_shared.dir/build.make
../lib/libgrid_a_star_shared.so: CMakeFiles/grid_a_star_shared.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/hybrid_a_star/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library ../lib/libgrid_a_star_shared.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/grid_a_star_shared.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/grid_a_star_shared.dir/build: ../lib/libgrid_a_star_shared.so

.PHONY : CMakeFiles/grid_a_star_shared.dir/build

CMakeFiles/grid_a_star_shared.dir/requires: CMakeFiles/grid_a_star_shared.dir/src/grid_a_star.cpp.o.requires

.PHONY : CMakeFiles/grid_a_star_shared.dir/requires

CMakeFiles/grid_a_star_shared.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/grid_a_star_shared.dir/cmake_clean.cmake
.PHONY : CMakeFiles/grid_a_star_shared.dir/clean

CMakeFiles/grid_a_star_shared.dir/depend:
	cd /home/ubuntu/hybrid_a_star/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/hybrid_a_star /home/ubuntu/hybrid_a_star /home/ubuntu/hybrid_a_star/build /home/ubuntu/hybrid_a_star/build /home/ubuntu/hybrid_a_star/build/CMakeFiles/grid_a_star_shared.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/grid_a_star_shared.dir/depend

