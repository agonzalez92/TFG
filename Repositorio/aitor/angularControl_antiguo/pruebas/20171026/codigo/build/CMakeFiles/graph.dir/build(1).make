# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.0

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
CMAKE_SOURCE_DIR = /home/teo/repos/AitorRepo/Repositorio/angularControl/pruebas/20171026/codigo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/teo/repos/AitorRepo/Repositorio/angularControl/pruebas/20171026/codigo/build

# Include any dependencies generated for this target.
include CMakeFiles/graph.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/graph.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/graph.dir/flags.make

CMakeFiles/graph.dir/main.cpp.o: CMakeFiles/graph.dir/flags.make
CMakeFiles/graph.dir/main.cpp.o: ../main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/teo/repos/AitorRepo/Repositorio/angularControl/pruebas/20171026/codigo/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/graph.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/graph.dir/main.cpp.o -c /home/teo/repos/AitorRepo/Repositorio/angularControl/pruebas/20171026/codigo/main.cpp

CMakeFiles/graph.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/graph.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/teo/repos/AitorRepo/Repositorio/angularControl/pruebas/20171026/codigo/main.cpp > CMakeFiles/graph.dir/main.cpp.i

CMakeFiles/graph.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/graph.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/teo/repos/AitorRepo/Repositorio/angularControl/pruebas/20171026/codigo/main.cpp -o CMakeFiles/graph.dir/main.cpp.s

CMakeFiles/graph.dir/main.cpp.o.requires:
.PHONY : CMakeFiles/graph.dir/main.cpp.o.requires

CMakeFiles/graph.dir/main.cpp.o.provides: CMakeFiles/graph.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/graph.dir/build.make CMakeFiles/graph.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/graph.dir/main.cpp.o.provides

CMakeFiles/graph.dir/main.cpp.o.provides.build: CMakeFiles/graph.dir/main.cpp.o

CMakeFiles/graph.dir/LIPM2d.cpp.o: CMakeFiles/graph.dir/flags.make
CMakeFiles/graph.dir/LIPM2d.cpp.o: ../LIPM2d.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/teo/repos/AitorRepo/Repositorio/angularControl/pruebas/20171026/codigo/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/graph.dir/LIPM2d.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/graph.dir/LIPM2d.cpp.o -c /home/teo/repos/AitorRepo/Repositorio/angularControl/pruebas/20171026/codigo/LIPM2d.cpp

CMakeFiles/graph.dir/LIPM2d.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/graph.dir/LIPM2d.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/teo/repos/AitorRepo/Repositorio/angularControl/pruebas/20171026/codigo/LIPM2d.cpp > CMakeFiles/graph.dir/LIPM2d.cpp.i

CMakeFiles/graph.dir/LIPM2d.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/graph.dir/LIPM2d.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/teo/repos/AitorRepo/Repositorio/angularControl/pruebas/20171026/codigo/LIPM2d.cpp -o CMakeFiles/graph.dir/LIPM2d.cpp.s

CMakeFiles/graph.dir/LIPM2d.cpp.o.requires:
.PHONY : CMakeFiles/graph.dir/LIPM2d.cpp.o.requires

CMakeFiles/graph.dir/LIPM2d.cpp.o.provides: CMakeFiles/graph.dir/LIPM2d.cpp.o.requires
	$(MAKE) -f CMakeFiles/graph.dir/build.make CMakeFiles/graph.dir/LIPM2d.cpp.o.provides.build
.PHONY : CMakeFiles/graph.dir/LIPM2d.cpp.o.provides

CMakeFiles/graph.dir/LIPM2d.cpp.o.provides.build: CMakeFiles/graph.dir/LIPM2d.cpp.o

# Object files for target graph
graph_OBJECTS = \
"CMakeFiles/graph.dir/main.cpp.o" \
"CMakeFiles/graph.dir/LIPM2d.cpp.o"

# External object files for target graph
graph_EXTERNAL_OBJECTS =

graph: CMakeFiles/graph.dir/main.cpp.o
graph: CMakeFiles/graph.dir/LIPM2d.cpp.o
graph: CMakeFiles/graph.dir/build.make
graph: /usr/local/lib/libYARP_OS.so.2.3.68
graph: /usr/local/lib/libYARP_sig.so.2.3.68
graph: /usr/local/lib/libYARP_math.so.2.3.68
graph: /usr/local/lib/libYARP_dev.so.2.3.68
graph: /usr/local/lib/libYARP_name.so.2.3.68
graph: /usr/local/lib/libYARP_init.so.2.3.68
graph: /usr/local/lib/libYARP_math.so.2.3.68
graph: /usr/local/lib/libYARP_sig.so.2.3.68
graph: /usr/local/lib/libYARP_OS.so.2.3.68
graph: CMakeFiles/graph.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable graph"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/graph.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/graph.dir/build: graph
.PHONY : CMakeFiles/graph.dir/build

CMakeFiles/graph.dir/requires: CMakeFiles/graph.dir/main.cpp.o.requires
CMakeFiles/graph.dir/requires: CMakeFiles/graph.dir/LIPM2d.cpp.o.requires
.PHONY : CMakeFiles/graph.dir/requires

CMakeFiles/graph.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/graph.dir/cmake_clean.cmake
.PHONY : CMakeFiles/graph.dir/clean

CMakeFiles/graph.dir/depend:
	cd /home/teo/repos/AitorRepo/Repositorio/angularControl/pruebas/20171026/codigo/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/teo/repos/AitorRepo/Repositorio/angularControl/pruebas/20171026/codigo /home/teo/repos/AitorRepo/Repositorio/angularControl/pruebas/20171026/codigo /home/teo/repos/AitorRepo/Repositorio/angularControl/pruebas/20171026/codigo/build /home/teo/repos/AitorRepo/Repositorio/angularControl/pruebas/20171026/codigo/build /home/teo/repos/AitorRepo/Repositorio/angularControl/pruebas/20171026/codigo/build/CMakeFiles/graph.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/graph.dir/depend

