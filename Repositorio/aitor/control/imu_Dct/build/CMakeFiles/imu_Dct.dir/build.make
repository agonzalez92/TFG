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
CMAKE_SOURCE_DIR = /home/teo/repos/aitor/control/imu_Dct

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/teo/repos/aitor/control/imu_Dct/build

# Include any dependencies generated for this target.
include CMakeFiles/imu_Dct.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/imu_Dct.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/imu_Dct.dir/flags.make

CMakeFiles/imu_Dct.dir/LIPM2d.cpp.o: CMakeFiles/imu_Dct.dir/flags.make
CMakeFiles/imu_Dct.dir/LIPM2d.cpp.o: ../LIPM2d.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/teo/repos/aitor/control/imu_Dct/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/imu_Dct.dir/LIPM2d.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/imu_Dct.dir/LIPM2d.cpp.o -c /home/teo/repos/aitor/control/imu_Dct/LIPM2d.cpp

CMakeFiles/imu_Dct.dir/LIPM2d.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imu_Dct.dir/LIPM2d.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/teo/repos/aitor/control/imu_Dct/LIPM2d.cpp > CMakeFiles/imu_Dct.dir/LIPM2d.cpp.i

CMakeFiles/imu_Dct.dir/LIPM2d.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imu_Dct.dir/LIPM2d.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/teo/repos/aitor/control/imu_Dct/LIPM2d.cpp -o CMakeFiles/imu_Dct.dir/LIPM2d.cpp.s

CMakeFiles/imu_Dct.dir/LIPM2d.cpp.o.requires:

.PHONY : CMakeFiles/imu_Dct.dir/LIPM2d.cpp.o.requires

CMakeFiles/imu_Dct.dir/LIPM2d.cpp.o.provides: CMakeFiles/imu_Dct.dir/LIPM2d.cpp.o.requires
	$(MAKE) -f CMakeFiles/imu_Dct.dir/build.make CMakeFiles/imu_Dct.dir/LIPM2d.cpp.o.provides.build
.PHONY : CMakeFiles/imu_Dct.dir/LIPM2d.cpp.o.provides

CMakeFiles/imu_Dct.dir/LIPM2d.cpp.o.provides.build: CMakeFiles/imu_Dct.dir/LIPM2d.cpp.o


CMakeFiles/imu_Dct.dir/pid.cpp.o: CMakeFiles/imu_Dct.dir/flags.make
CMakeFiles/imu_Dct.dir/pid.cpp.o: ../pid.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/teo/repos/aitor/control/imu_Dct/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/imu_Dct.dir/pid.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/imu_Dct.dir/pid.cpp.o -c /home/teo/repos/aitor/control/imu_Dct/pid.cpp

CMakeFiles/imu_Dct.dir/pid.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imu_Dct.dir/pid.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/teo/repos/aitor/control/imu_Dct/pid.cpp > CMakeFiles/imu_Dct.dir/pid.cpp.i

CMakeFiles/imu_Dct.dir/pid.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imu_Dct.dir/pid.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/teo/repos/aitor/control/imu_Dct/pid.cpp -o CMakeFiles/imu_Dct.dir/pid.cpp.s

CMakeFiles/imu_Dct.dir/pid.cpp.o.requires:

.PHONY : CMakeFiles/imu_Dct.dir/pid.cpp.o.requires

CMakeFiles/imu_Dct.dir/pid.cpp.o.provides: CMakeFiles/imu_Dct.dir/pid.cpp.o.requires
	$(MAKE) -f CMakeFiles/imu_Dct.dir/build.make CMakeFiles/imu_Dct.dir/pid.cpp.o.provides.build
.PHONY : CMakeFiles/imu_Dct.dir/pid.cpp.o.provides

CMakeFiles/imu_Dct.dir/pid.cpp.o.provides.build: CMakeFiles/imu_Dct.dir/pid.cpp.o


CMakeFiles/imu_Dct.dir/main.cpp.o: CMakeFiles/imu_Dct.dir/flags.make
CMakeFiles/imu_Dct.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/teo/repos/aitor/control/imu_Dct/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/imu_Dct.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/imu_Dct.dir/main.cpp.o -c /home/teo/repos/aitor/control/imu_Dct/main.cpp

CMakeFiles/imu_Dct.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imu_Dct.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/teo/repos/aitor/control/imu_Dct/main.cpp > CMakeFiles/imu_Dct.dir/main.cpp.i

CMakeFiles/imu_Dct.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imu_Dct.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/teo/repos/aitor/control/imu_Dct/main.cpp -o CMakeFiles/imu_Dct.dir/main.cpp.s

CMakeFiles/imu_Dct.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/imu_Dct.dir/main.cpp.o.requires

CMakeFiles/imu_Dct.dir/main.cpp.o.provides: CMakeFiles/imu_Dct.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/imu_Dct.dir/build.make CMakeFiles/imu_Dct.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/imu_Dct.dir/main.cpp.o.provides

CMakeFiles/imu_Dct.dir/main.cpp.o.provides.build: CMakeFiles/imu_Dct.dir/main.cpp.o


# Object files for target imu_Dct
imu_Dct_OBJECTS = \
"CMakeFiles/imu_Dct.dir/LIPM2d.cpp.o" \
"CMakeFiles/imu_Dct.dir/pid.cpp.o" \
"CMakeFiles/imu_Dct.dir/main.cpp.o"

# External object files for target imu_Dct
imu_Dct_EXTERNAL_OBJECTS =

imu_Dct: CMakeFiles/imu_Dct.dir/LIPM2d.cpp.o
imu_Dct: CMakeFiles/imu_Dct.dir/pid.cpp.o
imu_Dct: CMakeFiles/imu_Dct.dir/main.cpp.o
imu_Dct: CMakeFiles/imu_Dct.dir/build.make
imu_Dct: /usr/local/lib/libYARP_dev.so.2.3.72
imu_Dct: /usr/local/lib/libYARP_name.so.2.3.72
imu_Dct: /usr/local/lib/libYARP_init.so.2.3.72
imu_Dct: /usr/local/lib/libYARP_math.so.2.3.72
imu_Dct: /usr/local/lib/libYARP_sig.so.2.3.72
imu_Dct: /usr/local/lib/libYARP_OS.so.2.3.72
imu_Dct: CMakeFiles/imu_Dct.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/teo/repos/aitor/control/imu_Dct/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable imu_Dct"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/imu_Dct.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/imu_Dct.dir/build: imu_Dct

.PHONY : CMakeFiles/imu_Dct.dir/build

CMakeFiles/imu_Dct.dir/requires: CMakeFiles/imu_Dct.dir/LIPM2d.cpp.o.requires
CMakeFiles/imu_Dct.dir/requires: CMakeFiles/imu_Dct.dir/pid.cpp.o.requires
CMakeFiles/imu_Dct.dir/requires: CMakeFiles/imu_Dct.dir/main.cpp.o.requires

.PHONY : CMakeFiles/imu_Dct.dir/requires

CMakeFiles/imu_Dct.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/imu_Dct.dir/cmake_clean.cmake
.PHONY : CMakeFiles/imu_Dct.dir/clean

CMakeFiles/imu_Dct.dir/depend:
	cd /home/teo/repos/aitor/control/imu_Dct/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/teo/repos/aitor/control/imu_Dct /home/teo/repos/aitor/control/imu_Dct /home/teo/repos/aitor/control/imu_Dct/build /home/teo/repos/aitor/control/imu_Dct/build /home/teo/repos/aitor/control/imu_Dct/build/CMakeFiles/imu_Dct.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/imu_Dct.dir/depend
