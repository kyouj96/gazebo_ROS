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
CMAKE_SOURCE_DIR = /home/parallels/Desktop/gazebo_ws

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/parallels/Desktop/gazebo_ws/build

# Include any dependencies generated for this target.
include CMakeFiles/velodyne_plugin.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/velodyne_plugin.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/velodyne_plugin.dir/flags.make

CMakeFiles/velodyne_plugin.dir/velodyne_plugin.cc.o: CMakeFiles/velodyne_plugin.dir/flags.make
CMakeFiles/velodyne_plugin.dir/velodyne_plugin.cc.o: ../velodyne_plugin.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/parallels/Desktop/gazebo_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/velodyne_plugin.dir/velodyne_plugin.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/velodyne_plugin.dir/velodyne_plugin.cc.o -c /home/parallels/Desktop/gazebo_ws/velodyne_plugin.cc

CMakeFiles/velodyne_plugin.dir/velodyne_plugin.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/velodyne_plugin.dir/velodyne_plugin.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/parallels/Desktop/gazebo_ws/velodyne_plugin.cc > CMakeFiles/velodyne_plugin.dir/velodyne_plugin.cc.i

CMakeFiles/velodyne_plugin.dir/velodyne_plugin.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/velodyne_plugin.dir/velodyne_plugin.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/parallels/Desktop/gazebo_ws/velodyne_plugin.cc -o CMakeFiles/velodyne_plugin.dir/velodyne_plugin.cc.s

CMakeFiles/velodyne_plugin.dir/velodyne_plugin.cc.o.requires:

.PHONY : CMakeFiles/velodyne_plugin.dir/velodyne_plugin.cc.o.requires

CMakeFiles/velodyne_plugin.dir/velodyne_plugin.cc.o.provides: CMakeFiles/velodyne_plugin.dir/velodyne_plugin.cc.o.requires
	$(MAKE) -f CMakeFiles/velodyne_plugin.dir/build.make CMakeFiles/velodyne_plugin.dir/velodyne_plugin.cc.o.provides.build
.PHONY : CMakeFiles/velodyne_plugin.dir/velodyne_plugin.cc.o.provides

CMakeFiles/velodyne_plugin.dir/velodyne_plugin.cc.o.provides.build: CMakeFiles/velodyne_plugin.dir/velodyne_plugin.cc.o


# Object files for target velodyne_plugin
velodyne_plugin_OBJECTS = \
"CMakeFiles/velodyne_plugin.dir/velodyne_plugin.cc.o"

# External object files for target velodyne_plugin
velodyne_plugin_EXTERNAL_OBJECTS =

libvelodyne_plugin.so: CMakeFiles/velodyne_plugin.dir/velodyne_plugin.cc.o
libvelodyne_plugin.so: CMakeFiles/velodyne_plugin.dir/build.make
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_math.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_ccd.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libsdformat.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-math2.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-math2.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libsdformat.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
libvelodyne_plugin.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
libvelodyne_plugin.so: CMakeFiles/velodyne_plugin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/parallels/Desktop/gazebo_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libvelodyne_plugin.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/velodyne_plugin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/velodyne_plugin.dir/build: libvelodyne_plugin.so

.PHONY : CMakeFiles/velodyne_plugin.dir/build

CMakeFiles/velodyne_plugin.dir/requires: CMakeFiles/velodyne_plugin.dir/velodyne_plugin.cc.o.requires

.PHONY : CMakeFiles/velodyne_plugin.dir/requires

CMakeFiles/velodyne_plugin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/velodyne_plugin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/velodyne_plugin.dir/clean

CMakeFiles/velodyne_plugin.dir/depend:
	cd /home/parallels/Desktop/gazebo_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/parallels/Desktop/gazebo_ws /home/parallels/Desktop/gazebo_ws /home/parallels/Desktop/gazebo_ws/build /home/parallels/Desktop/gazebo_ws/build /home/parallels/Desktop/gazebo_ws/build/CMakeFiles/velodyne_plugin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/velodyne_plugin.dir/depend

