# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/hzx/motor/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hzx/motor/build

# Include any dependencies generated for this target.
include pubmotor/CMakeFiles/sub.dir/depend.make

# Include the progress variables for this target.
include pubmotor/CMakeFiles/sub.dir/progress.make

# Include the compile flags for this target's objects.
include pubmotor/CMakeFiles/sub.dir/flags.make

pubmotor/CMakeFiles/sub.dir/src/sub.cpp.o: pubmotor/CMakeFiles/sub.dir/flags.make
pubmotor/CMakeFiles/sub.dir/src/sub.cpp.o: /home/hzx/motor/src/pubmotor/src/sub.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hzx/motor/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object pubmotor/CMakeFiles/sub.dir/src/sub.cpp.o"
	cd /home/hzx/motor/build/pubmotor && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sub.dir/src/sub.cpp.o -c /home/hzx/motor/src/pubmotor/src/sub.cpp

pubmotor/CMakeFiles/sub.dir/src/sub.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sub.dir/src/sub.cpp.i"
	cd /home/hzx/motor/build/pubmotor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hzx/motor/src/pubmotor/src/sub.cpp > CMakeFiles/sub.dir/src/sub.cpp.i

pubmotor/CMakeFiles/sub.dir/src/sub.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sub.dir/src/sub.cpp.s"
	cd /home/hzx/motor/build/pubmotor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hzx/motor/src/pubmotor/src/sub.cpp -o CMakeFiles/sub.dir/src/sub.cpp.s

# Object files for target sub
sub_OBJECTS = \
"CMakeFiles/sub.dir/src/sub.cpp.o"

# External object files for target sub
sub_EXTERNAL_OBJECTS =

/home/hzx/motor/devel/lib/pubmotor/sub: pubmotor/CMakeFiles/sub.dir/src/sub.cpp.o
/home/hzx/motor/devel/lib/pubmotor/sub: pubmotor/CMakeFiles/sub.dir/build.make
/home/hzx/motor/devel/lib/pubmotor/sub: /opt/ros/melodic/lib/libroscpp.so
/home/hzx/motor/devel/lib/pubmotor/sub: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/hzx/motor/devel/lib/pubmotor/sub: /opt/ros/melodic/lib/librosconsole.so
/home/hzx/motor/devel/lib/pubmotor/sub: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/hzx/motor/devel/lib/pubmotor/sub: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/hzx/motor/devel/lib/pubmotor/sub: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/hzx/motor/devel/lib/pubmotor/sub: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/hzx/motor/devel/lib/pubmotor/sub: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/hzx/motor/devel/lib/pubmotor/sub: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/hzx/motor/devel/lib/pubmotor/sub: /opt/ros/melodic/lib/librostime.so
/home/hzx/motor/devel/lib/pubmotor/sub: /opt/ros/melodic/lib/libcpp_common.so
/home/hzx/motor/devel/lib/pubmotor/sub: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/hzx/motor/devel/lib/pubmotor/sub: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/hzx/motor/devel/lib/pubmotor/sub: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/hzx/motor/devel/lib/pubmotor/sub: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/hzx/motor/devel/lib/pubmotor/sub: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/hzx/motor/devel/lib/pubmotor/sub: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/hzx/motor/devel/lib/pubmotor/sub: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/hzx/motor/devel/lib/pubmotor/sub: /opt/ros/melodic/lib/libserial.so
/home/hzx/motor/devel/lib/pubmotor/sub: pubmotor/CMakeFiles/sub.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hzx/motor/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/hzx/motor/devel/lib/pubmotor/sub"
	cd /home/hzx/motor/build/pubmotor && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sub.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
pubmotor/CMakeFiles/sub.dir/build: /home/hzx/motor/devel/lib/pubmotor/sub

.PHONY : pubmotor/CMakeFiles/sub.dir/build

pubmotor/CMakeFiles/sub.dir/clean:
	cd /home/hzx/motor/build/pubmotor && $(CMAKE_COMMAND) -P CMakeFiles/sub.dir/cmake_clean.cmake
.PHONY : pubmotor/CMakeFiles/sub.dir/clean

pubmotor/CMakeFiles/sub.dir/depend:
	cd /home/hzx/motor/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hzx/motor/src /home/hzx/motor/src/pubmotor /home/hzx/motor/build /home/hzx/motor/build/pubmotor /home/hzx/motor/build/pubmotor/CMakeFiles/sub.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : pubmotor/CMakeFiles/sub.dir/depend

