# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

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

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /snap/clion/249/bin/cmake/linux/x64/bin/cmake

# The command to remove a file.
RM = /snap/clion/249/bin/cmake/linux/x64/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/student/CLionProjects/graphics_engine

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/student/CLionProjects/graphics_engine/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/engine.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/engine.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/engine.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/engine.dir/flags.make

CMakeFiles/engine.dir/easy_image.cc.o: CMakeFiles/engine.dir/flags.make
CMakeFiles/engine.dir/easy_image.cc.o: /home/student/CLionProjects/graphics_engine/easy_image.cc
CMakeFiles/engine.dir/easy_image.cc.o: CMakeFiles/engine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/CLionProjects/graphics_engine/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/engine.dir/easy_image.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/engine.dir/easy_image.cc.o -MF CMakeFiles/engine.dir/easy_image.cc.o.d -o CMakeFiles/engine.dir/easy_image.cc.o -c /home/student/CLionProjects/graphics_engine/easy_image.cc

CMakeFiles/engine.dir/easy_image.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/engine.dir/easy_image.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/student/CLionProjects/graphics_engine/easy_image.cc > CMakeFiles/engine.dir/easy_image.cc.i

CMakeFiles/engine.dir/easy_image.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/engine.dir/easy_image.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/student/CLionProjects/graphics_engine/easy_image.cc -o CMakeFiles/engine.dir/easy_image.cc.s

CMakeFiles/engine.dir/ini_configuration.cc.o: CMakeFiles/engine.dir/flags.make
CMakeFiles/engine.dir/ini_configuration.cc.o: /home/student/CLionProjects/graphics_engine/ini_configuration.cc
CMakeFiles/engine.dir/ini_configuration.cc.o: CMakeFiles/engine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/CLionProjects/graphics_engine/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/engine.dir/ini_configuration.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/engine.dir/ini_configuration.cc.o -MF CMakeFiles/engine.dir/ini_configuration.cc.o.d -o CMakeFiles/engine.dir/ini_configuration.cc.o -c /home/student/CLionProjects/graphics_engine/ini_configuration.cc

CMakeFiles/engine.dir/ini_configuration.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/engine.dir/ini_configuration.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/student/CLionProjects/graphics_engine/ini_configuration.cc > CMakeFiles/engine.dir/ini_configuration.cc.i

CMakeFiles/engine.dir/ini_configuration.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/engine.dir/ini_configuration.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/student/CLionProjects/graphics_engine/ini_configuration.cc -o CMakeFiles/engine.dir/ini_configuration.cc.s

CMakeFiles/engine.dir/engine.cc.o: CMakeFiles/engine.dir/flags.make
CMakeFiles/engine.dir/engine.cc.o: /home/student/CLionProjects/graphics_engine/engine.cc
CMakeFiles/engine.dir/engine.cc.o: CMakeFiles/engine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/CLionProjects/graphics_engine/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/engine.dir/engine.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/engine.dir/engine.cc.o -MF CMakeFiles/engine.dir/engine.cc.o.d -o CMakeFiles/engine.dir/engine.cc.o -c /home/student/CLionProjects/graphics_engine/engine.cc

CMakeFiles/engine.dir/engine.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/engine.dir/engine.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/student/CLionProjects/graphics_engine/engine.cc > CMakeFiles/engine.dir/engine.cc.i

CMakeFiles/engine.dir/engine.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/engine.dir/engine.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/student/CLionProjects/graphics_engine/engine.cc -o CMakeFiles/engine.dir/engine.cc.s

CMakeFiles/engine.dir/2D/l_parser/l_parser.cc.o: CMakeFiles/engine.dir/flags.make
CMakeFiles/engine.dir/2D/l_parser/l_parser.cc.o: /home/student/CLionProjects/graphics_engine/2D/l_parser/l_parser.cc
CMakeFiles/engine.dir/2D/l_parser/l_parser.cc.o: CMakeFiles/engine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/CLionProjects/graphics_engine/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/engine.dir/2D/l_parser/l_parser.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/engine.dir/2D/l_parser/l_parser.cc.o -MF CMakeFiles/engine.dir/2D/l_parser/l_parser.cc.o.d -o CMakeFiles/engine.dir/2D/l_parser/l_parser.cc.o -c /home/student/CLionProjects/graphics_engine/2D/l_parser/l_parser.cc

CMakeFiles/engine.dir/2D/l_parser/l_parser.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/engine.dir/2D/l_parser/l_parser.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/student/CLionProjects/graphics_engine/2D/l_parser/l_parser.cc > CMakeFiles/engine.dir/2D/l_parser/l_parser.cc.i

CMakeFiles/engine.dir/2D/l_parser/l_parser.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/engine.dir/2D/l_parser/l_parser.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/student/CLionProjects/graphics_engine/2D/l_parser/l_parser.cc -o CMakeFiles/engine.dir/2D/l_parser/l_parser.cc.s

CMakeFiles/engine.dir/2D/Lines2D.cpp.o: CMakeFiles/engine.dir/flags.make
CMakeFiles/engine.dir/2D/Lines2D.cpp.o: /home/student/CLionProjects/graphics_engine/2D/Lines2D.cpp
CMakeFiles/engine.dir/2D/Lines2D.cpp.o: CMakeFiles/engine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/CLionProjects/graphics_engine/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/engine.dir/2D/Lines2D.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/engine.dir/2D/Lines2D.cpp.o -MF CMakeFiles/engine.dir/2D/Lines2D.cpp.o.d -o CMakeFiles/engine.dir/2D/Lines2D.cpp.o -c /home/student/CLionProjects/graphics_engine/2D/Lines2D.cpp

CMakeFiles/engine.dir/2D/Lines2D.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/engine.dir/2D/Lines2D.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/student/CLionProjects/graphics_engine/2D/Lines2D.cpp > CMakeFiles/engine.dir/2D/Lines2D.cpp.i

CMakeFiles/engine.dir/2D/Lines2D.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/engine.dir/2D/Lines2D.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/student/CLionProjects/graphics_engine/2D/Lines2D.cpp -o CMakeFiles/engine.dir/2D/Lines2D.cpp.s

CMakeFiles/engine.dir/2D/2D_LSystem.cpp.o: CMakeFiles/engine.dir/flags.make
CMakeFiles/engine.dir/2D/2D_LSystem.cpp.o: /home/student/CLionProjects/graphics_engine/2D/2D_LSystem.cpp
CMakeFiles/engine.dir/2D/2D_LSystem.cpp.o: CMakeFiles/engine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/CLionProjects/graphics_engine/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/engine.dir/2D/2D_LSystem.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/engine.dir/2D/2D_LSystem.cpp.o -MF CMakeFiles/engine.dir/2D/2D_LSystem.cpp.o.d -o CMakeFiles/engine.dir/2D/2D_LSystem.cpp.o -c /home/student/CLionProjects/graphics_engine/2D/2D_LSystem.cpp

CMakeFiles/engine.dir/2D/2D_LSystem.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/engine.dir/2D/2D_LSystem.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/student/CLionProjects/graphics_engine/2D/2D_LSystem.cpp > CMakeFiles/engine.dir/2D/2D_LSystem.cpp.i

CMakeFiles/engine.dir/2D/2D_LSystem.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/engine.dir/2D/2D_LSystem.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/student/CLionProjects/graphics_engine/2D/2D_LSystem.cpp -o CMakeFiles/engine.dir/2D/2D_LSystem.cpp.s

CMakeFiles/engine.dir/3D/vector3D.cc.o: CMakeFiles/engine.dir/flags.make
CMakeFiles/engine.dir/3D/vector3D.cc.o: /home/student/CLionProjects/graphics_engine/3D/vector3D.cc
CMakeFiles/engine.dir/3D/vector3D.cc.o: CMakeFiles/engine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/CLionProjects/graphics_engine/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/engine.dir/3D/vector3D.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/engine.dir/3D/vector3D.cc.o -MF CMakeFiles/engine.dir/3D/vector3D.cc.o.d -o CMakeFiles/engine.dir/3D/vector3D.cc.o -c /home/student/CLionProjects/graphics_engine/3D/vector3D.cc

CMakeFiles/engine.dir/3D/vector3D.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/engine.dir/3D/vector3D.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/student/CLionProjects/graphics_engine/3D/vector3D.cc > CMakeFiles/engine.dir/3D/vector3D.cc.i

CMakeFiles/engine.dir/3D/vector3D.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/engine.dir/3D/vector3D.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/student/CLionProjects/graphics_engine/3D/vector3D.cc -o CMakeFiles/engine.dir/3D/vector3D.cc.s

CMakeFiles/engine.dir/3D/Figure3D.cpp.o: CMakeFiles/engine.dir/flags.make
CMakeFiles/engine.dir/3D/Figure3D.cpp.o: /home/student/CLionProjects/graphics_engine/3D/Figure3D.cpp
CMakeFiles/engine.dir/3D/Figure3D.cpp.o: CMakeFiles/engine.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/CLionProjects/graphics_engine/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/engine.dir/3D/Figure3D.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/engine.dir/3D/Figure3D.cpp.o -MF CMakeFiles/engine.dir/3D/Figure3D.cpp.o.d -o CMakeFiles/engine.dir/3D/Figure3D.cpp.o -c /home/student/CLionProjects/graphics_engine/3D/Figure3D.cpp

CMakeFiles/engine.dir/3D/Figure3D.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/engine.dir/3D/Figure3D.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/student/CLionProjects/graphics_engine/3D/Figure3D.cpp > CMakeFiles/engine.dir/3D/Figure3D.cpp.i

CMakeFiles/engine.dir/3D/Figure3D.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/engine.dir/3D/Figure3D.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/student/CLionProjects/graphics_engine/3D/Figure3D.cpp -o CMakeFiles/engine.dir/3D/Figure3D.cpp.s

# Object files for target engine
engine_OBJECTS = \
"CMakeFiles/engine.dir/easy_image.cc.o" \
"CMakeFiles/engine.dir/ini_configuration.cc.o" \
"CMakeFiles/engine.dir/engine.cc.o" \
"CMakeFiles/engine.dir/2D/l_parser/l_parser.cc.o" \
"CMakeFiles/engine.dir/2D/Lines2D.cpp.o" \
"CMakeFiles/engine.dir/2D/2D_LSystem.cpp.o" \
"CMakeFiles/engine.dir/3D/vector3D.cc.o" \
"CMakeFiles/engine.dir/3D/Figure3D.cpp.o"

# External object files for target engine
engine_EXTERNAL_OBJECTS =

engine: CMakeFiles/engine.dir/easy_image.cc.o
engine: CMakeFiles/engine.dir/ini_configuration.cc.o
engine: CMakeFiles/engine.dir/engine.cc.o
engine: CMakeFiles/engine.dir/2D/l_parser/l_parser.cc.o
engine: CMakeFiles/engine.dir/2D/Lines2D.cpp.o
engine: CMakeFiles/engine.dir/2D/2D_LSystem.cpp.o
engine: CMakeFiles/engine.dir/3D/vector3D.cc.o
engine: CMakeFiles/engine.dir/3D/Figure3D.cpp.o
engine: CMakeFiles/engine.dir/build.make
engine: CMakeFiles/engine.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/student/CLionProjects/graphics_engine/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Linking CXX executable engine"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/engine.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/engine.dir/build: engine
.PHONY : CMakeFiles/engine.dir/build

CMakeFiles/engine.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/engine.dir/cmake_clean.cmake
.PHONY : CMakeFiles/engine.dir/clean

CMakeFiles/engine.dir/depend:
	cd /home/student/CLionProjects/graphics_engine/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/student/CLionProjects/graphics_engine /home/student/CLionProjects/graphics_engine /home/student/CLionProjects/graphics_engine/cmake-build-debug /home/student/CLionProjects/graphics_engine/cmake-build-debug /home/student/CLionProjects/graphics_engine/cmake-build-debug/CMakeFiles/engine.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/engine.dir/depend

