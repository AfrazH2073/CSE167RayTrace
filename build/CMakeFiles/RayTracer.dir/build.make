# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.31

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
CMAKE_COMMAND = /opt/homebrew/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/ahameed/Documents/GitHub/CSE167RayTrace

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/ahameed/Documents/GitHub/CSE167RayTrace/build

# Include any dependencies generated for this target.
include CMakeFiles/RayTracer.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/RayTracer.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/RayTracer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/RayTracer.dir/flags.make

CMakeFiles/RayTracer.dir/codegen:
.PHONY : CMakeFiles/RayTracer.dir/codegen

CMakeFiles/RayTracer.dir/main.cpp.o: CMakeFiles/RayTracer.dir/flags.make
CMakeFiles/RayTracer.dir/main.cpp.o: /Users/ahameed/Documents/GitHub/CSE167RayTrace/main.cpp
CMakeFiles/RayTracer.dir/main.cpp.o: CMakeFiles/RayTracer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/ahameed/Documents/GitHub/CSE167RayTrace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/RayTracer.dir/main.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RayTracer.dir/main.cpp.o -MF CMakeFiles/RayTracer.dir/main.cpp.o.d -o CMakeFiles/RayTracer.dir/main.cpp.o -c /Users/ahameed/Documents/GitHub/CSE167RayTrace/main.cpp

CMakeFiles/RayTracer.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/RayTracer.dir/main.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/ahameed/Documents/GitHub/CSE167RayTrace/main.cpp > CMakeFiles/RayTracer.dir/main.cpp.i

CMakeFiles/RayTracer.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/RayTracer.dir/main.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/ahameed/Documents/GitHub/CSE167RayTrace/main.cpp -o CMakeFiles/RayTracer.dir/main.cpp.s

CMakeFiles/RayTracer.dir/src/Camera.cpp.o: CMakeFiles/RayTracer.dir/flags.make
CMakeFiles/RayTracer.dir/src/Camera.cpp.o: /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/Camera.cpp
CMakeFiles/RayTracer.dir/src/Camera.cpp.o: CMakeFiles/RayTracer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/ahameed/Documents/GitHub/CSE167RayTrace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/RayTracer.dir/src/Camera.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RayTracer.dir/src/Camera.cpp.o -MF CMakeFiles/RayTracer.dir/src/Camera.cpp.o.d -o CMakeFiles/RayTracer.dir/src/Camera.cpp.o -c /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/Camera.cpp

CMakeFiles/RayTracer.dir/src/Camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/RayTracer.dir/src/Camera.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/Camera.cpp > CMakeFiles/RayTracer.dir/src/Camera.cpp.i

CMakeFiles/RayTracer.dir/src/Camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/RayTracer.dir/src/Camera.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/Camera.cpp -o CMakeFiles/RayTracer.dir/src/Camera.cpp.s

CMakeFiles/RayTracer.dir/src/RayTracer.cpp.o: CMakeFiles/RayTracer.dir/flags.make
CMakeFiles/RayTracer.dir/src/RayTracer.cpp.o: /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/RayTracer.cpp
CMakeFiles/RayTracer.dir/src/RayTracer.cpp.o: CMakeFiles/RayTracer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/ahameed/Documents/GitHub/CSE167RayTrace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/RayTracer.dir/src/RayTracer.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RayTracer.dir/src/RayTracer.cpp.o -MF CMakeFiles/RayTracer.dir/src/RayTracer.cpp.o.d -o CMakeFiles/RayTracer.dir/src/RayTracer.cpp.o -c /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/RayTracer.cpp

CMakeFiles/RayTracer.dir/src/RayTracer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/RayTracer.dir/src/RayTracer.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/RayTracer.cpp > CMakeFiles/RayTracer.dir/src/RayTracer.cpp.i

CMakeFiles/RayTracer.dir/src/RayTracer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/RayTracer.dir/src/RayTracer.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/RayTracer.cpp -o CMakeFiles/RayTracer.dir/src/RayTracer.cpp.s

CMakeFiles/RayTracer.dir/src/Scene.cpp.o: CMakeFiles/RayTracer.dir/flags.make
CMakeFiles/RayTracer.dir/src/Scene.cpp.o: /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/Scene.cpp
CMakeFiles/RayTracer.dir/src/Scene.cpp.o: CMakeFiles/RayTracer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/ahameed/Documents/GitHub/CSE167RayTrace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/RayTracer.dir/src/Scene.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RayTracer.dir/src/Scene.cpp.o -MF CMakeFiles/RayTracer.dir/src/Scene.cpp.o.d -o CMakeFiles/RayTracer.dir/src/Scene.cpp.o -c /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/Scene.cpp

CMakeFiles/RayTracer.dir/src/Scene.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/RayTracer.dir/src/Scene.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/Scene.cpp > CMakeFiles/RayTracer.dir/src/Scene.cpp.i

CMakeFiles/RayTracer.dir/src/Scene.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/RayTracer.dir/src/Scene.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/Scene.cpp -o CMakeFiles/RayTracer.dir/src/Scene.cpp.s

CMakeFiles/RayTracer.dir/src/geometries/GeomSphere.cpp.o: CMakeFiles/RayTracer.dir/flags.make
CMakeFiles/RayTracer.dir/src/geometries/GeomSphere.cpp.o: /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/geometries/GeomSphere.cpp
CMakeFiles/RayTracer.dir/src/geometries/GeomSphere.cpp.o: CMakeFiles/RayTracer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/ahameed/Documents/GitHub/CSE167RayTrace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/RayTracer.dir/src/geometries/GeomSphere.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RayTracer.dir/src/geometries/GeomSphere.cpp.o -MF CMakeFiles/RayTracer.dir/src/geometries/GeomSphere.cpp.o.d -o CMakeFiles/RayTracer.dir/src/geometries/GeomSphere.cpp.o -c /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/geometries/GeomSphere.cpp

CMakeFiles/RayTracer.dir/src/geometries/GeomSphere.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/RayTracer.dir/src/geometries/GeomSphere.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/geometries/GeomSphere.cpp > CMakeFiles/RayTracer.dir/src/geometries/GeomSphere.cpp.i

CMakeFiles/RayTracer.dir/src/geometries/GeomSphere.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/RayTracer.dir/src/geometries/GeomSphere.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/geometries/GeomSphere.cpp -o CMakeFiles/RayTracer.dir/src/geometries/GeomSphere.cpp.s

CMakeFiles/RayTracer.dir/src/geometries/GeomTriangle.cpp.o: CMakeFiles/RayTracer.dir/flags.make
CMakeFiles/RayTracer.dir/src/geometries/GeomTriangle.cpp.o: /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/geometries/GeomTriangle.cpp
CMakeFiles/RayTracer.dir/src/geometries/GeomTriangle.cpp.o: CMakeFiles/RayTracer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/ahameed/Documents/GitHub/CSE167RayTrace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/RayTracer.dir/src/geometries/GeomTriangle.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RayTracer.dir/src/geometries/GeomTriangle.cpp.o -MF CMakeFiles/RayTracer.dir/src/geometries/GeomTriangle.cpp.o.d -o CMakeFiles/RayTracer.dir/src/geometries/GeomTriangle.cpp.o -c /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/geometries/GeomTriangle.cpp

CMakeFiles/RayTracer.dir/src/geometries/GeomTriangle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/RayTracer.dir/src/geometries/GeomTriangle.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/geometries/GeomTriangle.cpp > CMakeFiles/RayTracer.dir/src/geometries/GeomTriangle.cpp.i

CMakeFiles/RayTracer.dir/src/geometries/GeomTriangle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/RayTracer.dir/src/geometries/GeomTriangle.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/geometries/GeomTriangle.cpp -o CMakeFiles/RayTracer.dir/src/geometries/GeomTriangle.cpp.s

CMakeFiles/RayTracer.dir/src/materials/GlossyMaterial.cpp.o: CMakeFiles/RayTracer.dir/flags.make
CMakeFiles/RayTracer.dir/src/materials/GlossyMaterial.cpp.o: /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/materials/GlossyMaterial.cpp
CMakeFiles/RayTracer.dir/src/materials/GlossyMaterial.cpp.o: CMakeFiles/RayTracer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/ahameed/Documents/GitHub/CSE167RayTrace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/RayTracer.dir/src/materials/GlossyMaterial.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RayTracer.dir/src/materials/GlossyMaterial.cpp.o -MF CMakeFiles/RayTracer.dir/src/materials/GlossyMaterial.cpp.o.d -o CMakeFiles/RayTracer.dir/src/materials/GlossyMaterial.cpp.o -c /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/materials/GlossyMaterial.cpp

CMakeFiles/RayTracer.dir/src/materials/GlossyMaterial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/RayTracer.dir/src/materials/GlossyMaterial.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/materials/GlossyMaterial.cpp > CMakeFiles/RayTracer.dir/src/materials/GlossyMaterial.cpp.i

CMakeFiles/RayTracer.dir/src/materials/GlossyMaterial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/RayTracer.dir/src/materials/GlossyMaterial.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/materials/GlossyMaterial.cpp -o CMakeFiles/RayTracer.dir/src/materials/GlossyMaterial.cpp.s

CMakeFiles/RayTracer.dir/src/models/Sphere.cpp.o: CMakeFiles/RayTracer.dir/flags.make
CMakeFiles/RayTracer.dir/src/models/Sphere.cpp.o: /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/models/Sphere.cpp
CMakeFiles/RayTracer.dir/src/models/Sphere.cpp.o: CMakeFiles/RayTracer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/ahameed/Documents/GitHub/CSE167RayTrace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/RayTracer.dir/src/models/Sphere.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RayTracer.dir/src/models/Sphere.cpp.o -MF CMakeFiles/RayTracer.dir/src/models/Sphere.cpp.o.d -o CMakeFiles/RayTracer.dir/src/models/Sphere.cpp.o -c /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/models/Sphere.cpp

CMakeFiles/RayTracer.dir/src/models/Sphere.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/RayTracer.dir/src/models/Sphere.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/models/Sphere.cpp > CMakeFiles/RayTracer.dir/src/models/Sphere.cpp.i

CMakeFiles/RayTracer.dir/src/models/Sphere.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/RayTracer.dir/src/models/Sphere.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/models/Sphere.cpp -o CMakeFiles/RayTracer.dir/src/models/Sphere.cpp.s

CMakeFiles/RayTracer.dir/src/models/Square.cpp.o: CMakeFiles/RayTracer.dir/flags.make
CMakeFiles/RayTracer.dir/src/models/Square.cpp.o: /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/models/Square.cpp
CMakeFiles/RayTracer.dir/src/models/Square.cpp.o: CMakeFiles/RayTracer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/ahameed/Documents/GitHub/CSE167RayTrace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/RayTracer.dir/src/models/Square.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RayTracer.dir/src/models/Square.cpp.o -MF CMakeFiles/RayTracer.dir/src/models/Square.cpp.o.d -o CMakeFiles/RayTracer.dir/src/models/Square.cpp.o -c /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/models/Square.cpp

CMakeFiles/RayTracer.dir/src/models/Square.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/RayTracer.dir/src/models/Square.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/models/Square.cpp > CMakeFiles/RayTracer.dir/src/models/Square.cpp.i

CMakeFiles/RayTracer.dir/src/models/Square.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/RayTracer.dir/src/models/Square.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/models/Square.cpp -o CMakeFiles/RayTracer.dir/src/models/Square.cpp.s

CMakeFiles/RayTracer.dir/src/models/Tetrahedron.cpp.o: CMakeFiles/RayTracer.dir/flags.make
CMakeFiles/RayTracer.dir/src/models/Tetrahedron.cpp.o: /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/models/Tetrahedron.cpp
CMakeFiles/RayTracer.dir/src/models/Tetrahedron.cpp.o: CMakeFiles/RayTracer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/ahameed/Documents/GitHub/CSE167RayTrace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/RayTracer.dir/src/models/Tetrahedron.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/RayTracer.dir/src/models/Tetrahedron.cpp.o -MF CMakeFiles/RayTracer.dir/src/models/Tetrahedron.cpp.o.d -o CMakeFiles/RayTracer.dir/src/models/Tetrahedron.cpp.o -c /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/models/Tetrahedron.cpp

CMakeFiles/RayTracer.dir/src/models/Tetrahedron.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/RayTracer.dir/src/models/Tetrahedron.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/models/Tetrahedron.cpp > CMakeFiles/RayTracer.dir/src/models/Tetrahedron.cpp.i

CMakeFiles/RayTracer.dir/src/models/Tetrahedron.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/RayTracer.dir/src/models/Tetrahedron.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/ahameed/Documents/GitHub/CSE167RayTrace/src/models/Tetrahedron.cpp -o CMakeFiles/RayTracer.dir/src/models/Tetrahedron.cpp.s

# Object files for target RayTracer
RayTracer_OBJECTS = \
"CMakeFiles/RayTracer.dir/main.cpp.o" \
"CMakeFiles/RayTracer.dir/src/Camera.cpp.o" \
"CMakeFiles/RayTracer.dir/src/RayTracer.cpp.o" \
"CMakeFiles/RayTracer.dir/src/Scene.cpp.o" \
"CMakeFiles/RayTracer.dir/src/geometries/GeomSphere.cpp.o" \
"CMakeFiles/RayTracer.dir/src/geometries/GeomTriangle.cpp.o" \
"CMakeFiles/RayTracer.dir/src/materials/GlossyMaterial.cpp.o" \
"CMakeFiles/RayTracer.dir/src/models/Sphere.cpp.o" \
"CMakeFiles/RayTracer.dir/src/models/Square.cpp.o" \
"CMakeFiles/RayTracer.dir/src/models/Tetrahedron.cpp.o"

# External object files for target RayTracer
RayTracer_EXTERNAL_OBJECTS =

/Users/ahameed/Documents/GitHub/CSE167RayTrace/bin/RayTracer: CMakeFiles/RayTracer.dir/main.cpp.o
/Users/ahameed/Documents/GitHub/CSE167RayTrace/bin/RayTracer: CMakeFiles/RayTracer.dir/src/Camera.cpp.o
/Users/ahameed/Documents/GitHub/CSE167RayTrace/bin/RayTracer: CMakeFiles/RayTracer.dir/src/RayTracer.cpp.o
/Users/ahameed/Documents/GitHub/CSE167RayTrace/bin/RayTracer: CMakeFiles/RayTracer.dir/src/Scene.cpp.o
/Users/ahameed/Documents/GitHub/CSE167RayTrace/bin/RayTracer: CMakeFiles/RayTracer.dir/src/geometries/GeomSphere.cpp.o
/Users/ahameed/Documents/GitHub/CSE167RayTrace/bin/RayTracer: CMakeFiles/RayTracer.dir/src/geometries/GeomTriangle.cpp.o
/Users/ahameed/Documents/GitHub/CSE167RayTrace/bin/RayTracer: CMakeFiles/RayTracer.dir/src/materials/GlossyMaterial.cpp.o
/Users/ahameed/Documents/GitHub/CSE167RayTrace/bin/RayTracer: CMakeFiles/RayTracer.dir/src/models/Sphere.cpp.o
/Users/ahameed/Documents/GitHub/CSE167RayTrace/bin/RayTracer: CMakeFiles/RayTracer.dir/src/models/Square.cpp.o
/Users/ahameed/Documents/GitHub/CSE167RayTrace/bin/RayTracer: CMakeFiles/RayTracer.dir/src/models/Tetrahedron.cpp.o
/Users/ahameed/Documents/GitHub/CSE167RayTrace/bin/RayTracer: CMakeFiles/RayTracer.dir/build.make
/Users/ahameed/Documents/GitHub/CSE167RayTrace/bin/RayTracer: /opt/homebrew/lib/libfreeimage.dylib
/Users/ahameed/Documents/GitHub/CSE167RayTrace/bin/RayTracer: CMakeFiles/RayTracer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/Users/ahameed/Documents/GitHub/CSE167RayTrace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Linking CXX executable /Users/ahameed/Documents/GitHub/CSE167RayTrace/bin/RayTracer"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/RayTracer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/RayTracer.dir/build: /Users/ahameed/Documents/GitHub/CSE167RayTrace/bin/RayTracer
.PHONY : CMakeFiles/RayTracer.dir/build

CMakeFiles/RayTracer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/RayTracer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/RayTracer.dir/clean

CMakeFiles/RayTracer.dir/depend:
	cd /Users/ahameed/Documents/GitHub/CSE167RayTrace/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/ahameed/Documents/GitHub/CSE167RayTrace /Users/ahameed/Documents/GitHub/CSE167RayTrace /Users/ahameed/Documents/GitHub/CSE167RayTrace/build /Users/ahameed/Documents/GitHub/CSE167RayTrace/build /Users/ahameed/Documents/GitHub/CSE167RayTrace/build/CMakeFiles/RayTracer.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/RayTracer.dir/depend

