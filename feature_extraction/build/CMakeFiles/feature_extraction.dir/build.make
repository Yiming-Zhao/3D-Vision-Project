# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/yiming/Desktop/3D_Vision/feature_extraction

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yiming/Desktop/3D_Vision/feature_extraction/build

# Include any dependencies generated for this target.
include CMakeFiles/feature_extraction.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/feature_extraction.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/feature_extraction.dir/flags.make

CMakeFiles/feature_extraction.dir/feature_extraction.cpp.o: CMakeFiles/feature_extraction.dir/flags.make
CMakeFiles/feature_extraction.dir/feature_extraction.cpp.o: ../feature_extraction.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yiming/Desktop/3D_Vision/feature_extraction/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/feature_extraction.dir/feature_extraction.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/feature_extraction.dir/feature_extraction.cpp.o -c /home/yiming/Desktop/3D_Vision/feature_extraction/feature_extraction.cpp

CMakeFiles/feature_extraction.dir/feature_extraction.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/feature_extraction.dir/feature_extraction.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yiming/Desktop/3D_Vision/feature_extraction/feature_extraction.cpp > CMakeFiles/feature_extraction.dir/feature_extraction.cpp.i

CMakeFiles/feature_extraction.dir/feature_extraction.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/feature_extraction.dir/feature_extraction.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yiming/Desktop/3D_Vision/feature_extraction/feature_extraction.cpp -o CMakeFiles/feature_extraction.dir/feature_extraction.cpp.s

CMakeFiles/feature_extraction.dir/feature_extraction.cpp.o.requires:

.PHONY : CMakeFiles/feature_extraction.dir/feature_extraction.cpp.o.requires

CMakeFiles/feature_extraction.dir/feature_extraction.cpp.o.provides: CMakeFiles/feature_extraction.dir/feature_extraction.cpp.o.requires
	$(MAKE) -f CMakeFiles/feature_extraction.dir/build.make CMakeFiles/feature_extraction.dir/feature_extraction.cpp.o.provides.build
.PHONY : CMakeFiles/feature_extraction.dir/feature_extraction.cpp.o.provides

CMakeFiles/feature_extraction.dir/feature_extraction.cpp.o.provides.build: CMakeFiles/feature_extraction.dir/feature_extraction.cpp.o


# Object files for target feature_extraction
feature_extraction_OBJECTS = \
"CMakeFiles/feature_extraction.dir/feature_extraction.cpp.o"

# External object files for target feature_extraction
feature_extraction_EXTERNAL_OBJECTS =

feature_extraction: CMakeFiles/feature_extraction.dir/feature_extraction.cpp.o
feature_extraction: CMakeFiles/feature_extraction.dir/build.make
feature_extraction: /usr/lib/x86_64-linux-gnu/libglog.so
feature_extraction: /usr/lib/x86_64-linux-gnu/libfreeimage.so
feature_extraction: /usr/local/lib/libceres.a
feature_extraction: /usr/lib/x86_64-linux-gnu/libGL.so
feature_extraction: /usr/lib/x86_64-linux-gnu/libGLU.so
feature_extraction: /usr/lib/x86_64-linux-gnu/libGLEW.so
feature_extraction: /usr/lib/x86_64-linux-gnu/libQt5OpenGL.so.5.9.5
feature_extraction: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.9.5
feature_extraction: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
feature_extraction: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
feature_extraction: /usr/lib/x86_64-linux-gnu/libboost_regex.so
feature_extraction: /usr/lib/x86_64-linux-gnu/libboost_system.so
feature_extraction: /usr/lib/x86_64-linux-gnu/libCGAL.so.13.0.1
feature_extraction: /usr/lib/x86_64-linux-gnu/libgmp.so
feature_extraction: /usr/local/lib/libsqlite3.so
feature_extraction: /usr/lib/x86_64-linux-gnu/libglog.so
feature_extraction: /usr/lib/x86_64-linux-gnu/libgflags.so.2.2.1
feature_extraction: /usr/lib/x86_64-linux-gnu/libspqr.so
feature_extraction: /usr/lib/x86_64-linux-gnu/libtbbmalloc.so
feature_extraction: /usr/lib/x86_64-linux-gnu/libtbb.so
feature_extraction: /usr/lib/x86_64-linux-gnu/libcholmod.so
feature_extraction: /usr/lib/x86_64-linux-gnu/libccolamd.so
feature_extraction: /usr/lib/x86_64-linux-gnu/libcamd.so
feature_extraction: /usr/lib/x86_64-linux-gnu/libcolamd.so
feature_extraction: /usr/lib/x86_64-linux-gnu/libamd.so
feature_extraction: /usr/lib/x86_64-linux-gnu/liblapack.so
feature_extraction: /usr/lib/x86_64-linux-gnu/libf77blas.so
feature_extraction: /usr/lib/x86_64-linux-gnu/libatlas.so
feature_extraction: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
feature_extraction: /usr/lib/x86_64-linux-gnu/librt.so
feature_extraction: /usr/lib/x86_64-linux-gnu/libcxsparse.so
feature_extraction: /usr/lib/x86_64-linux-gnu/liblapack.so
feature_extraction: /usr/lib/x86_64-linux-gnu/libf77blas.so
feature_extraction: /usr/lib/x86_64-linux-gnu/libatlas.so
feature_extraction: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
feature_extraction: /usr/lib/x86_64-linux-gnu/librt.so
feature_extraction: /usr/lib/x86_64-linux-gnu/libcxsparse.so
feature_extraction: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.9.5
feature_extraction: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.9.5
feature_extraction: CMakeFiles/feature_extraction.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yiming/Desktop/3D_Vision/feature_extraction/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable feature_extraction"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/feature_extraction.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/feature_extraction.dir/build: feature_extraction

.PHONY : CMakeFiles/feature_extraction.dir/build

CMakeFiles/feature_extraction.dir/requires: CMakeFiles/feature_extraction.dir/feature_extraction.cpp.o.requires

.PHONY : CMakeFiles/feature_extraction.dir/requires

CMakeFiles/feature_extraction.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/feature_extraction.dir/cmake_clean.cmake
.PHONY : CMakeFiles/feature_extraction.dir/clean

CMakeFiles/feature_extraction.dir/depend:
	cd /home/yiming/Desktop/3D_Vision/feature_extraction/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yiming/Desktop/3D_Vision/feature_extraction /home/yiming/Desktop/3D_Vision/feature_extraction /home/yiming/Desktop/3D_Vision/feature_extraction/build /home/yiming/Desktop/3D_Vision/feature_extraction/build /home/yiming/Desktop/3D_Vision/feature_extraction/build/CMakeFiles/feature_extraction.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/feature_extraction.dir/depend

