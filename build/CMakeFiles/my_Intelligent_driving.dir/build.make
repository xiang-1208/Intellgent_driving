# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/xiang/Project/my_ID

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xiang/Project/my_ID/build

# Include any dependencies generated for this target.
include CMakeFiles/my_Intelligent_driving.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/my_Intelligent_driving.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/my_Intelligent_driving.dir/flags.make

CMakeFiles/my_Intelligent_driving.dir/src/Intel_dri_main.cpp.o: CMakeFiles/my_Intelligent_driving.dir/flags.make
CMakeFiles/my_Intelligent_driving.dir/src/Intel_dri_main.cpp.o: ../src/Intel_dri_main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xiang/Project/my_ID/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/my_Intelligent_driving.dir/src/Intel_dri_main.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/my_Intelligent_driving.dir/src/Intel_dri_main.cpp.o -c /home/xiang/Project/my_ID/src/Intel_dri_main.cpp

CMakeFiles/my_Intelligent_driving.dir/src/Intel_dri_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my_Intelligent_driving.dir/src/Intel_dri_main.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-5 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xiang/Project/my_ID/src/Intel_dri_main.cpp > CMakeFiles/my_Intelligent_driving.dir/src/Intel_dri_main.cpp.i

CMakeFiles/my_Intelligent_driving.dir/src/Intel_dri_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my_Intelligent_driving.dir/src/Intel_dri_main.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-5 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xiang/Project/my_ID/src/Intel_dri_main.cpp -o CMakeFiles/my_Intelligent_driving.dir/src/Intel_dri_main.cpp.s

# Object files for target my_Intelligent_driving
my_Intelligent_driving_OBJECTS = \
"CMakeFiles/my_Intelligent_driving.dir/src/Intel_dri_main.cpp.o"

# External object files for target my_Intelligent_driving
my_Intelligent_driving_EXTERNAL_OBJECTS =

my_Intelligent_driving: CMakeFiles/my_Intelligent_driving.dir/src/Intel_dri_main.cpp.o
my_Intelligent_driving: CMakeFiles/my_Intelligent_driving.dir/build.make
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stitching3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_superres3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videostab3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_aruco3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bgsegm3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bioinspired3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ccalib3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_cvv3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dpm3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_face3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_fuzzy3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_hdf3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_img_hash3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_line_descriptor3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_optflow3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_reg3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_rgbd3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_saliency3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stereo3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_structured_light3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_surface_matching3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_tracking3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xfeatures2d3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ximgproc3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xobjdetect3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xphoto3.so.3.3.1
my_Intelligent_driving: libmy_lib.a
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_shape3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_photo3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_datasets3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_plot3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_text3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dnn3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ml3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_video3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_viz3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_phase_unwrapping3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_objdetect3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
my_Intelligent_driving: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
my_Intelligent_driving: CMakeFiles/my_Intelligent_driving.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/xiang/Project/my_ID/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable my_Intelligent_driving"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/my_Intelligent_driving.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/my_Intelligent_driving.dir/build: my_Intelligent_driving

.PHONY : CMakeFiles/my_Intelligent_driving.dir/build

CMakeFiles/my_Intelligent_driving.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/my_Intelligent_driving.dir/cmake_clean.cmake
.PHONY : CMakeFiles/my_Intelligent_driving.dir/clean

CMakeFiles/my_Intelligent_driving.dir/depend:
	cd /home/xiang/Project/my_ID/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xiang/Project/my_ID /home/xiang/Project/my_ID /home/xiang/Project/my_ID/build /home/xiang/Project/my_ID/build /home/xiang/Project/my_ID/build/CMakeFiles/my_Intelligent_driving.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/my_Intelligent_driving.dir/depend

