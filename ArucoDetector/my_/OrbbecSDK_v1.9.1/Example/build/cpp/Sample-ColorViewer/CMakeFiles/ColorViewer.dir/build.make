# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/xchen/zhongda/arm/OrbbecSDK_v1.9.1/Example

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xchen/zhongda/arm/OrbbecSDK_v1.9.1/Example/build

# Include any dependencies generated for this target.
include cpp/Sample-ColorViewer/CMakeFiles/ColorViewer.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include cpp/Sample-ColorViewer/CMakeFiles/ColorViewer.dir/compiler_depend.make

# Include the progress variables for this target.
include cpp/Sample-ColorViewer/CMakeFiles/ColorViewer.dir/progress.make

# Include the compile flags for this target's objects.
include cpp/Sample-ColorViewer/CMakeFiles/ColorViewer.dir/flags.make

cpp/Sample-ColorViewer/CMakeFiles/ColorViewer.dir/ColorViewer.cpp.o: cpp/Sample-ColorViewer/CMakeFiles/ColorViewer.dir/flags.make
cpp/Sample-ColorViewer/CMakeFiles/ColorViewer.dir/ColorViewer.cpp.o: ../cpp/Sample-ColorViewer/ColorViewer.cpp
cpp/Sample-ColorViewer/CMakeFiles/ColorViewer.dir/ColorViewer.cpp.o: cpp/Sample-ColorViewer/CMakeFiles/ColorViewer.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xchen/zhongda/arm/OrbbecSDK_v1.9.1/Example/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object cpp/Sample-ColorViewer/CMakeFiles/ColorViewer.dir/ColorViewer.cpp.o"
	cd /home/xchen/zhongda/arm/OrbbecSDK_v1.9.1/Example/build/cpp/Sample-ColorViewer && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT cpp/Sample-ColorViewer/CMakeFiles/ColorViewer.dir/ColorViewer.cpp.o -MF CMakeFiles/ColorViewer.dir/ColorViewer.cpp.o.d -o CMakeFiles/ColorViewer.dir/ColorViewer.cpp.o -c /home/xchen/zhongda/arm/OrbbecSDK_v1.9.1/Example/cpp/Sample-ColorViewer/ColorViewer.cpp

cpp/Sample-ColorViewer/CMakeFiles/ColorViewer.dir/ColorViewer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ColorViewer.dir/ColorViewer.cpp.i"
	cd /home/xchen/zhongda/arm/OrbbecSDK_v1.9.1/Example/build/cpp/Sample-ColorViewer && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xchen/zhongda/arm/OrbbecSDK_v1.9.1/Example/cpp/Sample-ColorViewer/ColorViewer.cpp > CMakeFiles/ColorViewer.dir/ColorViewer.cpp.i

cpp/Sample-ColorViewer/CMakeFiles/ColorViewer.dir/ColorViewer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ColorViewer.dir/ColorViewer.cpp.s"
	cd /home/xchen/zhongda/arm/OrbbecSDK_v1.9.1/Example/build/cpp/Sample-ColorViewer && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xchen/zhongda/arm/OrbbecSDK_v1.9.1/Example/cpp/Sample-ColorViewer/ColorViewer.cpp -o CMakeFiles/ColorViewer.dir/ColorViewer.cpp.s

# Object files for target ColorViewer
ColorViewer_OBJECTS = \
"CMakeFiles/ColorViewer.dir/ColorViewer.cpp.o"

# External object files for target ColorViewer
ColorViewer_EXTERNAL_OBJECTS =

bin/ColorViewer: cpp/Sample-ColorViewer/CMakeFiles/ColorViewer.dir/ColorViewer.cpp.o
bin/ColorViewer: cpp/Sample-ColorViewer/CMakeFiles/ColorViewer.dir/build.make
bin/ColorViewer: /usr/local/lib/libopencv_gapi.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_stitching.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_alphamat.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_aruco.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_barcode.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_bgsegm.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_bioinspired.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_ccalib.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_dnn_objdetect.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_dnn_superres.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_dpm.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_face.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_freetype.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_fuzzy.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_hdf.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_hfs.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_img_hash.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_intensity_transform.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_line_descriptor.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_mcc.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_quality.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_rapid.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_reg.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_rgbd.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_saliency.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_stereo.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_structured_light.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_superres.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_surface_matching.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_tracking.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_videostab.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_viz.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_wechat_qrcode.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_xfeatures2d.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_xobjdetect.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_xphoto.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_shape.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_highgui.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_datasets.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_plot.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_text.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_ml.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_phase_unwrapping.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_optflow.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_ximgproc.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_video.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_videoio.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_imgcodecs.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_objdetect.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_calib3d.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_dnn.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_features2d.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_flann.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_photo.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_imgproc.so.4.6.0
bin/ColorViewer: /usr/local/lib/libopencv_core.so.4.6.0
bin/ColorViewer: cpp/Sample-ColorViewer/CMakeFiles/ColorViewer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/xchen/zhongda/arm/OrbbecSDK_v1.9.1/Example/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/ColorViewer"
	cd /home/xchen/zhongda/arm/OrbbecSDK_v1.9.1/Example/build/cpp/Sample-ColorViewer && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ColorViewer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
cpp/Sample-ColorViewer/CMakeFiles/ColorViewer.dir/build: bin/ColorViewer
.PHONY : cpp/Sample-ColorViewer/CMakeFiles/ColorViewer.dir/build

cpp/Sample-ColorViewer/CMakeFiles/ColorViewer.dir/clean:
	cd /home/xchen/zhongda/arm/OrbbecSDK_v1.9.1/Example/build/cpp/Sample-ColorViewer && $(CMAKE_COMMAND) -P CMakeFiles/ColorViewer.dir/cmake_clean.cmake
.PHONY : cpp/Sample-ColorViewer/CMakeFiles/ColorViewer.dir/clean

cpp/Sample-ColorViewer/CMakeFiles/ColorViewer.dir/depend:
	cd /home/xchen/zhongda/arm/OrbbecSDK_v1.9.1/Example/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xchen/zhongda/arm/OrbbecSDK_v1.9.1/Example /home/xchen/zhongda/arm/OrbbecSDK_v1.9.1/Example/cpp/Sample-ColorViewer /home/xchen/zhongda/arm/OrbbecSDK_v1.9.1/Example/build /home/xchen/zhongda/arm/OrbbecSDK_v1.9.1/Example/build/cpp/Sample-ColorViewer /home/xchen/zhongda/arm/OrbbecSDK_v1.9.1/Example/build/cpp/Sample-ColorViewer/CMakeFiles/ColorViewer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : cpp/Sample-ColorViewer/CMakeFiles/ColorViewer.dir/depend
