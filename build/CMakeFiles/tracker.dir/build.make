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
CMAKE_SOURCE_DIR = /media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2/build

# Include any dependencies generated for this target.
include CMakeFiles/tracker.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/tracker.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/tracker.dir/flags.make

CMakeFiles/tracker.dir/src/tracker/KalmanFilter.cpp.o: CMakeFiles/tracker.dir/flags.make
CMakeFiles/tracker.dir/src/tracker/KalmanFilter.cpp.o: ../src/tracker/KalmanFilter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/tracker.dir/src/tracker/KalmanFilter.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tracker.dir/src/tracker/KalmanFilter.cpp.o -c /media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2/src/tracker/KalmanFilter.cpp

CMakeFiles/tracker.dir/src/tracker/KalmanFilter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracker.dir/src/tracker/KalmanFilter.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2/src/tracker/KalmanFilter.cpp > CMakeFiles/tracker.dir/src/tracker/KalmanFilter.cpp.i

CMakeFiles/tracker.dir/src/tracker/KalmanFilter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracker.dir/src/tracker/KalmanFilter.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2/src/tracker/KalmanFilter.cpp -o CMakeFiles/tracker.dir/src/tracker/KalmanFilter.cpp.s

CMakeFiles/tracker.dir/src/tracker/KalmanFilter.cpp.o.requires:

.PHONY : CMakeFiles/tracker.dir/src/tracker/KalmanFilter.cpp.o.requires

CMakeFiles/tracker.dir/src/tracker/KalmanFilter.cpp.o.provides: CMakeFiles/tracker.dir/src/tracker/KalmanFilter.cpp.o.requires
	$(MAKE) -f CMakeFiles/tracker.dir/build.make CMakeFiles/tracker.dir/src/tracker/KalmanFilter.cpp.o.provides.build
.PHONY : CMakeFiles/tracker.dir/src/tracker/KalmanFilter.cpp.o.provides

CMakeFiles/tracker.dir/src/tracker/KalmanFilter.cpp.o.provides.build: CMakeFiles/tracker.dir/src/tracker/KalmanFilter.cpp.o


CMakeFiles/tracker.dir/src/tracker/Tracklet.cpp.o: CMakeFiles/tracker.dir/flags.make
CMakeFiles/tracker.dir/src/tracker/Tracklet.cpp.o: ../src/tracker/Tracklet.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/tracker.dir/src/tracker/Tracklet.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tracker.dir/src/tracker/Tracklet.cpp.o -c /media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2/src/tracker/Tracklet.cpp

CMakeFiles/tracker.dir/src/tracker/Tracklet.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracker.dir/src/tracker/Tracklet.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2/src/tracker/Tracklet.cpp > CMakeFiles/tracker.dir/src/tracker/Tracklet.cpp.i

CMakeFiles/tracker.dir/src/tracker/Tracklet.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracker.dir/src/tracker/Tracklet.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2/src/tracker/Tracklet.cpp -o CMakeFiles/tracker.dir/src/tracker/Tracklet.cpp.s

CMakeFiles/tracker.dir/src/tracker/Tracklet.cpp.o.requires:

.PHONY : CMakeFiles/tracker.dir/src/tracker/Tracklet.cpp.o.requires

CMakeFiles/tracker.dir/src/tracker/Tracklet.cpp.o.provides: CMakeFiles/tracker.dir/src/tracker/Tracklet.cpp.o.requires
	$(MAKE) -f CMakeFiles/tracker.dir/build.make CMakeFiles/tracker.dir/src/tracker/Tracklet.cpp.o.provides.build
.PHONY : CMakeFiles/tracker.dir/src/tracker/Tracklet.cpp.o.provides

CMakeFiles/tracker.dir/src/tracker/Tracklet.cpp.o.provides.build: CMakeFiles/tracker.dir/src/tracker/Tracklet.cpp.o


CMakeFiles/tracker.dir/src/tracker/Tracker.cpp.o: CMakeFiles/tracker.dir/flags.make
CMakeFiles/tracker.dir/src/tracker/Tracker.cpp.o: ../src/tracker/Tracker.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/tracker.dir/src/tracker/Tracker.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tracker.dir/src/tracker/Tracker.cpp.o -c /media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2/src/tracker/Tracker.cpp

CMakeFiles/tracker.dir/src/tracker/Tracker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracker.dir/src/tracker/Tracker.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2/src/tracker/Tracker.cpp > CMakeFiles/tracker.dir/src/tracker/Tracker.cpp.i

CMakeFiles/tracker.dir/src/tracker/Tracker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracker.dir/src/tracker/Tracker.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2/src/tracker/Tracker.cpp -o CMakeFiles/tracker.dir/src/tracker/Tracker.cpp.s

CMakeFiles/tracker.dir/src/tracker/Tracker.cpp.o.requires:

.PHONY : CMakeFiles/tracker.dir/src/tracker/Tracker.cpp.o.requires

CMakeFiles/tracker.dir/src/tracker/Tracker.cpp.o.provides: CMakeFiles/tracker.dir/src/tracker/Tracker.cpp.o.requires
	$(MAKE) -f CMakeFiles/tracker.dir/build.make CMakeFiles/tracker.dir/src/tracker/Tracker.cpp.o.provides.build
.PHONY : CMakeFiles/tracker.dir/src/tracker/Tracker.cpp.o.provides

CMakeFiles/tracker.dir/src/tracker/Tracker.cpp.o.provides.build: CMakeFiles/tracker.dir/src/tracker/Tracker.cpp.o


CMakeFiles/tracker.dir/src/viewer/Renderer.cpp.o: CMakeFiles/tracker.dir/flags.make
CMakeFiles/tracker.dir/src/viewer/Renderer.cpp.o: ../src/viewer/Renderer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/tracker.dir/src/viewer/Renderer.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tracker.dir/src/viewer/Renderer.cpp.o -c /media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2/src/viewer/Renderer.cpp

CMakeFiles/tracker.dir/src/viewer/Renderer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracker.dir/src/viewer/Renderer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2/src/viewer/Renderer.cpp > CMakeFiles/tracker.dir/src/viewer/Renderer.cpp.i

CMakeFiles/tracker.dir/src/viewer/Renderer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracker.dir/src/viewer/Renderer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2/src/viewer/Renderer.cpp -o CMakeFiles/tracker.dir/src/viewer/Renderer.cpp.s

CMakeFiles/tracker.dir/src/viewer/Renderer.cpp.o.requires:

.PHONY : CMakeFiles/tracker.dir/src/viewer/Renderer.cpp.o.requires

CMakeFiles/tracker.dir/src/viewer/Renderer.cpp.o.provides: CMakeFiles/tracker.dir/src/viewer/Renderer.cpp.o.requires
	$(MAKE) -f CMakeFiles/tracker.dir/build.make CMakeFiles/tracker.dir/src/viewer/Renderer.cpp.o.provides.build
.PHONY : CMakeFiles/tracker.dir/src/viewer/Renderer.cpp.o.provides

CMakeFiles/tracker.dir/src/viewer/Renderer.cpp.o.provides.build: CMakeFiles/tracker.dir/src/viewer/Renderer.cpp.o


CMakeFiles/tracker.dir/src/CloudManager.cpp.o: CMakeFiles/tracker.dir/flags.make
CMakeFiles/tracker.dir/src/CloudManager.cpp.o: ../src/CloudManager.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/tracker.dir/src/CloudManager.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tracker.dir/src/CloudManager.cpp.o -c /media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2/src/CloudManager.cpp

CMakeFiles/tracker.dir/src/CloudManager.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracker.dir/src/CloudManager.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2/src/CloudManager.cpp > CMakeFiles/tracker.dir/src/CloudManager.cpp.i

CMakeFiles/tracker.dir/src/CloudManager.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracker.dir/src/CloudManager.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2/src/CloudManager.cpp -o CMakeFiles/tracker.dir/src/CloudManager.cpp.s

CMakeFiles/tracker.dir/src/CloudManager.cpp.o.requires:

.PHONY : CMakeFiles/tracker.dir/src/CloudManager.cpp.o.requires

CMakeFiles/tracker.dir/src/CloudManager.cpp.o.provides: CMakeFiles/tracker.dir/src/CloudManager.cpp.o.requires
	$(MAKE) -f CMakeFiles/tracker.dir/build.make CMakeFiles/tracker.dir/src/CloudManager.cpp.o.provides.build
.PHONY : CMakeFiles/tracker.dir/src/CloudManager.cpp.o.provides

CMakeFiles/tracker.dir/src/CloudManager.cpp.o.provides.build: CMakeFiles/tracker.dir/src/CloudManager.cpp.o


# Object files for target tracker
tracker_OBJECTS = \
"CMakeFiles/tracker.dir/src/tracker/KalmanFilter.cpp.o" \
"CMakeFiles/tracker.dir/src/tracker/Tracklet.cpp.o" \
"CMakeFiles/tracker.dir/src/tracker/Tracker.cpp.o" \
"CMakeFiles/tracker.dir/src/viewer/Renderer.cpp.o" \
"CMakeFiles/tracker.dir/src/CloudManager.cpp.o"

# External object files for target tracker
tracker_EXTERNAL_OBJECTS =

libtracker.so: CMakeFiles/tracker.dir/src/tracker/KalmanFilter.cpp.o
libtracker.so: CMakeFiles/tracker.dir/src/tracker/Tracklet.cpp.o
libtracker.so: CMakeFiles/tracker.dir/src/tracker/Tracker.cpp.o
libtracker.so: CMakeFiles/tracker.dir/src/viewer/Renderer.cpp.o
libtracker.so: CMakeFiles/tracker.dir/src/CloudManager.cpp.o
libtracker.so: CMakeFiles/tracker.dir/build.make
libtracker.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_common.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_octree.so
libtracker.so: /usr/lib/libOpenNI.so
libtracker.so: /usr/lib/libOpenNI2.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libfreetype.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libz.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libexpat.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpython2.7.so
libtracker.so: /usr/lib/libvtkWrappingTools-6.3.a
libtracker.so: /usr/lib/x86_64-linux-gnu/libjpeg.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpng.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libtiff.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libproj.so
libtracker.so: /usr/lib/x86_64-linux-gnu/hdf5/openmpi/libhdf5.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libsz.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libdl.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libm.so
libtracker.so: /usr/lib/x86_64-linux-gnu/openmpi/lib/libmpi.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libnetcdf_c++.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libnetcdf.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libgl2ps.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libtheoraenc.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libtheoradec.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libogg.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libxml2.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libjsoncpp.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_io.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_search.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_filters.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_features.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_ml.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_segmentation.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libqhull.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_surface.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_registration.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_keypoints.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_tracking.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_recognition.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_stereo.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_apps.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_outofcore.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_people.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpthread.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libqhull.so
libtracker.so: /usr/lib/libOpenNI.so
libtracker.so: /usr/lib/libOpenNI2.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
libtracker.so: /usr/lib/x86_64-linux-gnu/libfreetype.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libz.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkDomainsChemistry-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libexpat.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneric-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkFiltersHyperTree-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelFlowPaths-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelGeometry-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelImaging-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelMPI-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelStatistics-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkFiltersProgrammable-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkFiltersPython-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libpython2.7.so
libtracker.so: /usr/lib/libvtkWrappingTools-6.3.a
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkFiltersReebGraph-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkFiltersSMP-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkFiltersSelection-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkFiltersVerdict-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkverdict-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libjpeg.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpng.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libtiff.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQtOpenGL-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQtSQL-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQtWebkit-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkViewsQt-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libproj.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOAMR-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/hdf5/openmpi/libhdf5.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libsz.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libdl.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libm.so
libtracker.so: /usr/lib/x86_64-linux-gnu/openmpi/lib/libmpi.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOEnSight-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libnetcdf_c++.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libnetcdf.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOExport-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkRenderingGL2PS-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libgl2ps.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOFFMPEG-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOMovie-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libtheoraenc.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libtheoradec.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libogg.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOGDAL-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOGeoJSON-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOImport-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOInfovis-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libxml2.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOMINC-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOMPIImage-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOMPIParallel-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOParallel-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIONetCDF-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libjsoncpp.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOMySQL-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOODBC-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOParallelExodus-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOParallelLSDyna-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOParallelNetCDF-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOParallelXML-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOPostgreSQL-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOVPIC-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkVPIC-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOVideo-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOXdmf2-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkxdmf2-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkImagingMath-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkImagingMorphological-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkImagingStatistics-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkImagingStencil-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkInteractionImage-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkLocalExample-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkParallelMPI4Py-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkRenderingExternal-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeTypeFontConfig-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkRenderingImage-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkRenderingMatplotlib-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkRenderingParallel-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkRenderingParallelLIC-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkRenderingQt-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolumeAMR-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolumeOpenGL-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkTestingGenericBridge-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkTestingIOSQL-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkTestingRendering-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkViewsGeovis-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkWrappingJava-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_common.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_octree.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_io.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_search.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_filters.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_features.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_ml.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_segmentation.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_surface.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_registration.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_keypoints.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_tracking.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_recognition.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_stereo.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_apps.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_outofcore.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libpcl_people.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkFiltersFlowPaths-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libtheoraenc.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libtheoradec.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libogg.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOExodus-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkexoIIc-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libnetcdf_c++.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libnetcdf.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOLSDyna-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libxml2.so
libtracker.so: /usr/lib/x86_64-linux-gnu/hdf5/openmpi/libhdf5.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libsz.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libdl.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libm.so
libtracker.so: /usr/lib/x86_64-linux-gnu/openmpi/lib/libmpi.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkWrappingPython27Core-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkPythonInterpreter-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libpython2.7.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallel-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkParallelMPI-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkRenderingLIC-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkFiltersTexture-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQt-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.9.5
libtracker.so: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.9.5
libtracker.so: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.9.5
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkFiltersAMR-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkParallelCore-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libGLU.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libSM.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libICE.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libX11.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libXext.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libXt.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOSQL-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkViewsInfovis-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkFiltersImaging-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkRenderingLabel-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkGeovisCore-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOXML-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkInfovisLayout-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkInfovisBoostGraphAlgorithms-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOImage-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkIOCore-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkmetaio-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libz.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkftgl-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libfreetype.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libGL.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkalglib-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtksys-6.3.so.6.3.0
libtracker.so: /usr/lib/x86_64-linux-gnu/libproj.so
libtracker.so: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.3.so.6.3.0
libtracker.so: CMakeFiles/tracker.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX shared library libtracker.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tracker.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/tracker.dir/build: libtracker.so

.PHONY : CMakeFiles/tracker.dir/build

CMakeFiles/tracker.dir/requires: CMakeFiles/tracker.dir/src/tracker/KalmanFilter.cpp.o.requires
CMakeFiles/tracker.dir/requires: CMakeFiles/tracker.dir/src/tracker/Tracklet.cpp.o.requires
CMakeFiles/tracker.dir/requires: CMakeFiles/tracker.dir/src/tracker/Tracker.cpp.o.requires
CMakeFiles/tracker.dir/requires: CMakeFiles/tracker.dir/src/viewer/Renderer.cpp.o.requires
CMakeFiles/tracker.dir/requires: CMakeFiles/tracker.dir/src/CloudManager.cpp.o.requires

.PHONY : CMakeFiles/tracker.dir/requires

CMakeFiles/tracker.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/tracker.dir/cmake_clean.cmake
.PHONY : CMakeFiles/tracker.dir/clean

CMakeFiles/tracker.dir/depend:
	cd /media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2 /media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2 /media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2/build /media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2/build /media/sf_Guida_autonoma/Assignment/Assignment_2/KF_assignment_2024/inf_assignment_2/build/CMakeFiles/tracker.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/tracker.dir/depend
