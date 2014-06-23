cmake_minimum_required(VERSION 2.8.3)

project(realtime_urdf_filter)
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Debug)


#set the default path for built executables to the "bin" directory
#set the default path for built libraries to the "lib" directory

# for future reference. you can ignore this.
#cmake_policy(SET CMP0012 NEW)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)

find_package(catkin REQUIRED COMPONENTS roscpp roslib image_transport urdf tf resource_retriever sensor_msgs
            cv_bridge nodelet)

find_package(OpenNI)
find_package(OpenCV REQUIRED)
find_package(OpenGL)
find_library(freeglut_LIBRARY glut /usr/lib)




find_path( freeglut_INCLUDE_DIR GL/freeglut.h
  /usr/include/GL
  /usr/openwin/share/include
  /usr/openwin/include
  /opt/graphics/OpenGL/include
  /opt/graphics/OpenGL/contrib/libglut
  )
include_directories(include
                    ${freeglut_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${OPENGL_INCLUDE_DIR}
                    ${Boost_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS include
    )


add_library (FBO src/FrameBufferObject.cpp)
target_link_libraries (FBO GLEW)

add_library (shaderwrapper src/shader_wrapper.cpp)

add_library (urdf_filter 
  src/urdf_filter.cpp
  src/urdf_renderer.cpp 
  src/renderable.cpp)
target_link_libraries (urdf_filter
  ${catkin_LIBRARIES}  
  ${OPENGL_LIBRARIES}
  ${freeglut_LIBRARY} 
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  FBO
  shaderwrapper
  assimp)

add_executable (urdf_filtered_tracker src/urdf_filtered_tracker.cpp)
target_link_libraries (urdf_filtered_tracker ${catkin_LIBRARIES} urdf_filter OpenNI)

add_executable (realtime_urdf_filter src/realtime_urdf_filter.cpp)
target_link_libraries (realtime_urdf_filter ${catkin_LIBRARIES} urdf_filter)

add_library (realtime_urdf_filter_nodelet src/realtime_urdf_filter_nodelet.cpp)
target_link_libraries (realtime_urdf_filter_nodelet ${catkin_LIBRARIES} urdf_filter)
