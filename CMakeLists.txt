cmake_minimum_required(VERSION 2.8.6)
project(realtime_urdf_filter)

# for future reference. you can ignore this.
#cmake_policy(SET CMP0012 NEW)

find_package(catkin REQUIRED COMPONENTS 
  urdf
  tf
  resource_retriever
  sensor_msgs
  cv_bridge
  nodelet
  image_transport
  urdfdom_headers
  moveit_ros_perception
  )

catkin_package(
  INCLUDE_DIRS include)

include_directories(include)

find_package(Boost REQUIRED COMPONENTS signals)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)
find_package(OpenNI)

find_package(OpenCV REQUIRED)

find_package(OpenGL)
find_library(freeglut_LIBRARY glut /usr/lib)

include_directories(${OPENGL_INCLUDE_DIR})
find_path( freeglut_INCLUDE_DIR GL/freeglut.h
  /usr/include/GL
  /usr/openwin/share/include
  /usr/openwin/include
  /opt/graphics/OpenGL/include
  /opt/graphics/OpenGL/contrib/libglut
  )
include_directories(${freeglut_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})

add_library (FBO src/FrameBufferObject.cpp )
target_link_libraries (FBO GLEW ${catkin_LIBRARIES})

add_library (shaderwrapper src/shader_wrapper.cpp)
target_link_libraries(shaderwrapper ${catkin_LIBRARIES})

add_library (urdf_filter 
  src/urdf_filter.cpp
  src/urdf_renderer.cpp 
  src/renderable.cpp)
target_link_libraries (urdf_filter
  ${catkin_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${freeglut_LIBRARY} 
  ${OpenCV_LIBS}
  FBO
  shaderwrapper
  assimp
  ${Boost_LIBRARIES})

add_executable (urdf_filtered_tracker src/urdf_filtered_tracker.cpp)
target_link_libraries (urdf_filtered_tracker urdf_filter OpenNI)

add_executable (realtime_urdf_filter src/realtime_urdf_filter.cpp)
target_link_libraries (realtime_urdf_filter urdf_filter)

add_library (realtime_urdf_filter_nodelet src/realtime_urdf_filter_nodelet.cpp)
target_link_libraries (realtime_urdf_filter_nodelet urdf_filter)

# install targets
install(DIRECTORY include/ DESTINATION include)

# moveit plugin
find_package(moveit_ros_perception)

if(moveit_ros_perception_FOUND)
  find_package(Eigen REQUIRED)
  set(MOVEIT_LIB_NAME realtime_urdf_filter_moveit_octomap_updater)

  include_directories(${Eigen_INCLUDE_DIRS} ${moveit_ros_perception_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

  add_library(${MOVEIT_LIB_NAME}_core
    src/moveit_octomap_updater.cpp
    )

  target_link_libraries(${MOVEIT_LIB_NAME}_core urdf_filter ${moveit_ros_perception_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

  add_library(${MOVEIT_LIB_NAME} src/moveit_octomap_updater_plugin.cpp)
  target_link_libraries(${MOVEIT_LIB_NAME} ${MOVEIT_LIB_NAME}_core ${catkin_LIBRARIES} ${Boost_LIBRARIES})

  install(TARGETS ${MOVEIT_LIB_NAME}_core ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
else()
  message(STATUS "moveit not found; not building moveit plugin")
endif()
