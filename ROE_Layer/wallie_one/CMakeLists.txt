cmake_minimum_required(VERSION 2.8.3)
project(wallie_one)

set(CMAKE_BUILD_TYPE Debug)
find_package(Eigen3)
find_package(octomap REQUIRED)
find_package(ompl)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

find_package(catkin REQUIRED
    COMPONENTS
	pcl_ros
      roscpp
	rospy
	ompl
      message_generation
      sensor_msgs
	create_msgs
      dynamic_reconfigure
      driver_base
      cv_bridge
      image_transport
      camera_info_manager
      nav_msgs
	zed_wrapper
	message_runtime
	catkin
	stereo_msgs
	rosconsole
	robot_state_publisher
	urdf
	roslint
	diagnostic_updater
	dynamic_reconfigure
	nodelet)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  stereo_image.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  FeatureDetection.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  nav_msgs
  create_msgs
  geometry_msgs
  
)

generate_dynamic_reconfigure_options(
	cfg/ZedPubStereo.cfg)

catkin_package(
    CATKIN_DEPENDS
      roscpp
      message_runtime
      sensor_msgs
      dynamic_reconfigure
      driver_base
      cv_bridge
      image_transport
	image_geometry
      camera_info_manager)


include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

include_directories(${BFL_INCLUDE_DIRS})
##add_executable(camera_stereo src/camera_stereo.cpp)
##add_dependencies(camera_stereo ${PROJECT_NAME}_gencfg)
##target_link_libraries(camera_stereo ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(pcl_reader src/pcl_reader.cpp)
add_dependencies(pcl_reader ${PROJECT_NAME}_gencfg)
target_link_libraries(pcl_reader ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_INCLUDE_DIRS})

add_executable(pcl_reader_no_imagew src/pcl_reader_no_imagew.cpp)
add_dependencies(pcl_reader_no_imagew ${PROJECT_NAME}_gencfg)
target_link_libraries(pcl_reader_no_imagew ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_INCLUDE_DIRS})

add_executable(imu_transformer src/imu_transformer.cpp)
add_dependencies(imu_transformer ${PROJECT_NAME}_gencfg)
target_link_libraries(imu_transformer ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_INCLUDE_DIRS})

add_executable(zed_sub_stereo src/zed_sub_stereo.cpp)
add_dependencies(zed_sub_stereo ${PROJECT_NAME}_gencfg)
target_link_libraries(zed_sub_stereo ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_INCLUDE_DIRS})
#link_directories(/usr/lib)
#add_executable(robot_precision_ekf 
#                       src/robot_precision_ekf.cpp 
#                       src/nonlinearanalyticconditionalgaussian_robot.cpp 
#                       src/nonlinearanalyticconditionalgaussian_robot_verr.cpp 
#                       src/nonlinearanalyticconditionalgaussian_3state.cpp 
#                       src/nonlinearanalyticconditionalgaussian_gpsmeasurement.cpp
#                       src/robot_precision_ekf_node.cpp)
#target_link_libraries(robot_precision_ekf ${fcl_LIBRARIES}  ${OCTOMAP_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBS} ${BFL_LIBRARY_DIRS})
#target_include_directories( robot_precision_ekf  PUBLIC include/robot_precision_ekf/)
add_executable(Slam src/Slam.cpp)
target_link_libraries(Slam ${fcl_LIBRARIES}  ${OCTOMAP_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBS})

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/Localizer.py
  scripts/EKFJoy.py
	scripts/pcl_transformer.py
  scripts/ZedServer.py
  scripts/SlamRepub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark cpp header files for installation
install(DIRECTORY include/beginner_tutorials/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
install(DIRECTORY include/robot_precision_ekf/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
launch/launcher.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS zed_sub_stereo ##camera_stereo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

##foreach(dir bag config launch)
  ##  install(DIRECTORY ${dir}/
    ##    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
##endforeach(dir)
