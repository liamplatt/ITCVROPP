cmake_minimum_required(VERSION 2.8.3)
project(wallie_one)
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Debug)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED )
find_package(OpenGL REQUIRED )

find_package(catkin REQUIRED
    COMPONENTS
	pcl_ros
      roscpp
	rospy
      message_generation
      sensor_msgs
      dynamic_reconfigure
      cv_bridge
      image_transport
      camera_info_manager
driver_base
      nav_msgs
	message_runtime
	catkin
	stereo_msgs
	rosconsole
	robot_state_publisher
	urdf
	roslint
image_geometry
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
  assemble.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  darknet_ros_msgs
  nav_msgs
  geometry_msgs
  
)

generate_dynamic_reconfigure_options(
	cfg/ZedPubStereo.cfg)

catkin_package(
    CATKIN_DEPENDS
      roscpp
driver_base
      message_runtime
      sensor_msgs
      dynamic_reconfigure
      driver_base
      cv_bridge
	image_geometry
      image_transport
	image_geometry
      camera_info_manager)


include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenGL_INCLUDE_DIRS})

include_directories(${BFL_INCLUDE_DIRS})
##add_executable(camera_stereo src/camera_stereo.cpp)
##add_dependencies(camera_stereo ${PROJECT_NAME}_gencfg)
##target_link_libraries(camera_stereo ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(pcl_model_create src/pcl_model_create.cpp)
add_dependencies(pcl_model_create ${PROJECT_NAME}_gencfg)
target_link_libraries(pcl_model_create ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_INCLUDE_DIRS})

add_executable(pcl_reader_no_imagew src/pcl_reader_no_imagew.cpp)
add_dependencies(pcl_reader_no_imagew ${PROJECT_NAME}_gencfg)
target_link_libraries(pcl_reader_no_imagew ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_INCLUDE_DIRS})

add_executable(detection src/detection.cpp)
add_dependencies(detection ${PROJECT_NAME}_gencfg)
target_link_libraries(detection ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_INCLUDE_DIRS})


add_executable(imu_transformer src/imu_transformer.cpp)
add_dependencies(imu_transformer ${PROJECT_NAME}_gencfg)
target_link_libraries(imu_transformer ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_INCLUDE_DIRS})


add_executable(pcl_buffer src/pcl_buffer.cpp)
add_dependencies(pcl_buffer ${PROJECT_NAME}_gencfg)
target_link_libraries(pcl_buffer ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_INCLUDE_DIRS})
##add_executable(imu_transformer src/imu_transformer.cpp)
##add_dependencies(imu_transformer ${PROJECT_NAME}_gencfg)
##target_link_libraries(imu_transformer ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_INCLUDE_DIRS})

add_executable(correspondence_grouping_hyp src/correspondence_grouping_hyp.cpp)
add_dependencies(correspondence_grouping_hyp ${PROJECT_NAME}_gencfg)
target_link_libraries(correspondence_grouping_hyp ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_INCLUDE_DIRS} ${PCL_LIBRARIES})

add_executable(correspondence_grouping_PFH src/correspondence_grouping_PFH.cpp)
add_dependencies(correspondence_grouping_PFH ${PROJECT_NAME}_gencfg)
target_link_libraries(correspondence_grouping_PFH ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_INCLUDE_DIRS} ${PCL_LIBRARIES})

add_executable(pcl_icp src/pcl_icp.cpp)
add_dependencies(pcl_icp ${PROJECT_NAME}_gencfg)
target_link_libraries(pcl_icp ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_INCLUDE_DIRS} ${PCL_LIBRARIES})

add_executable(correspondence_grouping src/correspondence_grouping.cpp)
add_dependencies(correspondence_grouping ${PROJECT_NAME}_gencfg)
target_link_libraries(correspondence_grouping ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_INCLUDE_DIRS} ${PCL_LIBRARIES})

add_executable(correspondence_grouping_RIFT src/correspondence_grouping_RIFT.cpp)
add_dependencies(correspondence_grouping_RIFT ${PROJECT_NAME}_gencfg)
target_link_libraries(correspondence_grouping_RIFT ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_INCLUDE_DIRS} ${PCL_LIBRARIES})
#add_executable(correspondence_grouping_live src/correspondence_grouping_live.cpp)
#add_dependencies(correspondence_grouping_live ${PROJECT_NAME}_gencfg)
#target_link_libraries(correspondence_grouping_live ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_INCLUDE_DIRS} ${PCL_LIBRARIES})

add_executable(image_buffer src/image_buffer.cpp)
add_dependencies(image_buffer ${PROJECT_NAME}_gencfg)
target_link_libraries(image_buffer ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_INCLUDE_DIRS})

add_executable(image_loader src/image_loader.cpp)
add_dependencies(image_loader ${PROJECT_NAME}_gencfg)
target_link_libraries(image_loader ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_INCLUDE_DIRS})


#add_executable(point_cloud_assembler src/point_cloud_assembler.cpp)
#add_dependencies(point_cloud_assembler ${PROJECT_NAME}_gencfg)
#target_link_libraries(point_cloud_assembler ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_INCLUDE_DIRS})
#
#add_executable(tf_broadcast_keyboard src/tf_broadcast_keyboard.cpp)
#add_dependencies(tf_broadcast_keyboard ${PROJECT_NAME}_gencfg)
#target_link_libraries(tf_broadcast_keyboard ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_INCLUDE_DIRS})

add_executable(zed_sub_stereo src/zed_sub_stereo.cpp)
add_dependencies(zed_sub_stereo ${PROJECT_NAME}_gencfg)
target_link_libraries(zed_sub_stereo ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_INCLUDE_DIRS})

add_executable(pcl_processer src/pcl_processer.cpp)
add_dependencies(pcl_processer ${PROJECT_NAME}_gencfg)
target_link_libraries(pcl_processer ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_INCLUDE_DIRS})
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

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/Localizer.py
  scripts/EKFJoy.py
  scripts/pcl_transformer.py
  scripts/ZedImageSub.py
  scripts/SlamRepub.py
  scripts/TFBroadcaster.py
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
