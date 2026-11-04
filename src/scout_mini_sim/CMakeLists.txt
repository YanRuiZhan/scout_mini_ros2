cmake_minimum_required(VERSION 3.8)
project(scout_mini_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes urdf worlds
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/lidar_time.py scripts/lio_odom_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_model.py)
    ament_add_pytest_test(model_contract test/test_model.py)
  endif()
  ament_add_pytest_test(lio_config test/test_lio_config.py)
  ament_add_pytest_test(lio_odom_bridge test/test_lio_odom_bridge.py
    ENV ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1)
endif()

ament_package()
