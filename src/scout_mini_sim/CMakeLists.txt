cmake_minimum_required(VERSION 3.8)
project(scout_mini_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes urdf worlds
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/lidar_time.py scripts/lio_to_odom.py scripts/poi_patrol.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(lio_config test/test_lio_config.py)
  ament_add_pytest_test(odometry_config test/test_odometry_config.py)
  ament_add_pytest_test(lidar_time test/test_lidar_time.py)
  ament_add_pytest_test(poi_patrol test/test_poi_patrol.py)
  ament_add_pytest_test(lio_to_odom test/test_lio_to_odom.py)
endif()

ament_package()
