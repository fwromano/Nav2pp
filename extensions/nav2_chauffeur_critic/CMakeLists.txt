cmake_minimum_required(VERSION 3.20)
project(nav2_chauffeur_critic)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(chauffeur_critic SHARED
  src/chauffeur_critic.cpp
)
target_compile_features(chauffeur_critic PUBLIC cxx_std_20)
target_compile_options(chauffeur_critic PRIVATE -O3)
target_include_directories(chauffeur_critic
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
ament_target_dependencies(chauffeur_critic
  nav2_mppi_controller
  pluginlib
  rclcpp
)
target_link_libraries(chauffeur_critic PUBLIC Eigen3::Eigen)

pluginlib_export_plugin_description_file(
  nav2_mppi_controller
  chauffeur_critic_plugin.xml
)

install(TARGETS chauffeur_critic
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY config demos
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_chauffeur_math test/test_chauffeur_math.cpp)
endif()

ament_export_dependencies(nav2_mppi_controller pluginlib rclcpp Eigen3)
ament_package()
