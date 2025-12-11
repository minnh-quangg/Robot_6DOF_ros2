cmake_minimum_required(VERSION 3.5)

project(robot_6dof)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes textures config launch rviz
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
 
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()



