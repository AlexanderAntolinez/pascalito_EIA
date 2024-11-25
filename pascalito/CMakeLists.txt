cmake_minimum_required(VERSION 3.8)
project(pascalito)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)  # Añadido para Python
find_package(rclpy REQUIRED)              # Añadido para Python
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Configuración de dependencias para C++
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Agregar ejecutable C++
add_executable(odom src/pascalito_odom.cpp)
ament_target_dependencies(odom ${THIS_PACKAGE_INCLUDE_DEPENDS})
install(TARGETS  odom DESTINATION lib/${PROJECT_NAME})

# Instalar scripts de Python
install(
  PROGRAMS
    src/cinematicaDirecta_node.py
    src/cinematicaInversa_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Instalar otros directorios
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch  
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config  
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
