cmake_minimum_required(VERSION 3.20)
project(speedTest_dxlP1)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)


# debug mode
set(CMAKE_BUILD_TYPE RelWithDebInfo)

## Source files
#add_executable(test_dxlP1
#                test_dxlP1.cpp
#                source/robot.cpp)
#
## Path to other CMakeLists
#add_subdirectory(KMR_dxlP1)
#            
## Directories containing header files
#target_include_directories(test_dxlP1 PUBLIC header)
#target_include_directories(test_dxlP1 PUBLIC KMR_dxlP1/include)
#
## Locations of the used libraries: KMR_dxl
#target_link_directories(test_dxlP1 PRIVATE KMR_dxlP1/build)
#
## Link the used libraries: KMR_dxl
#target_link_libraries(test_dxlP1 KMR_dxlP1)


###################################
#   For test_controller           #
###################################


# add_executable(test_controller
#                   test_controller.cpp
#                   source/robot.cpp)
#
# # Path to other CMakeLists
# add_subdirectory(KMR_dxlP1)
#
# # Directories containing header files
# target_include_directories(test_controller PUBLIC header)
# target_include_directories(test_controller PUBLIC KMR_dxlP1/include)
#
# # # Locations of the used libraries: KMR_dxl
# target_link_directories(test_controller PRIVATE KMR_dxlP1/build)
#
# # Link the used libraries: KMR_dxl
# target_link_libraries(test_controller KMR_dxlP1)

###################################
#   For the controller_trajectory #
###################################

#[[
add_executable(controller_trajectory
                controller_trajectory.cpp
                source/robot.cpp)

 # Path to other CMakeLists
add_subdirectory(KMR_dxlP1)

# Directories containing header files
target_include_directories(controller_trajectory PUBLIC header)
target_include_directories(controller_trajectory PUBLIC KMR_dxlP1/include)

# Locations of the used libraries: KMR_dxl
target_link_directories(controller_trajectory PRIVATE KMR_dxlP1/build)

# Link the used libraries: KMR_dxl
target_link_libraries(controller_trajectory KMR_dxlP1)]]

#####################################
#   For phase controller_trajectory #
#####################################


# add_executable(phase_controller_trajectory
#                   phase_controller_trajectory.cpp
#                   source/robot.cpp)
#
# # Path to other CMakeLists
# add_subdirectory(KMR_dxlP1)
#
# # Directories containing header files
# target_include_directories(phase_controller_trajectory PUBLIC header)
# target_include_directories(phase_controller_trajectory PUBLIC KMR_dxlP1/include)
#
# # # Locations of the used libraries: KMR_dxl
# target_link_directories(phase_controller_trajectory PRIVATE KMR_dxlP1/build)
#
# # Link the used libraries: KMR_dxl
# target_link_libraries(phase_controller_trajectory KMR_dxlP1)

###################################
#   For leg_controller           #
###################################


add_executable(4legs_controller
                  4legs_controller.cpp
                  source/robot.cpp)

# Path to other CMakeLists
add_subdirectory(KMR_dxlP1)

# Directories containing header files
target_include_directories(4legs_controller PUBLIC header)
target_include_directories(4legs_controller PUBLIC KMR_dxlP1/include)

# # Locations of the used libraries: KMR_dxl
target_link_directories(4legs_controller PRIVATE KMR_dxlP1/build)

# Link the used libraries: KMR_dxl
target_link_libraries(4legs_controller KMR_dxlP1)
