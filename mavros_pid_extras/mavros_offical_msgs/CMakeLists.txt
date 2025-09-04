cmake_minimum_required(VERSION 3.10)
project(mavros_offical_msgs)

find_package(catkin REQUIRED COMPONENTS geographic_msgs geometry_msgs message_generation sensor_msgs std_msgs)

# add_message_files(DIRECTORY msg FILES
#     )

# add_service_files(DIRECTORY srv FILES
#     )

# generate_messages(DEPENDENCIES geographic_msgs geometry_msgs sensor_msgs std_msgs)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS geographic_msgs geometry_msgs sensor_msgs std_msgs message_runtime
)

install(
    DIRECTORY include/${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)