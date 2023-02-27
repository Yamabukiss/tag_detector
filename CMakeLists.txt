cmake_minimum_required(VERSION 3.0.2)
project(tag_detector)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        cv_bridge
        nodelet
        pluginlib
        dynamic_reconfigure
        tf
        message_generation
        geometry_msgs
        )

find_package(OpenCV REQUIRED)

add_message_files(FILES
        tag_msg.msg
        )

generate_dynamic_reconfigure_options(
        cfg/dynamic.cfg
)

generate_messages(DEPENDENCIES
        std_msgs
        geometry_msgs
        )

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp cv_bridge dynamic_reconfigure nodelet message_runtime geometry_msgs
        LIBRARIES ${PROJECT_NAME}
)

include_directories(
        include ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/main.cpp include/tag_detector/header.h)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )