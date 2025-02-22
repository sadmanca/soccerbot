cmake_minimum_required(VERSION 3.0.2)
project(soccer_fieldline_detection)

set(CMAKE_CXX_FLAGS "-std=c++17")

set(PKG_DEPS
    roscpp
    soccer_geometry
    image_transport
    cv_bridge
    sensor_msgs
    soccer_object_detection
    geometry_msgs
    tf2_ros
    )
find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})

find_package(OpenCV REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${PKG_DEPS}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(
    soccer_fieldline_detector
    src/soccer_fieldline_detector.cpp
    src/camera.cpp
    src/image.cpp
)
add_dependencies(soccer_fieldline_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(soccer_fieldline_detector
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_executable(
    ball_detector
    src/ball_detector.cpp
    src/camera.cpp
)
add_dependencies(ball_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ball_detector
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
add_executable(
        test_soccer_vision
        src/test_soccer_vision.cpp
        src/camera.cpp
)
add_dependencies(test_soccer_vision ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_soccer_vision
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_fieldline_detector test/test_soccer_fieldline_detector.cpp src/camera.cpp  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
    add_dependencies(test_fieldline_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(test_fieldline_detector
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
    )
endif ()
