cmake_minimum_required(VERSION 2.8.11)
project(rosmapinterface)
#set(CMAKE_PREFIX_PATH $(CMAKE_PREFIX_PATH):/home/adam/Qt/5.9.3/gcc_64/lib/cmake)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)

set(CMAKE_PREFIX_PATH /home/adam/Qt/5.9.3/gcc_64/lib/cmake)
find_package(Qt5Core REQUIRED 5.9.3)
find_package(Qt5Quick REQUIRED 5.9.3)
find_package(Qt5Widgets REQUIRED 5.9.3)
add_definitions(-std=c++11 -Wall -Wextra)

message(WARNING, "1. |${catkin_INCLUDE_DIRS}|${CATKIN_DEVEL_PREFIX}|${CMAKE_INSTALL_PREFIX}|${Qt5Core_CONSIDERED_VERSIONS}")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

add_message_files(FILES RobotInformation.msg WaypointInformation.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

QT5_ADD_RESOURCES(Resources src/res.qrc)
add_executable(rosmapinterface
            src/main.cpp
            ${Resources})
target_link_libraries(rosmapinterface ${catkin_LIBRARIES} Qt5::Core Qt5::Quick Qt5::Widgets mapwidget)
add_dependencies(rosmapinterface ${catkin_EXPORTED_TARGETS})
add_dependencies(rosmapinterface rosmapinterface_generate_messages_cpp)

#fakeRobot
add_executable(fakeRobot src/fakeRobot.cpp)
target_link_libraries(fakeRobot Qt5::Core ${catkin_LIBRARIES})
add_dependencies(fakeRobot rosmapinterface_generate_messages_cpp)


