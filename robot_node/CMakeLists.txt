cmake_minimum_required(VERSION 2.8.3)
project(robot_node)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  robot_comm
  matvec
)
find_package(Protobuf REQUIRED)

catkin_package()

add_subdirectory(ABBInterpreter)

include_directories(src include ABBInterpreter/include ${catkin_INCLUDE_DIRS})

add_executable(robot_node src/robot_node.cpp
                        src/PracticalSocket/PracticalSocket.cpp
                        src/tinyxml/tinystr.cpp
                        src/tinyxml/tinyxml.cpp
                        src/tinyxml/tinyxmlerror.cpp
                        src/tinyxml/tinyxmlparser.cpp
                        src/EGMControl/EGMHelper.cpp
                        src/EGMControl/RobotHelper.cpp
                        src/EGMControl/ROSHelper.cpp
                        src/EGMControl/egm.pb.cc)

target_link_libraries(robot_node ${catkin_LIBRARIES} ${PROTOBUF_LIBRARY} robot_comm ABBInterpreter)
