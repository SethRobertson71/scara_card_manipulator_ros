cmake_minimum_required(VERSION 3.8)
project(m1pro_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "msg/ToolVectorActual.msg"
  "srv/EnableRobot.srv"  "srv/DisableRobot.srv"  "srv/ClearError.srv"
  "srv/ResetRobot.srv"   "srv/SpeedFactor.srv"   "srv/User.srv"
  "srv/Tool.srv"         "srv/RobotMode.srv"     "srv/PayLoad.srv"
  "srv/DO.srv"           "srv/DOExecute.srv"     "srv/ToolDO.srv"
  "srv/ToolDOExecute.srv" "srv/AccJ.srv"         "srv/AccL.srv"
  "srv/SpeedJ.srv"       "srv/SpeedL.srv"        "srv/Arch.srv"
  "srv/CP.srv"           "srv/LimZ.srv"          "srv/SetArmOrientation.srv"
  "srv/RunScript.srv"    "srv/StopScript.srv"    "srv/PauseScript.srv"
  "srv/ContinueScript.srv" "srv/GetHoldRegs.srv" "srv/SetHoldRegs.srv"
  "srv/SetObstacleAvoid.srv" "srv/SetCollisionLevel.srv" "srv/EmergencyStop.srv"
  "srv/MovJ.srv"         "srv/MovL.srv"          "srv/JointMovJ.srv"
  "srv/Jump.srv"         "srv/RelMovJ.srv"       "srv/RelMovL.srv"
  "srv/MovLIO.srv"       "srv/MovJIO.srv"        "srv/Arc.srv"
  "srv/Circle.srv"       "srv/RelMovJUser.srv"   "srv/RelMovLUser.srv"
  "srv/RelJointMovJ.srv" "srv/MovJExt.srv"       "srv/Sync.srv"
  "srv/SyncAll.srv"      "srv/MoveJog.srv"       "srv/StopmoveJog.srv"
  "srv/Wait.srv"         "srv/Continues.srv"     "srv/Pause.srv"
  "srv/ModbusCreate.srv" "srv/ModbusClose.srv"   "srv/GetInBits.srv"
  "srv/GetInRegs.srv"    "srv/GetCoils.srv"      "srv/SetCoils.srv"
  "srv/DI.srv"           "srv/ToolDI.srv"        "srv/DOGroup.srv"
  "srv/BrakeControl.srv" "srv/StartDrag.srv"     "srv/StopDrag.srv"
  "srv/LoadSwitch.srv"   "srv/GetAngle.srv"      "srv/GetPose.srv"
  "srv/GetErrorID.srv"   "srv/SetPayload.srv"    "srv/PositiveSolution.srv"
  "srv/InverseSolution.srv"
  DEPENDENCIES std_msgs
)

add_executable(m1pro_driver
  src/main.cpp
  src/robot.cpp
  src/tcp_socket.cpp
)

target_include_directories(m1pro_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(m1pro_driver
  rclcpp rclcpp_action std_msgs sensor_msgs control_msgs trajectory_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(m1pro_driver "${cpp_typesupport_target}")

install(TARGETS m1pro_driver DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
