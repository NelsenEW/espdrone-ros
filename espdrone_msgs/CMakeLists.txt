cmake_minimum_required(VERSION 3.1)
project(espdrone_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_service_files(FILES
  AddEspdrone.srv
  Enable.srv
  GoTo.srv
  Land.srv
  RemoveEspdrone.srv
  SetGroupMask.srv
  StartTrajectory.srv
  Stop.srv
  BsValid.srv
  Takeoff.srv
  UpdateParams.srv
  Motors.srv
  UploadTrajectory.srv
  LoadLighthouseGeo.srv
)

add_message_files(FILES
  LogBlock.msg
  GenericLogData.msg
  FullState.msg
  VelocityWorld.msg
  TrajectoryPolynomialPiece.msg
  Hover.msg
  MotorStatus.msg
  TofMeasurement.msg
  Position.msg
  BatteryStatus.msg
  LighthouseGeo.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
)
