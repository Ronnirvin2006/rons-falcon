cmake_minimum_required(VERSION 3.8)
project(falcon_arm_teleop)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  src/joy_teleop.py       # joystick-based teleop node
  src/slider_teleop.py    # GUI slider-based teleop node (tkinter)
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
