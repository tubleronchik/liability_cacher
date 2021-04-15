cmake_minimum_required(VERSION 2.8.3)
project(liability_cacher)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

catkin_python_setup()

add_service_files(
    FILES
	LiabilityAddress.srv
)

generate_messages()

catkin_package()

catkin_install_python(
    PROGRAMS
        scripts/cacher_node
		scripts/web_server_node
        scripts/datalog_cacher
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    FILES
        launch/agent.launch
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

