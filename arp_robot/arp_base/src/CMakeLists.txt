# Declare a cpp executable
add_executable(${PROJECT_NAME}_node arp_base arp_hardware)
#add_dependencies(${PROJECT_NAME}_node hardware_interface)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME arp_base_node PREFIX "")

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
