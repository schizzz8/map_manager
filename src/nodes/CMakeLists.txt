## Declare a C++ executable
add_executable(${PROJECT_NAME}_node map_manager_node.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   map_manager_library
   ${catkin_LIBRARIES}
 )

