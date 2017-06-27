add_library(map_manager_library SHARED
  map_manager.h map_manager.cpp
)

target_link_libraries(map_manager_library 
  ${catkin_LIBRARIES}
)
