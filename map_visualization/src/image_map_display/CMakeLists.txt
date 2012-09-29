rosbuild_add_library(${PROJECT_NAME}
		image_map_display.cpp
		image_map_visual.cpp
		image_map_info.cpp
		mobots_info.cpp
		)

target_link_libraries(${PROJECT_NAME} ${OGRE_LIBRARIES})

rosbuild_add_executable(image_map_test image_map_test.cpp)
