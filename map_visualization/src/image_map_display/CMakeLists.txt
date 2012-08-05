rosbuild_add_library(${PROJECT_NAME}
		image_map_display.cpp
		image_map_visual.cpp
		)

target_link_libraries(${PROJECT_NAME} ${OGRE_LIBRARIES})
