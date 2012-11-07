rosbuild_add_executable(image_store image_store.cpp image_pose.cpp feature.cpp image_pose_data_types.cpp)
rosbuild_add_executable(image_store_test image_store_test.cpp feature.cpp)

#rosbuild_link_boost(image_store filesystem system)
#rosbuild_link_boost(default_plugin thread signals filesystem system program_options)
