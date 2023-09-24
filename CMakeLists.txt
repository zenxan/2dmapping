add_library(${PROJECT_NAME}.2dmapping
        lidar_2d_utils.cc
        occupancy_map.cc
        likelihood_field.cc
        frame.cc
        mapping_2d.cc
        submap.cc
        )

target_link_libraries(${PROJECT_NAME}.2dmapping
        ${PCL_LIBRARIES}
        ${PROJECT_NAME}.common
        )

add_executable(show_occupy show_occupy.cc)

target_link_libraries(show_occupy
        ${PROJECT_NAME}.2dmapping
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        glog gflags
        ${PCL_LIBRARIES}
        ${g2o_libs}
        )

add_executable(show_likelihood show_likelihood.cc)

target_link_libraries(show_likelihood
        ${PROJECT_NAME}.2dmapping
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        glog gflags
        ${PCL_LIBRARIES}
        ${g2o_libs}
        )

add_executable(show_2d_mapping show_2d_mapping.cc)

target_link_libraries(show_2d_mapping
        ${PROJECT_NAME}.2dmapping
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        glog gflags
        ${PCL_LIBRARIES}
        ${g2o_libs}
        )