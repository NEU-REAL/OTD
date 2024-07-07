add_executable(run_otd_KITTI run_otd_KITTI.cpp)
    
add_dependencies(run_otd_KITTI livox_ros_driver_gencpp)

target_link_libraries(run_otd_KITTI
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${PYTHON_LIBRARIES}
        tbb
        # glog
        yaml-cpp
        OpenMP::OpenMP_CXX
        gflags
        )
target_include_directories(run_otd_KITTI PRIVATE ${PYTHON_INCLUDE_DIRS})


add_executable(run_otd_online run_otd_online.cpp)
add_dependencies(run_otd_online livox_ros_driver_gencpp)

target_link_libraries(run_otd_online
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${PYTHON_LIBRARIES}
        tbb
        # glog
        yaml-cpp
        OpenMP::OpenMP_CXX
        gflags
        )
target_include_directories(run_otd_online PRIVATE ${PYTHON_INCLUDE_DIRS})

        