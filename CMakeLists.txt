# CMake>=3.8 supports CUDA C++ as intrinsically supported language
cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(sgm_gpu_ros CUDA CXX)
set(AIRSIM_ROOT /home/seungwooubuntu/AirSim)
add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wpedantic -Wstrict-null-sentinel -Wno-unused")
#set(CXX_EXP_LIB "-nostdinc++ -I/usr/include/c++/8 -I/usr/include/x86_64-linux-gnu/c++/8 -nodefaultlibs
#-l/usr/lib/x86_64-linux-gnu/libc++.so -l/usr/lib/x86_64-linux-gnu/libc++abi.so
#-lm -lc -lgcc_s -lgcc
#-lstdc++fs -fmax-errors=10")

set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.3.0/include")
set(RPC_LIB rpc) # name of .a file with lib prefix
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

find_package(Boost REQUIRED)

find_package(OpenCV 4.2 REQUIRED)


#set(CMAKE_BUILD_TYPE Debug)

set(CUDA_NVCC_FLAGS
        ${CUDA_NVCC_FLAGS};
        -O3
        -gencode=arch=compute_30,code=sm_30
        -gencode=arch=compute_35,code=sm_35
        -gencode=arch=compute_50,code=sm_50
        -gencode=arch=compute_52,code=sm_52
        -gencode=arch=compute_61,code=sm_61
        )
find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        image_geometry
        mavros_msgs
        message_generation
        nav_msgs
        # nodelet
        roscpp
        rospy
        sensor_msgs
        std_msgs
        geographic_msgs
        geometry_msgs
        std_srvs
        tf2
        tf2_ros
        tf2_sensor_msgs
        tf2_geometry_msgs
        )

catkin_package(
        CATKIN_DEPENDS cv_bridge image_geometry roscpp sensor_msgs stereo_msgs
        DEPENDS OpenCV
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${AIRSIM_ROOT}/AirLib/deps/eigen3
        ${AIRSIM_ROOT}/AirLib/include
        ${RPC_LIB_INCLUDES}
        ${AIRSIM_ROOT}/MavLinkCom/include
        ${AIRSIM_ROOT}/MavLinkCom/common_utils
        ${OpenCV_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

# Build lib${PROJECT_NAME}
add_library(${PROJECT_NAME}
        src/sgm_gpu.cu
        src/costs.cu
        src/hamming_cost.cu
        src/left_right_consistency.cu
        src/median_filter.cu
        )
add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS} AirLib
        )


# Build ${PROJECT_NAME}_node
set(PROJECT_NAME_ "sgm_gpu")
add_executable(${PROJECT_NAME}_node
        src/${PROJECT_NAME_}_node.cpp
        src/${PROJECT_NAME_}_node_main.cpp
        )
add_dependencies(${PROJECT_NAME}_node
        ${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(${PROJECT_NAME}_node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

# Build $airsim_sgm_node
add_executable(
        airsim_sgm_node
        src/airsim_sgm_node.cpp
)
add_dependencies(airsim_sgm_node
        ${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(airsim_sgm_node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        AirLib
        )

# Install lib${PROJECT_NAME}
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )
install(DIRECTORY include/${PROJECT_NAME_}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )

# Install ${PROJECT_NAME}_node
install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Install launch, bag and document
install(FILES test_input.bag README.md
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

