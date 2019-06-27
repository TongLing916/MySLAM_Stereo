cmake_minimum_required(VERSION 2.8)
project(myslam)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++11 -O3 -fopenmp -pthread")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

############### dependencies ######################
# Eigen
include_directories("/usr/include/eigen3")

# OpenCV
find_package(OpenCV 3.1 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# Sophus
find_package(sophus REQUIRED)
include_directories(${sophus_INCLUDE_DIRS})

# G2O
# find_package(G2O REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/g2o/)

# glog
#find_package(Glog REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/glog/)

# gtest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# gflags
# find_package(GFlags REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/gflags/)

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${sophus_LIBRARIES}
        ${Pangolin_LIBRARIES} GL GLU GLEW glut
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_core.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_stuff.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_types_sba.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_solver_csparse.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib/libg2o_csparse_extension.so
        ${GTEST_BOTH_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/thirdparty/glog/build/libglog.a
        ${PROJECT_SOURCE_DIR}/thirdparty/gflags/build/lib/libgflags.a
        pthread
        ${CSPARSE_LIBRARY}
        )

enable_testing()
############### source and test ######################
include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(app)
