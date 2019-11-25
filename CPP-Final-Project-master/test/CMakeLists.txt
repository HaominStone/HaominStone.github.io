SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-Wall -std=c++11")
set(CMAKE_BUILD_TYPE Debug)

add_executable(test_main main.cpp)

target_link_libraries(test_main lp_lib)

find_package(Boost REQUIRED COMPONENTS
filesystem
)
include_directories(${Boost_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
target_link_libraries(test_main ${OpenCV_LIBS})


find_package(PkgConfig REQUIRED)
target_link_libraries( test_main ${PKG_OPENCV_LDFLAGS} )

find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
target_link_libraries(test_main ${Boost_LIBRARIES} -lgtsam -ltbb)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
target_link_libraries(test_main ${CERES_LIBRARIES})
