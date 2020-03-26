cmake_minimum_required(VERSION 3.4.1)

# eigen
include_directories(/home/lynx/fuck_mount/eigen/eigen_323c052e1731)

# opencv
set(OpenCV_DIR /home/lynx/fuck_mount/opencv/install_dir/4_1_0/lib/cmake/opencv4)
find_package(OpenCV REQUIRED)
include_directories(/home/lynx/fuck_mount/opencv/build_dir/4_1_0_android/OpenCV-android-sdk/sdk/native/jni/include)

# vlfeat
set(vlfeat_dir /home/lynx/fuck_mount/vlfeat/vlfeat_android)
include_directories(${vlfeat_dir})
include_directories(${CMAKE_CURRENT_SOURCCE_DIR})
# set(vlfeat_lib "${vlfeat_dir}/build/libvl.so")

aux_source_directory(/home/lynx/fuck_mount/vlfeat/vlfeat_android/src vl_src)
aux_source_directory(/home/lynx/fuck_mount/vlfeat/vlfeat_android/vl vl_head)

set(CMAKE_CXX_COMPILER /usr/bin/g++)

add_library(
  vlfeat_lib

  SHARED

  ${vl_src}
  ${vl_head}
)

add_executable(
  entry

  common.h
  Transform.cpp
  FeatureController.cpp
  MultiImages.cpp
  APAP_Stitching.cpp
  NISwGSP_Stitching.cpp
  entry.cpp )

target_link_libraries(
  entry

  ${OpenCV_LIBS}
  # ${vlfeat_lib}
  vlfeat_lib
)
