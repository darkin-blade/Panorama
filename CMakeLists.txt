cmake_minimum_required(VERSION 3.4.1)

# eigen
include_directories(/home/lynx/fuck_mount/eigen/eigen_323c052e1731)

# opencv
set (OpenCV_DIR /home/lynx/fuck_mount/opencv/install_dir/4_1_0/lib/cmake/opencv4)
find_package(OpenCV REQUIRED)
include_directories(/home/lynx/fuck_mount/opencv/build_dir/4_1_0_android/OpenCV-android-sdk/sdk/native/jni/include)

# vlfeat
include_directories(/home/lynx/fuck_mount/vlfeat/vlfeat-0.9.21)
set(VLFEAT_LIB "/home/lynx/fuck_mount/vlfeat/vlfeat-0.9.21/bin/glnxa64/libvl.so")

set(CMAKE_CXX_COMPILER /usr/bin/g++)

add_executable(
  entry

  common.h
  Transform.h
  Transform.cpp
  MultiImages.h
  MultiImages.cpp
  APAP_Stitching.h
  APAP_Stitching.cpp
  NISwGSP_Stitching.h
  NISwGSP_Stitching.cpp
  entry.cpp )
target_link_libraries(
  entry
  ${OpenCV_LIBS}
  ${VLFEAT_LIB}
)
