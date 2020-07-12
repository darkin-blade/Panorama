cmake_minimum_required(VERSION 3.4.1)

set(ubuntu true)

# 编译选项
set(CMAKE_CXX_FLAGS "-g")

# source files
aux_source_directory(./Feature feature_src)
aux_source_directory(./Mesh mesh_src)
aux_source_directory(./Stitch stitch_src)
aux_source_directory(./Util util_src)
set(niswgsp_source
    ${feature_src}
    ${mesh_src}
    ${stitch_src}
    ${util_src}
    common.h
    common.cpp
    entry.cpp )

# eigen
include_directories(/home/lynx/fuck_mount/eigen)

# opencv
if(ubuntu)
  set(OpenCV_DIR /home/lynx/fuck_mount/opencv/install_dir/4_1_0/lib/cmake/opencv4)
  # include_directories(/home/lynx/fuck_mount/opencv/build_dir/4_1_0_android/OpenCV-android-sdk/sdk/native/jni/include)
  find_package(OpenCV REQUIRED)
else()
  set(sdk_dir /home/lynx/fuck_mount/opencv/build_dir/4_1_0_android)
  set(OpenCV_DIR ${sdk_dir}/OpenCV-android-sdk/sdk/native/jni)
  include_directories(${sdk_dir}/OpenCV-android-sdk/sdk/native/jni/include)
  find_package(OpenCV REQUIRED)
endif()

# vlfeat
set(vlfeat_dir /home/lynx/fuck_mount/vlfeat_android)
include_directories(${vlfeat_dir})
# include_directories(${CMAKE_CURRENT_SOURCCE_DIR})
# set(vlfeat_lib "${vlfeat_dir}/build/libvl.so")
aux_source_directory(/home/lynx/fuck_mount/vlfeat_android/src vl_src)
aux_source_directory(/home/lynx/fuck_mount/vlfeat_android/vl vl_head)
add_library(
  vlfeat_lib

  SHARED

  ${vl_src}
  ${vl_head}
)

if(ubuntu)
  set(CMAKE_CXX_COMPILER /usr/bin/g++)
endif()

# 添加目标文件/目标库
if(ubuntu)
  add_executable(
    entry

    ${niswgsp_source})
else()
  add_library(
          entry

          SHARED

          ${niswgsp_source} )
endif()

# 连接库文件
if(ubuntu)
  target_link_libraries(
    entry

    ${OpenCV_LIBS}
    # ${vlfeat_lib}
    vlfeat_lib
  )
else()
  find_library( # Sets the name of the path variable.
                log-lib
                log )

  target_link_libraries( # Specifies the target library.
          entry

          ${OpenCV_LIBS}
          # ${vlfeat_lib}
          vlfeat_lib

          # TODO 解决 AndroidBitmap 报错, 使本机代码安全地访问Java对象的像素缓冲区的位图
          jnigraphics
          ${log-lib} )
endif()