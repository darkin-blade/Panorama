cmake_minimum_required(VERSION 3.4.1)

set(CXX_FLAGS
  `pkg-config --cflags --libs opencv4`
  -I$ENV{VLFEAT_DIR}
  -L$ENV{VLFEAT_DIR}/bin/glnxa64
  -lvl)

INCLUDE_DIRECTORIES(/home/lynx/fuck_mount/eigen/eigen_323c052e1731)

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
