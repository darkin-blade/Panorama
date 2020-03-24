#define COMMON_H

#include <assert.h>
#include <string>
#include <vector>

#define UBUNTU
#if !defined(UBUNTU)
#include <jni.h>
#include <android/log.h>
#include <android/bitmap.h>
#endif

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/IterativeLinearSolvers>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#if defined(UBUNTU)
#define LOG(format, ...) \
  printf("\033[1;32m[%s, %d]" format "\33[0m\n", __func__, __LINE__, ## __VA_ARGS__)
#else
#define LOG(format, ...) __android_log_print(ANDROID_LOG_INFO, "fuck", "[%s, %d] " format, __func__, __LINE__, ## __VA_ARGS__)
#endif

/*** APAP ***/
const double APAP_GAMMA = 0.0015;
const double APAP_SIGMA = 8.5;

/* type */
const int DIMENSION_2D = 2;
const int HOMOGRAPHY_VARIABLES_COUNT = 9;
