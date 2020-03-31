#if !defined(COMMON_H)
#define COMMON_H

#include <assert.h>
#include <cmath>
#include <set>
#include <string>
#include <vector>

#define using_opencv
#define UBUNTU
#if !defined(UBUNTU)

#include <jni.h>
#include <android/log.h>
#include <android/bitmap.h>

#endif

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/IterativeLinearSolvers>
#include <opencv2/calib3d/calib3d_c.h> /* CV_RANSAC */
#include <opencv2/calib3d.hpp> /* findHomography */
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#if defined(UBUNTU)

#define LOG(format, ...) \
  printf("\033[1;32m[%s, %d]" format "\33[0m\n", __func__, __LINE__, ## __VA_ARGS__)
#else
#define LOG(format, ...) __android_log_print(ANDROID_LOG_INFO, "fuck", "[%s, %d] " format, __func__, __LINE__, ## __VA_ARGS__)

#endif

extern "C" {
#include <vl/generic.h>
#include <vl/sift.h>
}

/*** APAP ***/
const double APAP_GAMMA = 0.0015;
const double APAP_SIGMA = 8.5;

/*** matching method ***/
const double FEATURE_RATIO_TEST_THRESHOLD = 1.5;// atof("15e-1");

/*** homography based ***/
const double GLOBAL_HOMOGRAPHY_MAX_INLIERS_DIST   = 5.;
const double  LOCAL_HOMOGRAPHY_MAX_INLIERS_DIST   = 3.;
const    int  LOCAL_HOMOGRAPHY_MIN_FEATURES_COUNT = 40;

/* type */
const int DIMENSION_2D = 2;
const int HOMOGRAPHY_VARIABLES_COUNT = 9;

/*** vlfeat sift ***/
const    int SIFT_LEVEL_COUNT          = 3;// nlevels
#if defined(UBUNTU)
const    int SIFT_MINIMUM_OCTAVE_INDEX = 0;// o_min
#else
const    int SIFT_MINIMUM_OCTAVE_INDEX = 2;// o_min
#endif
const double SIFT_PEAK_THRESH = 0.;
const double SIFT_EDGE_THRESH = 10.;

/*** init feature ***/
const double INLIER_TOLERANT_STD_DISTANCE = 4.25; /* mean + 4.25 * std */

/*** sRANSAC ***/
const double GLOBAL_TRUE_PROBABILITY = 0.225;
const double LOCAL_TRUE_PROBABILITY = 0.2;
const double OPENCV_DEFAULT_CONFIDENCE = 0.995;
const int HOMOGRAPHY_MODEL_MIN_POINTS = 4;
// const int GLOBAL_MAX_ITERATION = log(1 - OPENCV_DEFAULT_CONFIDENCE) / log(1 - pow(GLOBAL_TRUE_PROBABILITY, HOMOGRAPHY_MODEL_MIN_POINTS));
const int GLOBAL_MAX_ITERATION = 200;// 2000+
// const int LOCAL_MAX_ITERATION = log(1 - OPENCV_DEFAULT_CONFIDENCE) / log(1 - pow(LOCAL_TRUE_PROBABILITY, HOMOGRAPHY_MODEL_MIN_POINTS));
const int LOCAL_MAX_ITERATION = 1000;// 16000+

/* draw image */
#if defined(UBUNTU)
const int CIRCLE_SIZE = 3;
const int LINE_SIZE   = 1;
#else
const int CIRCLE_SIZE = 10;
const int LINE_SIZE   = 3;
#endif

#endif
