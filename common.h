#if !defined(COMMON_H)
#define COMMON_H

#include <assert.h>
#include <stdarg.h>
#include <unistd.h>
#include <cmath>
#include <iostream>
#include <queue>
#include <set>
#include <string>
#include <time.h>
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
#include <opencv2/calib3d/calib3d_c.h> /* CV_RANSAC */
#include <opencv2/calib3d.hpp> /* findHomography */
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/stitching/detail/autocalib.hpp>
#include <opencv2/stitching/detail/camera.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>
#include <opencv2/stitching.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/ximgproc/fast_line_detector.hpp>
/* stitch */
#include <opencv2/opencv_modules.hpp>
#include <opencv2/stitching/detail/blenders.hpp>
#include <opencv2/stitching/detail/timelapsers.hpp>
#include <opencv2/stitching/detail/camera.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <opencv2/stitching/detail/matchers.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>
#include <opencv2/stitching/detail/warpers.hpp>
#include <opencv2/stitching/warpers.hpp>

#define LOG(format, ...) \
  print_message("[%s, %d] " format, __func__, __LINE__, ## __VA_ARGS__)

extern "C" {
#include <vl/generic.h>
#include <vl/sift.h>
}

using namespace std;

using namespace Eigen;

using namespace cv;
using namespace cv::detail;
using namespace cv::xfeatures2d;

/************************************** 自定义 ********************************************/

void print_message(const char *fmt, ...);
void set_progress(int progress, int mode);
void show_img(const char *window_name, Mat img);

/*****************************************************************************************/

/** MeshGrid.cpp **/
const int GRID_VERTEX_SIZE = 4;

/*****************************************************************************************/

/*** data setting ***/
const int GRID_SIZE = 40;
const int DOWN_SAMPLE_IMAGE_SIZE = 800 * 600;

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
const    int SIFT_MINIMUM_OCTAVE_INDEX = 0;// o_min
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

/*** sparse linear system ***/
const double STRONG_CONSTRAINT = 1e4;

/*** bundle adjustment ***/
const int CRITERIA_MAX_COUNT = 1000;
const double CRITERIA_EPSILON = DBL_EPSILON;

/*** 2D Method ***/
const double TOLERANT_ANGLE = 1.5;

/*** 3D Method ***/
const double LAMBDA_GAMMA = 10;

/************************/
/************************/
/************************/

/* AutoStitch */
enum  AUTO_STITCH_WAVE_CORRECTS { WAVE_X = 0, WAVE_H, WAVE_V };
const AUTO_STITCH_WAVE_CORRECTS   WAVE_CORRECT = WAVE_H;

/************************/
/************************/
/************************/

/* draw image */
#if defined(UBUNTU)
const int CIRCLE_SIZE = 3;
const int LINE_SIZE   = 1;
#else
const int CIRCLE_SIZE = 10;
const int LINE_SIZE   = 3;
#endif

#endif
