#if !defined(COMMON_H)
#include "common.h"
#endif

#include "NISwGSP_Stitching.h"
#include "../../../../../../opencv/build_dir/4_1_0_android/OpenCV-android-sdk/sdk/native/jni/include/opencv2/core/mat.hpp"

using namespace std;

using namespace cv;
using namespace cv::detail;
using namespace cv::xfeatures2d;

extern "C" JNIEXPORT int JNICALL
Java_com_example_niswgsp_11_MainActivity_main_1test(
    JNIEnv* env,
    jobject thiz,
    jstring appPath,
    jlong matBGR);


extern "C" JNIEXPORT int JNICALL
Java_com_example_niswgsp_11_MainActivity_main_1test(
    JNIEnv* env,
    jobject thiz,
    jstring appPath,
    jlong matBGR) {
  const char *app_path = env->GetStringUTFChars(appPath, 0);// app/files
  char img_path[128];// 图片路径

  // 读取图片
  MultiImages multiImages;
  Mat img_read;
  sprintf(img_path, "%s/1.jpg", app_path);
  img_read = imread(img_path);
  multiImages.imgs.push_back(img_read);
  sprintf(img_path, "%s/2.jpg", app_path);
  img_read = imread(img_path);
  multiImages.imgs.push_back(img_read);

  NISwGSP_Stitching niswgsp(multiImages);
//  *(Mat *)matBGR = niswgsp.draw_matches().clone();// 匹配点
  niswgsp.draw_matches();// 特征点
  *(Mat *)matBGR = niswgsp.get_matching_pts().clone();// 匹配点

  //    sprintf(img_path, "%s/3.jpg", app_path);
  //    imwrite(img_path, *(Mat *)matBGR);

  return 0;
}
