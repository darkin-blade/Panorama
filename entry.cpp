#include "common.h"

#include "Stitch/NISwGSP_Stitching.h"

#if defined(UBUNTU)

int main() {
  char app_path[64] = "../..";
  char img_path[128];// 图片路径

  // 读取图片
  MultiImages multiImages;
  Mat img_read;
  sprintf(img_path, "%s/1.jpg", app_path);
  multiImages.read_img(img_path);
  sprintf(img_path, "%s/2.jpg", app_path);
  multiImages.read_img(img_path);

  NISwGSP_Stitching niswgsp(multiImages);
  Mat result_1 = niswgsp.feature_match().clone();// 特征点
  Mat result_2 = niswgsp.matching_match().clone();// 匹配点
  Mat result_3 = niswgsp.texture_mapping().clone();// 图像拼接

  // 显示图片
  niswgsp.show_img("1", result_1);
  niswgsp.show_img("2", result_2);
  niswgsp.show_img("3", result_3);
}

#else

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
  multiImages.read_img(img_path);
  sprintf(img_path, "%s/2.jpg", app_path);
  multiImages.read_img(img_path);

  NISwGSP_Stitching niswgsp(multiImages);
  *(Mat *)matBGR = niswgsp.feature_match().clone();// 特征点
//  niswgsp.feature_match();// 特征点
//  *(Mat *)matBGR = niswgsp.matching_match().clone();// 匹配点

  //    sprintf(img_path, "%s/3.jpg", app_path);
  //    imwrite(img_path, *(Mat *)matBGR);

  return 0;
}

#endif
