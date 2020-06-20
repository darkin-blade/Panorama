#include "common.h"

#include "Stitch/NISwGSP_Stitching.h"

#if defined(UBUNTU)

int main(int argc, char *argv[]) {
  char app_path[64] = "../..";
  char img_path[128];// 图片路径

  // 读取图片
  MultiImages multi_images;
  Mat img_read;
  for (int i = 1; i <= 2; i ++) {
    sprintf(img_path, "%s/%d.jpg", app_path, i);
    multi_images.read_img(img_path);
  }

  // 自定义图片配对关系
  multi_images.img_pairs.emplace_back(make_pair(0, 1));
  // multi_images.img_pairs.emplace_back(make_pair(1, 2));
  // multi_images.img_pairs.emplace_back(make_pair(2, 3));
  // multi_images.img_pairs.emplace_back(make_pair(3, 4));
  multi_images.center_index = 0;// 参照图片的索引

  NISwGSP_Stitching niswgsp(multi_images);

  Mat result_1 = niswgsp.feature_match().clone();// 特征点
  Mat result_2 = niswgsp.matching_match().clone();// 匹配点
  // niswgsp.show_img("1", result_1);
  // niswgsp.show_img("2", result_2);

  niswgsp.get_solution();
  Mat result_3 = niswgsp.texture_mapping().clone();// 图像拼接
  niswgsp.show_img("3", result_3);
}

#else

extern "C" JNIEXPORT int JNICALL
Java_com_example_niswgsp_11_MainActivity_main_1test(
    JNIEnv* env,
    jobject thiz,
    jobjectArray imgPaths,
    jlong matBGR) {
  // 获取String数组长度
  jsize str_len = env->GetArrayLength(imgPaths);

  // 读取图片
  MultiImages multi_images;
  Mat img_read;
  for (int i = 0; i < str_len; i ++) {
    jstring tmp = (jstring) env->GetObjectArrayElement(imgPaths, i);
    const char *img_path = env->GetStringUTFChars(tmp, 0);
    multi_images.read_img(img_path);
    if (i != 0) {
      // 自定义图片配对关系
      multi_images.img_pairs.emplace_back(make_pair(i - 1, i));
    }
  }
  multi_images.center_index = 0;// 参照图片的索引

  NISwGSP_Stitching niswgsp(multi_images);

  // *(Mat *)matBGR = niswgsp.feature_match().clone();// 特征点
  // *(Mat *)matBGR = niswgsp.matching_match().clone();// 匹配点
  niswgsp.feature_match();// 特征点
  niswgsp.matching_match();// 匹配点

  niswgsp.get_solution();// 获取最优解
  *(Mat *)matBGR = niswgsp.texture_mapping().clone();// 图像拼接

  //    sprintf(img_path, "%s/3.jpg", app_path);
  //    imwrite(img_path, *(Mat *)matBGR);

  return 0;
}

void print_message(const char *msg) {
  __android_log_print(ANDROID_LOG_INFO, "fuck", msg);
}

#endif
