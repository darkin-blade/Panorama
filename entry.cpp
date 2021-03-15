#include "common.h"

#include "Stitch/My_Stitching.h"

#if defined(UBUNTU)

int main(int argc, char *argv[]) {

  clock_t begin_time, end_time;
  begin_time = clock();

  char app_path[64] = "../..";
  char img_path[128];// 图片路径

  // 调用自己的方法
  
  // 读取图片, 先存入multi_images中
  MultiImages multi_images;
  for (int i = 1; i <= 12; i ++) {
    sprintf(img_path, "%s/%d.jpg", app_path, i);
    multi_images.readImg(img_path);
  }
  // 自动从前往后匹配
  for (int i = 1; i < multi_images.img_num; i ++) {
    // 自定义图片配对关系,如果配对错误会导致`type == CV_32F || type == CV_64F`错误
    // multi_images.img_pairs.emplace_back(make_pair(i - 1, i));// 顺序: 后者为先计算的图像索引, 即(目标图像, 参考图像)
    multi_images.img_pairs.emplace_back(make_pair(i, i - 1));// 顺序: 后者为先计算的图像索引, 即(目标图像, 参考图像)
  }

  My_Stitching my_stitcher(multi_images);
  Mat result = my_stitcher.getMyResult();

  end_time = clock();
  LOG("totoal time %lf", (double)(end_time - begin_time)/CLOCKS_PER_SEC);

}

#else

Mat method_openCV(vector<string>);
Mat method_my(vector<string>, vector<double>);

extern "C" JNIEXPORT int JNICALL
Java_com_example_my_1stitcher_MainActivity_main_1test(
    JNIEnv* env,
    jobject thiz,
    jobjectArray imgPaths,
    jdoubleArray imgRotations,
    jlong matBGR,
    jintArray pairFirst,
    jintArray pairSecond,
    jint mode) {// mode: 0 for my_stitcher, 1 for opencv
  total_env = env;
//  if (total_env != NULL) {
//    jclass clazz = total_env->FindClass("com.example.my_stitcher/MainActivity");
//    if (clazz != NULL) {
//        jmethodID id = total_env->GetStaticMethodID(clazz, "infoLog", "(Ljava/lang/String;)V");
//        if (id != NULL) {
//            jstring msg = total_env->NewStringUTF("fuck your mother");
//            total_env->CallStaticVoidMethod(clazz, id, msg);
//        } else {
//            assert(0);
//        }
//    } else {
//        assert(0);
//    }
//  }

  // 获取String数组长度
  jsize str_len = env->GetArrayLength(imgPaths);

  // 读取图片路径
  vector<string> img_paths;
  vector<double> img_rotations;

  jdouble *rotations = env->GetDoubleArrayElements(imgRotations, NULL);
  for (int i = 0; i < str_len; i ++) {
    jstring tmp = (jstring) env->GetObjectArrayElement(imgPaths, i);
    const char *img_path = env->GetStringUTFChars(tmp, 0);
    string tmp_path = img_path;
    img_paths.push_back(tmp_path);
    img_rotations.push_back(rotations[i]);
  }

  int result = 0;

  return result;
}

Mat method_my(vector<string> img_paths, vector<double> img_rotations) {
    Mat result;

    return result;
}

void print_message(const char *msg) {
  __android_log_print(ANDROID_LOG_INFO, "fuck", msg);
}

#endif
