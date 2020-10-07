#include "common.h"

#include "Stitch/NISwGSP_Stitching.h"
#include "Stitch/OpenCV_Stitching.h"

#if defined(UBUNTU)

int main(int argc, char *argv[]) {

  clock_t begin_time, end_time;
  begin_time = clock();

  char app_path[64] = "../..";
  char img_path[128];// 图片路径

  if (true) {
    // 读取图片
    MultiImages multi_images;
    for (int i = 1; i <= 2; i ++) {
        sprintf(img_path, "%s/%d.jpg", app_path, i);
        multi_images.read_img(img_path);
    }
    // 自动从前往后匹配
    for (int i = 1; i < multi_images.img_num; i ++) {
      // 自定义图片配对关系,如果配对错误会导致`type == CV_32F || type == CV_64F`错误
      multi_images.img_pairs.emplace_back(make_pair(i - 1, i));
    }
    // 手动匹配
    // multi_images.img_pairs.emplace_back(make_pair(0, 1));

    NISwGSP_Stitching niswgsp(multi_images);

    Mat result_1 = niswgsp.feature_match().clone();// 特征点
    Mat result_2 = niswgsp.matching_match().clone();// 匹配点
    // show_img("1", result_1);
    // show_img("2", result_2);

    niswgsp.get_mesh();
    Mat result_3 = niswgsp.texture_mapping().clone();// 图像拼接

    end_time = clock();
    LOG("totoal time %f", (double)(end_time - begin_time)/CLOCKS_PER_SEC);

    show_img("3", result_3);
  } else {
    // 调用OpenCV
    vector<Mat> images;
    for (int i = 1; i <= 2; i ++) {
        sprintf(img_path, "%s/%d.jpg", app_path, i);
        Mat tmp_img = imread(img_path);
        images.push_back(tmp_img.clone());
    }
    Mat result_1 = OpenCV_Stitching::opencv_stitch(images);

    show_img("1", result_1);
  }
}

#else

JNIEnv * total_env;
Mat method_openCV(vector<string>);
Mat method_NISwGSP(vector<string>, vector<double>);

extern "C" JNIEXPORT int JNICALL
Java_com_example_niswgsp_11_MainActivity_main_1test(
    JNIEnv* env,
    jobject thiz,
    jobjectArray imgPaths,
    jdoubleArray imgRotations,
    jlong matBGR,
    jint mode) {// mode: 0 for niswgsp, 1 for opencv
  total_env = env;
//  if (total_env != NULL) {
//    jclass clazz = total_env->FindClass("com.example.niswgsp_1/MainActivity");
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

  clock_t begin_time, end_time;
  begin_time = clock();

  Mat result_img;
  int result = 0;
  if (mode == 1) {
      result_img = method_NISwGSP(img_paths, img_rotations);
  } else if (mode == 2) {
      result_img = method_openCV(img_paths);
  }
  LOG("result size %ld %ld", result_img.cols, result_img.rows);
  if (result_img.cols <= 1 || result_img.rows <= 1) {
      // 拼接失败
      set_progress(-100, mode);
      result = -1;
  }

  end_time = clock();
  LOG("totoal time %f", (double)(end_time - begin_time)/CLOCKS_PER_SEC);

  *(Mat *)matBGR = result_img.clone();// 图像拼接

  return result;
}

Mat method_NISwGSP(vector<string> img_paths, vector<double> img_rotations) {
    MultiImages multi_images;
    for (int i = 0; i < img_paths.size(); i ++) {
        const char *img_path = img_paths[i].c_str();
        multi_images.read_img(img_path);
        multi_images.img_rotations.push_back(img_rotations[i]);// 记录拍摄时的旋转角度
        if (i != 0) {
            // 自定义图片配对关系
            multi_images.img_pairs.emplace_back(make_pair(i - 1, i));
        }
    }

    NISwGSP_Stitching niswgsp(multi_images);
    set_progress(5, 1);

    // *(Mat *)matBGR = niswgsp.feature_match().clone();// 特征点
    // *(Mat *)matBGR = niswgsp.matching_match().clone();// 匹配点
    niswgsp.feature_match();// 特征点
    set_progress(30, 1);
    niswgsp.matching_match();// 匹配点
    set_progress(65, 1);

    niswgsp.get_mesh();// 获取最优解
    set_progress(90, 1);
    Mat result = niswgsp.texture_mapping();// 纹理映射
    set_progress(100, 1);

    return result;
}

Mat method_openCV(vector<string> img_paths) {
    vector<Mat> imgs;
    for (int i = 0; i < img_paths.size(); i ++) {
        const char *img_path = img_paths[i].c_str();
        Mat img = imread(img_path);
        imgs.push_back(img);
    }
    set_progress(15, 2);
    Mat result = OpenCV_Stitching::opencv_stitch(imgs);
    set_progress(100, 2);

    return result;
}

void print_message(const char *msg) {
  __android_log_print(ANDROID_LOG_INFO, "fuck", msg);
}

#endif
