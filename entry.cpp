#include "common.h"

#include "Stitch/My_Stitching.h"
#include "Stitch/OpenCV_Stitching.h"
#include "Translate/Translate.h"

#if defined(UBUNTU)

int main(int argc, char *argv[]) {

  clock_t begin_time, end_time;
  begin_time = clock();

  char app_path[64] = "../..";
  char img_path[128];// 图片路径

  if (1) {
    // 计算相机平移
    
    // 读取图片
    vector<Mat> imgs;
    int img_num = 7;
    for (int i = 1; i <= img_num; i ++) {
      sprintf(img_path, "%s/%d.jpg", app_path, i);
      imgs.emplace_back(imread(img_path));
    }
    
    // 初始化旋转角度
    vector<vector<double> > angles;
    angles.resize(img_num);
    // angles[0].emplace_back(+1.59);
    // angles[0].emplace_back(-0.14);
    // angles[0].emplace_back(-1.76);
    // angles[1].emplace_back(+1.57);
    // angles[1].emplace_back(-0.15);
    // angles[1].emplace_back(-1.73);
    // angles[2].emplace_back(+1.57);
    // angles[2].emplace_back(-0.16);
    // angles[2].emplace_back(-1.77);
    // angles[3].emplace_back(+1.56);
    // angles[3].emplace_back(-0.09);
    // angles[3].emplace_back(-1.75);
    // angles[4].emplace_back(+1.57);
    // angles[4].emplace_back(-0.23);
    // angles[4].emplace_back(-1.76);
    // angles[5].emplace_back(+1.58);
    // angles[5].emplace_back(-0.12);
    // angles[5].emplace_back(-1.73);
    // angles[6].emplace_back(+1.57);
    // angles[6].emplace_back(-0.14);
    // angles[6].emplace_back(-1.79);
    angles[0].emplace_back(+1.55);
    angles[0].emplace_back(-0.21);
    angles[0].emplace_back(-1.48);
    angles[1].emplace_back(+1.55);
    angles[1].emplace_back(-0.21);
    angles[1].emplace_back(-1.43);
    angles[2].emplace_back(+1.55);
    angles[2].emplace_back(-0.22);
    angles[2].emplace_back(-1.44);
    angles[3].emplace_back(+1.56);
    angles[3].emplace_back(-0.17);
    angles[3].emplace_back(-1.41);
    angles[4].emplace_back(+1.55);
    angles[4].emplace_back(-0.27);
    angles[4].emplace_back(-1.36);
    angles[5].emplace_back(+1.55);
    angles[5].emplace_back(-0.20);
    angles[5].emplace_back(-1.31);
    angles[6].emplace_back(+1.54);
    angles[6].emplace_back(-0.20);
    angles[6].emplace_back(-1.36);

    // 计算平移
    Translate translator(0);
    translator.init(imgs, angles);
    translator.getFeaturePairs();
    translator.computeTranslate(0, 1);
    translator.computeTranslate(0, 2);
    translator.computeTranslate(0, 3);
    translator.computeTranslate(0, 4);
    translator.computeTranslate(0, 5);
    translator.computeTranslate(0, 6);
    translator.computeDistance(1, 0, 2);
    translator.computeDistance(3, 0, 4);
    translator.computeDistance(5, 0, 6);
    // translator.computeTranslate(0, 3);
    // translator.computeTranslate(0, 4);
    // translator.computeTranslate(0, 5);
    // translator.computeTranslate(0, 6);
    // translator.computeDistance(1, 0, 2);
    // translator.computeDistance(0, 1, 2);

    // 计算每幅图像的位置, TODO: 从安卓端用生成树得到匹配
    vector<pair<int, int> > img_pairs;
    translator.computeOrigin(img_pairs);

  } else if (true) {
    // 调用自己的方法
    
    // 读取图片
    MultiImages multi_images;
    for (int i = 1; i <= 2; i ++) {
      sprintf(img_path, "%s/%d.jpg", app_path, i);
      multi_images.readImg(img_path);
    }
    // 自动从前往后匹配
    for (int i = 1; i < multi_images.img_num; i ++) {
      // 自定义图片配对关系,如果配对错误会导致`type == CV_32F || type == CV_64F`错误
      multi_images.img_pairs.emplace_back(make_pair(i - 1, i));
    }
    // 手动匹配
    // multi_images.img_pairs.emplace_back(make_pair(0, 3));
    // multi_images.img_pairs.emplace_back(make_pair(1, 3));
    // multi_images.img_pairs.emplace_back(make_pair(2, 3));

    My_Stitching my_stitcher(multi_images);
    Mat result = my_stitcher.getMyResult();
    // Mat result = my_stitcher.getNISResult();

    end_time = clock();
    LOG("totoal time %lf", (double)(end_time - begin_time)/CLOCKS_PER_SEC);

    show_img("My", result);

  } else {
    // 调用OpenCV
    
    vector<Mat> images;
    for (int i = 1; i <= 2; i ++) {
      sprintf(img_path, "%s/%d.jpg", app_path, i);
      Mat tmp_img = imread(img_path);
      images.push_back(tmp_img);
    }
    Mat result_1 = OpenCV_Stitching::opencv_stitch(images);

    show_img("OpenCV", result_1);

  }
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

  clock_t begin_time, end_time;
  begin_time = clock();

  Mat result_img;
  int result = 0;
  if (mode == METHOD_MY) {
      result_img = method_my(img_paths, img_rotations);
  } else if (mode == METHOD_OPENCV) {
      result_img = method_openCV(img_paths);
  } else {
      LOG("invalide mode %d", mode);
  }
  LOG("result size %ld %ld", result_img.cols, result_img.rows);
  if (result_img.cols <= 1 || result_img.rows <= 1) {
      // 拼接失败
      result = -1;
  }

  end_time = clock();
  LOG("totoal time %f", (double)(end_time - begin_time)/CLOCKS_PER_SEC);

  *(Mat *)matBGR = result_img.clone();// 图像拼接

  return result;
}

Mat method_my(vector<string> img_paths, vector<double> img_rotations) {
    MultiImages multi_images;
    for (int i = 0; i < img_paths.size(); i ++) {
        const char *img_path = img_paths[i].c_str();
        multi_images.readImg(img_path);
        multi_images.img_rotations.push_back(img_rotations[i]);// 记录拍摄时的旋转角度
        if (i != 0) {
            // 自定义图片配对关系
            multi_images.img_pairs.emplace_back(make_pair(i - 1, i));
        }
    }

    My_Stitching my_stitcher(multi_images);
    Mat result = my_stitcher.getNISResult();// 纹理映射

    return result;
}

Mat method_openCV(vector<string> img_paths) {
    vector<Mat> imgs;
    for (int i = 0; i < img_paths.size(); i ++) {
        const char *img_path = img_paths[i].c_str();
        Mat img = imread(img_path);
        imgs.push_back(img);
    }
    Mat result = OpenCV_Stitching::opencv_stitch(imgs);

    return result;
}

void print_message(const char *msg) {
  __android_log_print(ANDROID_LOG_INFO, "fuck", msg);
}

#endif
