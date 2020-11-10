#include "common.h"

#if !defined(UBUNTU)
JNIEnv * total_env = NULL;
jclass total_clazz = NULL;
jmethodID total_id = NULL;
#endif

static char sprint_buf[1024];
static char tmp_fmt[1024];

void print_message(const char *fmt, ...) {
  va_list args;
  int n;
  va_start(args, fmt);

#if defined(UBUNTU)
  sprintf(tmp_fmt, "\033[1;32m%s\33[0m\n", fmt);
#else
  sprintf(tmp_fmt, "%s", fmt);
#endif

  n = vsprintf(sprint_buf, tmp_fmt, args);
  va_end(args);

#if defined(UBUNTU)
  write(STDOUT_FILENO, sprint_buf, n);
#else
  __android_log_vprint(ANDROID_LOG_INFO, "fuck", tmp_fmt, args);
  // 加载到控制台
  if (total_env == NULL) {
    assert(0);
  }
  if (total_clazz == NULL) {
    total_clazz = total_env->FindClass("com.example.my_stitcher/MainActivity");
    if (total_clazz == NULL) {
      assert(0);
    }
  }
  if (total_id == NULL) {
    total_id = total_env->GetStaticMethodID(total_clazz, "jniLog", "(Ljava/lang/String;)V");
    if (total_id == NULL) {
      assert(0);
    }
  }

  jstring msg = total_env->NewStringUTF(sprint_buf);
  total_env->CallStaticVoidMethod(total_clazz, total_id, msg);
#endif
}

void set_progress(const int progress, const int mode) {
#if !defined(UBUNTU)
  // 修改进度条
  if (total_env != NULL) {
    jclass clazz = total_env->FindClass("com.example.my_stitcher/MainActivity");
    if (clazz != NULL) {
        jmethodID id = total_env->GetStaticMethodID(clazz, "jniProgress", "(II)V");
        if (id != NULL) {
            total_env->CallStaticVoidMethod(clazz, id, (jint)progress, (jint)mode);
        } else {
            assert(0);
        }
    } else {
        assert(0);
    }
  }
#endif
}

void show_img(Mat img, const char *fmt, ...) {
  va_list args;
  int n;
  va_start(args, fmt);

  n = vsprintf(sprint_buf, fmt, args);
  va_end(args);

  show_img(sprint_buf, img);
}

void show_img(UMat img, const char *fmt, ...) {
  va_list args;
  int n;
  va_start(args, fmt);

  n = vsprintf(sprint_buf, fmt, args);
  va_end(args);

  show_img(sprint_buf, img);
}

void show_img(const char *window_name, Mat img) {
#if defined(UBUNTU)
  if (img.rows * img.cols <= 0) {
    LOG("invalid img %s", window_name);
    return;
  }

  namedWindow(window_name, WINDOW_NORMAL);
  resizeWindow(window_name, 1280, 720);
  imshow(window_name, img);
  waitKey(0);

  // 保存图片
  char img_name[128];
  int savable = 0;
  for (int i = 0; i < 100; i ++) {
    sprintf(img_name, "../../result_%d.png", i);
    if (fopen(img_name, "r") == NULL) {
      savable = 1;
      break;
    }
  }
  if (savable) {
    imwrite(img_name, img);
  } else {
    LOG("can't save img");
  }
#endif
}

void show_img(const char *window_name, UMat img) {
  Mat tmp;
  img.copyTo(tmp);
  show_img(window_name, tmp);
}