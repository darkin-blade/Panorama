#if !defined(Translate_H)
#define Translate_H

#include "../common.h"

class Translate {
public:
  static void computeTranslation(Mat _img1, Mat _img2, double _r1, double _r2, double _r3, Mat & translate);// 计算两张图片之间的位置关系
};

#endif