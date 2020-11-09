#if !defined(Translate_H)
#define Translate_H

#include "../common.h"
#include "../Feature/MultiImages.h" /* TODO */
#include "../Util/Statistics.h"


class Translate {
public:
  Translate(Mat _img1, Mat _img2);

  const int SIZE_SMALL = 1920 * 1080;
  const int SIZE_CIRCLE = 8;
  const int SIZE_LINE   = 4;

  Mat img1, img2;
  Mat grey1, grey2;
  /* 特征 */
  vector<Point2f> feature_points1, feature_points2;// 检测出来的特征点
  vector<vector<Mat> > descriptor1, descriptor2;
  vector<pair<int, int> > initial_indices;// RANSAC之前的配对信息
  vector<pair<int, int> > indices;// RANSAC之后的配对信息
  vector<Point2f> feature_pair1, feature_pair2;// 筛选之后的匹配的特征点
  /* 结果 */
  Mat R;// 两张照片之间的相对旋转矩阵
  Mat H;// 单应矩阵
  Mat K;// 内参矩阵
  Mat E;// 本质矩阵
  Mat F;// 基本矩阵
  Mat T;// 平移矩阵

  void setRotation(double _alpha1, double _beta1, double _gamma1,
                   double _alpha2, double _beta2, double _gamma2);
  Mat computeTranslate();// 计算两张图片之间的位置关系
  void getFeaturePairs();
  void getInitialFeaturePairs();
  void getFeaturePairsBySequentialRANSAC(
    const vector<Point2f> & _X,
    const vector<Point2f> & _Y);
  void getDescriptors(
    const Mat & _grey_img,
    vector<Point2f> & _feature_points,
    vector<vector<Mat> > & _feature_descriptors);
  double getDistance(
    const vector<Mat> & _descriptor1,
    const vector<Mat> & _descriptor2,
    const double _threshold);

  /* DEBUG */
  void drawFeature();
};

#endif