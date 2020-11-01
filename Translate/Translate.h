#if !defined(Translate_H)
#define Translate_H

#include "../common.h"
#include "../Feature/MultiImages.h" /* TODO */
#include "../Util/Statistics.h"


class Translate {
public:
  Translate(Mat _img1, Mat _img2, double _r1, double _r2, double _r3);

  Mat img1, img2;
  Mat grey1, grey2;
  /* 特征 */
  vector<Point2f> feature_points1, feature_points2;// 检测出来的特征点
  vector<vector<Mat> > descriptor1, descriptor2;
  vector<pair<int, int> > initial_indices;// RANSAC之前的配对信息
  vector<pair<int, int> > indices;// RANSAC之后的配对信息
  /* 结果 */
  Mat rotate;
  Mat translate;

  void compute(Mat & translate);// 计算两张图片之间的位置关系
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