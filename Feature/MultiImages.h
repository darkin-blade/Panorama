#if !defined(MultiImages_H)
#define MultiImages_H

#include "../common.h"

#include "../Feature/FeatureController.h"
#include "../Feature/ImageData.h"
#include "../Stitch/Homographies.h"
#include "../Util/Blending.h"
#include "../Util/Statistics.h"
#include "../Util/Transform.h"

class FeatureDistance {
public:
  double distance;
  int feature_index[2];

  FeatureDistance() {// 无参初始化
    feature_index[0] = feature_index[1] = -1;
    distance = MAXFLOAT;
  }

  FeatureDistance(const double _distance,
                  const int _p,
                  const int _feature_index_1,
                  const int _feature_index_2) {// 有参构造函数
    distance = _distance;
    feature_index[    _p] = _feature_index_1;
    feature_index[1 - _p] = _feature_index_2;
  }

  bool operator < (const FeatureDistance &fd) const {
      return distance > fd.distance;
  }
};

class MultiImages {
public:
  MultiImages();

  /* 原始数据 */
  int img_num;
  vector<ImageData *>      imgs;
  vector<double>           img_rotations;// 拍摄时的旋转角度
  vector<pair<int, int> >  img_pairs;// 图片的配对信息

  /* 特征匹配 */
  // [m1][m2]<i, j>,第m1张图片的第i个特征点对应第m2张图片的第j个特征点(实际上[m1][m2]与[m2][m1]重复(相反))
  vector<vector<vector<pair<int, int> > > > initial_pairs;// debug, RANSAC之前的特征点配对信息
  vector<vector<vector<pair<int, int> > > > feature_pairs;// RANSAC之后的特征点配对信息
  vector<vector<vector<Point2f> > >         feature_points;// [m1][m2]: m1与m2成功匹配(RANSAC)的特征点;

  /* 图片读取 */
  void readImg(const char *img_path);

  /* 特征匹配 */
  void getFeaturePairs();
  vector<pair<int, int> > getInitialFeaturePairs(const int m1, const int m2);
  vector<pair<int, int> > getFeaturePairsBySequentialRANSAC(
      const vector<Point2f> & _X,
      const vector<Point2f> & _Y,
      const vector<pair<int, int> > & _initial_indices);

  /* 自定义 */
  void rotateImage(vector<double> _angle, vector<Point2f> _src_p, vector<Point2f> & _dst_p);
  void getFeatureInfo();
  void getMeshInfo();
  void getHomographyInfo();
  void warpImage(
      vector<Point2f> _src_p, vector<Point2f> _dst_p,
      vector<vector<int> > _indices, // 三角形的线性索引
      Mat _src, Mat & _dst);
};

#endif