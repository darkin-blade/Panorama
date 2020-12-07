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
  int                      img_num;
  vector<ImageData *>      imgs;
  vector<double>           img_rotations;// 拍摄时的旋转角度
  vector<pair<int, int> >  img_pairs;// 图片的配对信息

  /* 特征匹配 */
  // [m1][m2]<i, j>,第m1张图片的第i个特征点对应第m2张图片的第j个特征点(实际上[m1][m2]与[m2][m1]重复(相反))
  vector<vector<vector<pair<int, int> > > > initial_pairs;// debug, RANSAC之前的特征点配对信息
  vector<vector<vector<pair<int, int> > > > feature_pairs;// RANSAC之后的特征点配对信息
  vector<vector<vector<Point2f> > >         feature_points;// [m1][m2]: m1与m2成功匹配(RANSAC)的特征点;

  /* 图像融合 */
  Mat                         pano_mask;// 全景图的mask
  Size2f                      pano_size;
  vector<vector<Point2f> >    matching_pts;// 前半段为mdlt计算的网格点, 后半段为经过平移计算得到的网格点

  /* 图片读取 */
  void readImg(const char *img_path);

  /* 特征匹配 */
  void getFeaturePairs();
  vector<pair<int, int> > getInitialFeaturePairs(const int m1, const int m2);
  vector<pair<int, int> > getFeaturePairsBySequentialRANSAC(
      const vector<Point2f> & _X,
      const vector<Point2f> & _Y,
      const vector<pair<int, int> > & _initial_indices);

  /* 图像配准 */
  void rotateImage(vector<double> _angle, vector<Point2f> _src_p, vector<Point2f> & _dst_p);
  void getFeatureInfo();
  void getMeshInfo();
  void getHomographyInfo();
  void repairWarpping();

  /* 图像形变 */
  void warpImage(
      vector<Point2f> _src_p, vector<Point2f> _dst_p,
      vector<vector<int> > _indices, // 三角形的线性索引
      Mat _src, Mat & _dst, Mat & _mask);
  void warpImage2(
      vector<Point2f> _src_p, vector<Point2f> _dst_p,
      vector<vector<int> > _indices, // 三角形的线性索引
      Mat _src, Mat & _dst, Mat & _mask);

  /* 图像融合 */
  Mat textureMapping(int _mode);// 0 for mdlt, 1: 纯粹的平移
};

#endif