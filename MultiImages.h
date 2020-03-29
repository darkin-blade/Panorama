#include "common.h"

using namespace std;

using namespace cv;
using namespace cv::detail;
using namespace cv::xfeatures2d;

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
    _feature_index_1[    _p] = _feature_index_1;
    _feature_index_1[1 - _p] = _feature_index_2;
  }

  bool operator < (const FeatureDistance &fd) const {
      return distance > fd.distance;
  }
};

class ImageData {
public:
  Mat data;// 原始数据
  vector<vector<Mat> > descriptors;// TODO, 与feature_points数目相等
  vector<Point2f> mesh_points;// 网格点
  vector<Point2f> feature_points;// 特征点(全部)
  vector<KeyPoint> key_points;// 关键点(全部)(包含特征点)

  // [i],以目前图片为目标,第i个图片为参照
  vector<vector<Point2f> > matching_points;// 第i个图片在此图片的匹配点
  vector<vector<Mat> > homographies;// 第i个图片在此图片的单应矩阵
};

class MultiImages {// 注意reserve与resize的区别
public:
  MultiImages();

  int img_num;
  vector<ImageData *> imgs;

  // 两辆图片的配对信息:[m1][m2],第m1张图片为参照,与第m2张图片为目标
  vector<vector<pair<int, int> > > matching_pairs;// 匹配点配对信息:[m1][m2]<i, j>,第m1张图片的第i个网格点对应第m2张图片的第j个匹配点

  void read_img(const char *img_path);
  vector<pair<int, int> > getInitialFeaturePairs(const int m1, const int m2);
};
