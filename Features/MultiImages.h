#include "../common.h"

#include "../Utils/Blending.h"
#include "../Utils/Statistics.h"
#include "../Utils/Transform.h"

#include "../Features/FeatureController.h"
#include "../Features/ImageData.h"

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

class MultiImages {// 注意reserve与resize的区别
public:
  MultiImages();

  int img_num;
  vector<ImageData *> imgs;
  vector<int> triangulation_indices[2];// TODO 用于纹理映射

  // 两辆图片的配对信息:[m1][m2],第m1张图片为参照,与第m2张图片为目标
  vector<vector<vector<pair<int, int> > > > feature_pairs;// 特征点配对信息:[m1][m2]<i, j>,第m1张图片的第i个网格点对应第m2张图片的第j个匹配点
  vector<vector<vector<pair<int, int> > > > matching_pairs;// 匹配点配对信息:[m1][m2]<i, j>,第m1张图片的第i个网格点对应第m2张图片的第j个匹配点

  void read_img(const char *img_path);
  vector<pair<int, int> > getVlfeatFeaturePairs(const int m1, const int m2);
  vector<pair<int, int> > getOpencvFeaturePairs(const int m1, const int m2);
  vector<pair<int, int> > getFeaturePairsBySequentialRANSAC(const vector<Point2f> & _X,
                                                            const vector<Point2f> & _Y,
                                                            const vector<pair<int, int> > & _initial_indices);
  Mat textureMapping(const vector<vector<Point2f> > &_vertices);
  void getFeaturePairs();
};
