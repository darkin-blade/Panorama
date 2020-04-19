#if !defined(MultiImages_H)
#define MultiImages_H

#include "../common.h"

#include "../Feature/FeatureController.h"
#include "../Feature/ImageData.h"
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

class dijkstraNode {
public:
  int from, pos;
  double dis;
  dijkstraNode(const int & _from,
      const int & _pos,
      const double & _dis) : from(_from), pos(_pos), dis(_dis) {
  }
  bool operator < (const dijkstraNode & rhs) const {
    return dis > rhs.dis;
  }
};

class SimilarityElements {
public:
  double scale;
  double theta;
  SimilarityElements() {// TODO
    scale = 1;
    theta = 0;
  };
  SimilarityElements(const double _scale,
                     const double _theta) {
    scale = _scale;
    theta = _theta;
  }
};

class MultiImages {// 注意reserve与resize的区别
public:
  MultiImages();

  int img_num;
  vector<ImageData *> imgs;

  // 两辆图片的配对信息:[m1][m2],第m1张图片为参照,与第m2张图片为目标
  vector<vector<vector<pair<int, int> > > > feature_pairs;// 特征点配对信息:[m1][m2]<i, j>,第m1张图片的第i个网格点对应第m2张图片的第j个匹配点
  vector<vector<vector<pair<int, int> > > > matching_pairs;// (过滤后)匹配点配对信息:[m1][m2]<i, j>,第m1张图片的第i个网格点对应第m2张图片的第j个匹配点
  vector<vector<bool> > matching_mask;// [m1][i],第m1张的第i个匹配点是否可行(只要对任意一张图片可行则可行)

  vector<vector<InterpolateVertex> > mesh_interpolate_vertex_of_matching_pts;// TODO
  vector<int> images_vertices_start_index;// TODO
  vector<vector<double> > images_polygon_space_matching_pts_weight;

  vector<SimilarityElements> images_similarity_elements;// TODO

  vector<vector<Point2f> > image_mesh_points;// 最终结果

  void read_img(const char *img_path);
  void getFeaturePairs();
  vector<pair<int, int> > getVlfeatFeaturePairs(const int m1, const int m2);
  vector<pair<int, int> > getFeaturePairsBySequentialRANSAC(const vector<Point2f> & _X,
                                                            const vector<Point2f> & _Y,
                                                            const vector<pair<int, int> > & _initial_indices);
  vector<vector<InterpolateVertex> > getInterpolateVerticesOfMatchingPoints();
  vector<int> getImagesVerticesStartIndex();
  vector<vector<double> > getImagesGridSpaceMatchingPointsWeight(const double _global_weight_gamma);
  vector<SimilarityElements> getImagesSimilarityElements();
  Mat textureMapping(vector<vector<Point2f> > &_vertices,
                     int _blend_method);
};

#endif