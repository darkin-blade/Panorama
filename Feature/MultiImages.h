#if !defined(MultiImages_H)
#define MultiImages_H

#include "../common.h"

#include "../Feature/FeatureController.h"
#include "../Feature/ImageData.h"
#include "../Stitch/APAP_Stitching.h"
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

  /* Base */
  int center_index = 0;// 参照图片的索引
  int img_num;
  vector<ImageData *> imgs;
  vector<double>      img_rotations;// 拍摄时的旋转角度

  vector<pair<int, int> > img_pairs;// 图片的配对信息
  vector<vector<bool> >   images_match_graph;// 配对矩阵

  /* Features */
  // [m1][m2]<i, j>,第m1张图片的第i个特征点对应第m2张图片的第j个特征点(实际上[m1][m2]与[m2][m1]重复(相反))
  vector<vector<vector<pair<int, int> > > > initial_pairs;// debug, RANSAC之前的特征点配对信息
  vector<vector<vector<pair<int, int> > > > feature_pairs;// RANSAC之后的特征点配对信息
  vector<vector<vector<Point2f> > >         feature_points;// [m1][m2]: m1与m2成功匹配(RANSAC)的特征点;

  vector<ImageFeatures>             image_features;// TODO 包含keypoints, 包含匹配点和特征点, 函数调用时用
  vector<MatchesInfo>               pairwise_matches;// TODO, 临时构建, 只用于调库函数
  vector<CameraParams>              camera_params;
  vector<vector<bool> >             image_features_mask;// [m1][i],第m1张的第i个匹配点是否可行(只要对任意一张图片可行则可行)

  /* APAP */
  vector<vector<vector<pair<int, int> > > > keypoints_pairs;// (pairwise_matches)(mesh点 + 匹配点)配对信息:[m1][m2]<i, j>,第m1张图片的第i个网格点对应第m2张图片的第j个匹配点

  vector<vector<vector<Mat> > >     apap_homographies;
  vector<vector<vector<bool> > >    apap_overlap_mask;
  vector<vector<vector<Point2f> > > apap_matching_points;

  /* Optimization */
  vector<vector<InterpolateVertex> >      mesh_interpolate_vertex_of_matching_pts;// TODO
  vector<int>                             images_vertices_start_index;// 每个图像第一个顶点的总索引
  vector<vector<pair<double, double> > >  images_relative_rotation_range;// 旋转角度范围
  vector<vector<double> >                 images_polygon_space_matching_pts_weight;// TODO

  vector<SimilarityElements>              images_similarity_elements;// 旋转角度和缩放比

  /* Blending */
  int    using_seam_finder;// 使用接缝线进行图像拼接
  Size2f target_size;// 最终Mat大小

  vector<Mat>              polygon_index_masks;// 整个图片所有像素对应的三角形区域索引
  vector<vector<Mat> >     affine_transforms;// 每个图片的每个mesh(分成两个三角形)内部的单应矩阵变换

  vector<vector<Point2f> > image_mesh_points;// 最终结果(从上往下, 从左往右)
  vector<Mat>              images_warped;// 存放wrap后图片
  vector<Mat>              masks_warped;// 存放wrap后mask
  vector<UMat>             gpu_images_warped;
  vector<UMat>             gpu_masks_warped;

  vector<Point2f>          origins;// 每幅warped图像的初始坐标
  vector<Point2i>          corners;// 初始坐标的整数形式
  vector<Mat>              blend_weight_mask;// new_weight_mask

  /* Seam */
  vector<Mat>              pano_masks_warped;// 在全景中的mask(有偏移)
  vector<Mat>              blocks_warped;// 分水岭之后的图像分区(无偏移)

  /* Line */
  // 已删除

  /* Debug */

  void read_img(const char *img_path);
  void getFeaturePairs();
  vector<pair<int, int> > getVlfeatFeaturePairs(const int m1, const int m2);
  vector<pair<int, int> > getFeaturePairsBySequentialRANSAC(const vector<Point2f> & _X,
                                                            const vector<Point2f> & _Y,
                                                            const vector<pair<int, int> > & _initial_indices);
  vector<vector<InterpolateVertex> > getInterpolateVerticesOfMatchingPoints();
  vector<int> getImagesVerticesStartIndex();
  vector<vector<double> > getImagesGridSpaceMatchingPointsWeight(const double _global_weight_gamma);
  vector<CameraParams> getCameraParams();
  vector<SimilarityElements> getImagesSimilarityElements();

  void do_matching();
  void warpImages();
  void exposureCompensate();// 曝光补偿
  void getBlock();
  void removeMask(const int _src_idx, const int _dst_idx, const int _row, const int _col, Mat &_intersect);
  void getSeam();// 寻找接缝线
  Mat blending();
  Mat textureMapping();
};

#endif