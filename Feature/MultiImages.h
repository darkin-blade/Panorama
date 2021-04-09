#if !defined(MultiImages_H)
#define MultiImages_H

#include "../common.h"

#include "../Feature/FeatureController.h"
#include "../Feature/ImageData.h"
#include "../GraphCut/SeamFinder.h"
#include "../Stitch/Homographies.h"
#include "../Util/Blending.h"
#include "../Util/Rotation.h"
#include "../Util/Similarity.h"
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

  FeatureDistance(
      const double _distance,
      const int _p,
      const int _feature_index_1,
      const int _feature_index_2) {// 有参构造函数
    distance = _distance;
    feature_index[    _p] = _feature_index_1;
    feature_index[1 - _p] = _feature_index_2;
  }

  bool operator < (const FeatureDistance & fd) const {
    return distance > fd.distance;
  }
};

class ImageDistance {
public:
  int u;
  int v;
  double distance;

  ImageDistance(
      const int _u,
      const int _v,
      const double _d) {
    u = _u;
    v = _v;
    distance = _d;
  }

  bool operator < (const ImageDistance & id) const {
    return distance > id.distance;
  }
};

class UnionFind {
public:
  vector<int> father;// 该节点的父节点
  vector<int> sizes;// 与该节点连通的节点数
  int total_size;
  int cur_size;

  UnionFind(int num) {
    father.resize(num);
    sizes.resize(num);
    for (int i = 0; i < num; i ++) {
      father[i] = i;
      sizes[i] = 1;
    }
    total_size = num;
    cur_size = 1;
  }

  int getFather(int x) {
    if (father[x] != x) {
      father[x] = getFather(father[x]);
    }
    return father[x];
  }

  void unionNode(int x, int y) {
    int father_x = getFather(x);
    int father_y = getFather(y);
    if (father_x == father_y) {
      return;
    }
    int size_x = sizes[father_x];
    int size_y = sizes[father_y];
    father[father_x] = father_y;
    sizes[father_y] = size_x + size_y;
    if (sizes[father_y] > cur_size) {
      cur_size = sizes[father_y];
      assert(cur_size <= total_size);
    }
  }
};

class MultiImages {
public:
  MultiImages();

  /* 原始数据 */
  int                      img_num;
  vector<Mat>              origin_data;// 未进行缩放的图像数据
  vector<ImageData *>      imgs;

  /* 特征匹配 */
  vector<pair<int, int> >                   initial_pairs;// debug, RANSAC之前的特征点配对信息
  vector<pair<int, int> >                   feature_pairs;// RANSAC之后的特征点配对信息
  vector<vector<vector<Point2f> > >         feature_points;// RANSAC的特征点, [m1][m2]: m1与m2配对的特征点

  /* 相似变换 */
  vector<double>           rotations;
  vector<double>           scales;
  vector<Point2f>          translations;
  vector<pair<int, int> >  img_pairs;// TODO, 图片的配对信息, 没用
  
  /* 网格变换 */
  vector<vector<bool> >               total_mask;// 目标图像在参考图像上未出界的匹配点mask
  vector<vector<vector<bool> > >      single_mask;// [m1][m2][j]: m1的第j个匹配点是否在m2上出界
  vector<vector<vector<Point2f> > >   apap_pts;// apap的结果, [m1][m2]: m1与m2的结果
  vector<vector<Point2f> >            similarity_pts;// 相似变换的结果, [i]: 第i张图片
  vector<vector<Point2f> >            matching_pts;// 最终结果;

  /* 网格优化 */
  vector<vector<int> >                pair_index;// 记录和第i张图片配对的图片索引
  vector<int>                         image_order;// 添加图片的顺序
  double alignment_weight               = 1 * 1;
  double local_similarity_weight        = 5 * 1;// 0.56
  double sensor_weight                  = 1 * 1;
  // 下面3项pair的含义:
  // first: 该部分等式中第一个等式在所有等式中的索引
  // second: 该部分等式含有的等式的总数
  pair<int, int> alignment_equation;
  pair<int, int> local_similarity_equation;
  // 传感器数据
  pair<int, int> sensor_equation;

  /* 图像融合 */
  Size2i                  pano_size;// 最终结果的图像大小

  /* 接缝线寻找 */
  vector<Mat>      pano_images;
  vector<Mat>      pano_masks;
  vector<Mat>      origin_masks;// 寻找接缝线前的mask

  /* 最终结果 */
  Mat    pano_result;

  /* debug */
  int    total_eq;

  /* 图片读取 */
  void readImg(const char *img_path);
  void readAngle(const char *path);

  /* 预处理 */
  void init();

  /* 特征匹配 */
  void getFeaturePairs(int _m1, int _m2);
  vector<pair<int, int> > getInitialFeaturePairs(int _m1, int _m2);
  vector<pair<int, int> > getFeaturePairsBySequentialRANSAC(
      const vector<Point2f> & _X,
      const vector<Point2f> & _Y,
      const vector<pair<int, int> > & _initial_indices);

  /* 图像配准 */
  void getFeatureInfo();
  void getMeshInfo();
  void similarityTransform(int _mode);// 0: 平移; 1: 平移 + 缩放;
  void getImagePairs();// 根据位置关系计算图像之间的配对关系

  /* 网格优化 */
  void meshOptimization();
  void reserveData(
      int _m1,
      vector<Triplet<double> > & _triplets, 
      vector<pair<int, double> > & _b_vector);
  void prepareAlignmentTerm(
      int _m1,
      vector<Triplet<double> > & _triplets, 
      vector<pair<int, double> > & _b_vector);
  void prepareLocalSimilarityTerm(
    int _m1,
      vector<Triplet<double> > & _triplets, 
      vector<pair<int, double> > & _b_vector);
  void prepareSensorTerm(
      int _m1,
      vector<Triplet<double> > & _triplets, 
      vector<pair<int, double> > & _b_vector);
  void getSolution(
      int _m1,
      vector<Triplet<double> > & _triplets, 
      vector<pair<int, double> > & _b_vector);

  /**
    图像形变
    _src_p: 图像原始顶点
    _dst_p: 形变之后的顶点
    _indices: 顶点的索引
    _src: 原图片
    _dst: 输出的图片
    _weight_mask: 权值mask(用于线性融合)
    _img_mask: 图片mask(用于接缝线)
    */
  void warpImage(
      vector<Point2f> _src_p, vector<Point2f> _dst_p,
      vector<vector<int> > _indices, // 三角形的线性索引
      Mat _src, Mat & _dst, Mat & _img_mask);
  void warpImage2(
      vector<Point2f> _src_p, vector<Point2f> _dst_p,
      vector<vector<int> > _indices, // 四边形的线性索引
      Mat _src, Mat & _dst, Mat & _img_mask);
  void myWarping();

  /* 图像融合 */
  void myBlending(
      Mat src_image,
      Mat dst_image,
      Mat src_mask,
      Mat dst_mask,
      Mat src_origin, // 用于扩展
      Mat dst_origin,
      Mat & result_image,
      Mat & result_mask);

  /* 寻找接缝线 */
  void getMask();// 计算接缝线的mask
  void getSeam();

  /* DEBUG */
  void getTmpResult();
  Mat drawPoints(Mat _img, vector<Point2f> _points);
};

#endif