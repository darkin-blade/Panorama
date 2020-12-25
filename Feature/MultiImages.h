#if !defined(MultiImages_H)
#define MultiImages_H

#include "../common.h"

#include "../Feature/FeatureController.h"
#include "../Feature/ImageData.h"
#include "../GraphCut/SeamFinder.h"
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

  /* 相似变换 */
  double    scale;
  double    rotate;// 顺时针(因为坐标系是反的)
  double    shift_x, shift_y;
  Point2f   shift_vec;// 在图片的原始尺寸和方向状态的坐标系, 图片中心从src到dst的平移向量

  /* 图像融合 */
  Size2f                      pano_size;
  vector<vector<Point2f> >    matching_pts;// 前半段为mdlt计算的网格点, 后半段为经过平移计算得到的网格点

  /* 接缝线寻找 */
  vector<Mat>      pano_images;
  vector<Mat>      pano_masks;

  /* 最终结果 */
  Mat    pano_result;

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
  void similarityTransform(int _mode, double _angle);// 0: 平移; 1: 平移 + 缩放; 2: 平移 + 缩放 + 旋转; 在2条件下_angle参数无效
  void repairWarpping();

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
      Mat _src, Mat & _dst, Mat & _weight_mask, Mat & _img_mask);
  void warpImage2(
      vector<Point2f> _src_p, vector<Point2f> _dst_p,
      vector<vector<int> > _indices, // 三角形的线性索引
      Mat _src, Mat & _dst, Mat & _weight_mask, Mat & _img_mask);

  /* 
    特征点形变
    _src_vertices: 原始顶点
    _dst_vertices: 形变之后的顶点
    _indices: 顶点的索引
    _src_features: 原始特征点
    _dst_features: 形变之后的特征点
    */
  void warpPoints(
      vector<Point2f> _src_vertices, vector<Point2f> _dst_vertices,
      vector<vector<int> > _indices,
      vector<Point2f> _src_features, vector<Point2f> & _dst_features);

  /* 图像融合 */
  void textureMapping(int _mode);// 0 for mdlt, 1: 纯粹的平移

  /* 寻找接缝线 */
  void getMask();// 计算接缝线的mask
  void getSeam();

  /* DEBUG */
  void drawPoints(Mat _img, vector<Point2f> _points);
};

#endif