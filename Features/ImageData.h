#include "../common.h"

class ImageData {
public:
  // 原始数据
  Mat data;
  char *name;

  // mesh相关参数
  vector<Point2f> mesh_points;// 网格点
  vector<int> polygons_indices;// TODO Indices

  vector<vector<Mat> > descriptors;// TODO, 与feature_points数目相等
  vector<Point2f> feature_points;// 特征点(全部)
  vector<KeyPoint> key_points;// 关键点(全部)(包含特征点)

  // [i],以目前图片为目标,第i个图片为参照
  vector<vector<Point2f> > matching_points;// 此图片在第i个图片的匹配点
  vector<vector<Mat> > homographies;// 此图片的单应矩阵在第i张图片的单应矩阵

  void init_data();
  void get_mesh2d_points();
  void get_polygons_indices();
};