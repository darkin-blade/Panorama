#include "../common.h"

class ImageData {
public:
  // mesh数据
  int nw;// 横向mesh数目
  int nh;// 纵向mesh数目
  double lw;// mesh宽度
  double lh;// mesh高度

  // 图像数据
  Mat data;
  Mat grey_data;
  Mat rgba_data;
  Mat alpha_mask;// TODO
  char *name;

  // mesh相关参数
  vector<Point2f> mesh_points;// 网格点
  vector<vector<int> > polygons_indices;// TODO Indices
  vector<vector<int> > triangulation_indices;// TODO Indices

  vector<vector<Mat> > descriptors;// TODO, 与feature_points数目相等
  vector<Point2f> feature_points;// 特征点(全部)
  vector<KeyPoint> key_points;// 关键点(全部)(包含特征点)

  // [i],以目前图片为目标,第i个图片为参照
  vector<vector<Point2f> > matching_points;// 此图片在第i个图片的匹配点
  vector<vector<Mat> > homographies;// 此图片的单应矩阵在第i张图片的单应矩阵

  void init_data(const char *img_path);
  void get_img(const char *img_path);
  void get_size();
  void get_mesh2d_points();
  void get_polygons_indices();
  void get_triangulation_indices();
};