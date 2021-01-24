#if !defined(ImageData_H)
#define ImageData_H

#include "../common.h"
#include "../Util/Statistics.h"

class ImageData {
public:
  /* 图像数据 */
  char *name;
  Mat data;// 原始数据
  Mat grey_data;// 灰度数据
  Mat mask;// 中间数据 TODO

  /* 特征 */
  vector<vector<Mat> > descriptors;// TODO, 与feature_points数目相等
  vector<Point2f>      feature_points;// 特征点(全部)

  /* 网格 */
  int rows, cols;// 网格顶点的行列数目 
  vector<Point2f>      vertices;// 原始网格顶点
  vector<vector<int> > triangle_indices;// 三角形的线型索引
  vector<vector<int> > rectangle_indices;// 四边形的线性索引

  /* 线性变换 */
  vector<Mat>          homographies;

  void readImg(const char *img_path);
  void readImg(const Mat & _img, const Mat & _mask);// 直接由mat初始化
  void initVertices(vector<double> _col, vector<double> _row);
};

#endif
