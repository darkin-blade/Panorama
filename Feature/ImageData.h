#if !defined(ImageData_H)
#define ImageData_H

#include "../common.h"
#include "../Util/Statistics.h"

class ImageData {
public:
  /* 图像数据 */
  Mat data;// 原始数据
  Mat grey_data;// 灰度数据
  Mat mask;// 中间数据 TODO

  /* 特征 */
  vector<vector<Mat> > descriptors;// TODO, 与feature_points数目相等
  vector<Point2f>      feature_points;// 特征点(全部)

  /* 网格 */
  int rows, cols;// 网格顶点的行列数目
  vector<double>       row_vec, col_vec;// 在初始网格时使用的比例序列, 用于计算点在图像中的网格索引
  vector<Point2f>      vertices;// 原始网格顶点
  vector<vector<int> > triangle_indices;// 三角形的线型索引
  vector<vector<int> > rectangle_indices;// 四边形的线性索引

  /* 线性变换 */
  vector<Mat>          homographies;

  void initData();
  void readImg(const Mat & _img, int mode);
  void initVertices(vector<double> _col, vector<double> _row);
  int getGridIndexOfPoint(const Point2f & _p);
  void getInterpolateVertex(
    const Point2f & _p,
    int & _grid_index,
    vector<double> & _weights);
};

#endif
