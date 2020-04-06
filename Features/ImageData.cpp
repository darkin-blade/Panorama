#include "ImageData.h"

void ImageData::init_data() {
  ;
}

void ImageData::get_mesh2d_points() {
  // 划分图像mesh
  int cols = data.cols;
  int rows = data.rows;
  // 计算间距
  double ratio = ((double) cols) / rows;
  int row_num = 20;
  int col_num = (int) (ratio * 20);
  double col_step = ((double) cols) / col_num;
  double row_step = ((double) rows) / row_num;
  // 添加mesh
  for (int j = 0; j <= col_num; j ++) {
    for (int k = 0; k <= row_num; k ++) {
      mesh_points.push_back(Point2f(j * col_step, k * row_step));
    }
  }
}

void ImageData::get_polygons_indices() {
  ;
}