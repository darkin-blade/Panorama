#include "ImageData.h"

void ImageData::initData() {
  descriptors.clear();
  feature_points.clear();
  rows = cols = 0;// 网格顶点的行列数目 
  row_vec.clear(), col_vec.clear();

  vertices.clear();
  triangle_indices.clear();
  rectangle_indices.clear();
  homographies.clear();
}

void ImageData::readImg(const Mat & _img, int mode) {
  LOG("origin channels %d", _img.channels());
  _img.copyTo(data);

  if (mode == 1) {
    // 原图
    mask = Mat(data.size(), CV_8UC1, Scalar(255));
  } else if (mode == 2) {
    // 需手动提取mask
    assert(data.channels() == 4);
    vector<Mat> channels;
    split(data, channels);
    channels[3].copyTo(mask);
  } else {
    assert(0);
  }

  // 修改通道
  if (data.channels() == 4) {
    cvtColor(data, data, CV_RGBA2RGB);
  }
  cvtColor(data, grey_data, CV_BGR2GRAY);// 灰色图
}

void ImageData::initVertices(vector<double> _col, vector<double> _row) {
  // 将原始图像按照_col, _row中的比例划分网格
  cols = _col.size();
  rows = _row.size();
  assert(cols >= 2);
  assert(rows >= 2);

  // 便于查找grid索引
  assert(row_vec.empty() && col_vec.empty());
  for (int i = 0; i < cols; i ++) {
    col_vec.emplace_back(_col[i] * data.cols);
  }
  for (int i = 0; i < rows; i ++) {
    row_vec.emplace_back(_row[i] * data.rows);
  }

  // 从左往右, 从上往下
  assert(vertices.empty());
  for (int i = 0; i < rows; i ++) {
    for (int j = 0; j < cols; j ++) {
      // 记录网格顶点
      vertices.emplace_back(data.cols * _col[j], data.rows * _row[i]);
    }
  }

  // 记录网格线性索引
  assert(triangle_indices.empty());
  assert(rectangle_indices.empty());
  for (int i = 0; i < rows - 1; i ++) {
    for (int j = 0; j < cols - 1; j ++) {
      // 将一个mesh分为两个三角形记录
      vector<int> indice_1;
      vector<int> indice_2;
      // 左上, 右上, 右下
      indice_1.emplace_back(i * cols + j);
      indice_1.emplace_back(i * cols + (j + 1));
      indice_1.emplace_back((i + 1) * cols + (j + 1));
      // 左上, 左下, 右下
      indice_2.emplace_back(i * cols + j);
      indice_2.emplace_back((i + 1) * cols + j);
      indice_2.emplace_back((i + 1) * cols + (j + 1));
      triangle_indices.emplace_back(indice_1);
      triangle_indices.emplace_back(indice_2);
      // 将一个mesh分为一个矩形记录
      vector<int> indice_3;
      // 左上, 右上, 右下, 左下
      indice_3.emplace_back(i * cols + j);
      indice_3.emplace_back(i * cols + (j + 1));
      indice_3.emplace_back((i + 1) * cols + (j + 1));
      indice_3.emplace_back((i + 1) * cols + j);
      rectangle_indices.emplace_back(indice_3);
    }
  }
}

int ImageData::getGridIndexOfPoint(const Point2f & _p) {
  assert(_p.x >= 0);
  assert(_p.x >= 0);
  assert(_p.x <= data.cols);
  assert(_p.y <= data.rows);

  // 计算某点在这幅图中所在的grid位置
  int row_index = 0, col_index = 0;
  while (_p.x > col_vec[col_index + 1]) {
    col_index ++;
    assert(col_index + 1 < cols);
  }
  while (_p.y > row_vec[row_index + 1]) {
    row_index ++;
    assert(row_index + 1 < rows);
  }
  return col_index + row_index * rows;
}

void ImageData::getInterpolateVertex(
    const Point2f & _p,
    int & _grid_index,
    vector<double> & _weights) {
  // 计算某点在这幅图片中所在的grid位置, 并求出在这个grid内与4个顶点的归一化距离
  assert(_weights.size() == 4);
  assert(!vertices.empty());

  // 计算grid索引
  _grid_index = getGridIndexOfPoint(_p);

  // 获取grid的4个顶点
  assert(!rectangle_indices.empty());
  vector<int> g = rectangle_indices[_grid_index];
  LOG("%d %d", rectangle_indices.size(), _grid_index);
  // LOG("%d %d %d %d %d", vertices.size(), g[0], g[1], g[2], g[3]);

  // 需要按一定次序存储 TODO
  const vector<int> diagonal_indices = {2, 3, 0, 1};
  double sum_inv = 0;// 用于归一化
  for (int i = 0; i < 4; i ++) {
    Point2f tmp(_p.x - vertices[g[diagonal_indices[i]]].x, _p.y - vertices[g[diagonal_indices[i]]].y);
    _weights[i] = fabs(tmp.x * tmp.y);
    sum_inv += _weights[i];
  }
  sum_inv = 1. / sum_inv;
  for (int i = 0; i < 4; i ++) {
    _weights[i] *= sum_inv;
  }
}