#include "ImageData.h"

void ImageData::initData() {
  // 清空所有数据
  descriptors.clear();
  feature_points.clear();
  rows = cols = 0;// 网格顶点的行列数目 
  row_vec.clear(), col_vec.clear();

  vertices.clear();
  triangle_indices.clear();
  polygons_indices.clear();
  homographies.clear();
  edges.clear();
  polygons_neighbors.clear();
  vertex_structures.clear();
  polygons_center.clear();
  polygons_weight.clear();
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

  // 记录顶点: 从左往右, 从上往下
  assert(vertices.empty());
  for (int i = 0; i < rows; i ++) {
    for (int j = 0; j < cols; j ++) {
      // 记录网格顶点
      vertices.emplace_back(data.cols * _col[j], data.rows * _row[i]);
    }
  }

  getPolygonsCenter();
  getIndices();
  getEdges();
  getPolygonsNeighbors();
  getVertexStructures();
}

void ImageData::getIndices() {
  // 记录网格(三角形, 四边形)线性索引
  assert(triangle_indices.empty());
  assert(polygons_indices.empty());
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
      polygons_indices.emplace_back(indice_3);
    }
  }
}

void ImageData::getPolygonsCenter() {
  assert(polygons_center.empty());
  for (int i = 0; i < polygons_indices.size(); i ++) {
    Point2f center(0, 0);
    for (int j = 0; j < polygons_indices[i].size(); j ++) {
      center += vertices[polygons_indices[i][j]];
    }
    // 求4个顶点的平均值
    assert(polygons_indices[i].size() == 4);
    polygons_center.emplace_back(center / 4);
  }
}

void ImageData::getEdges() {
  assert(edges.empty());
  const vector<Point2i> nexts = { Point2i(1, 0), Point2i(0, 1) };
  for (int r = 0; r < rows; r ++) {
    for (int c = 0; c < cols; c ++) {
      const Point2i p1(c, r);
      for (int i = 0; i < nexts.size(); i ++) {
        const Point2i p2 = p1 + nexts[i];
        if (p2.x >= 0 && p2.y >= 0 && p2.x < cols && p2.y < rows) {
          edges.emplace_back(p1.x + p1.y * cols, p2.x + p2.y * cols);
        }
      }
    }
  }
  assert(edges.size() == 2 * cols * rows - cols - rows);
}

void ImageData::getPolygonsNeighbors() {
  assert(polygons_neighbors.empty());
  const vector<Point2i> nexts = { Point2i(1, 0), Point2i(0, 1), Point2i(-1, 0), Point2i(0, -1) };
  polygons_neighbors.resize((rows - 1)*(cols - 1));// 网格数(不是顶点数)
  int index = 0;
  for (int r = 0; r < rows - 1; r ++) {
    for (int c = 0; c < cols - 1; c ++) {
      const Point2i p1(c, r);
      for (int i = 0; i < nexts.size(); i ++) {
        const Point2i p2 = p1 + nexts[i];
        if (p2.x >= 0 && p2.y >= 0 && p2.x < cols - 1 && p2.y < rows - 1) {
          polygons_neighbors[index].emplace_back(p2.x + p2.y * (cols - 1));
        }
      }
      index ++;
    }
  }
}

void ImageData::getVertexStructures() {
  assert(vertex_structures.empty());
  const vector<Point2i> nexts = { Point2i(1, 0), Point2i(0, 1), Point2i(-1, 0), Point2i(0, -1) };
  vertex_structures.resize(rows * cols);// 顶点数
  int index = 0;
  for (int r = 0; r < rows; r ++) {
    for (int c = 0; c < cols; c ++) {
      Point2i p1(c, r);
      for (int i = 0; i < nexts.size(); i ++) {
        Point2i p2 = p1 + nexts[i];
        if (p2.x >= 0 && p2.y >= 0 && p2.x < cols && p2.y < rows) {
          vertex_structures[index].emplace_back(p2.x + p2.y * cols);
        }
      }
      index ++;
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
  return col_index + row_index * (rows - 1);
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
  vector<int> g = polygons_indices[_grid_index];

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