#include "ImageData.h"

LineData::LineData(const Point2f & _a,
    const Point2f & _b,
    const double _width,
    const double _length) {
  data[0] = _a;
  data[1] = _b;
  width   = _width;
  length  = _length;
}

void ImageData::init_data(const char *img_path) {
  get_img(img_path);
  get_size();
}

void ImageData::get_img(const char *img_path) {
  data = imread(img_path);
  rgba_data = imread(img_path, IMREAD_UNCHANGED);
  grey_data = Mat();// 灰色图
  cvtColor(data, grey_data, CV_BGR2GRAY);

  float original_img_size = data.rows * data.cols;

  if (original_img_size > DOWN_SAMPLE_IMAGE_SIZE) {
    float scale = sqrt(DOWN_SAMPLE_IMAGE_SIZE / original_img_size);
    resize(data, data, Size(), scale, scale);
    resize(rgba_data, rgba_data, Size(), scale, scale);
  }

  assert(rgba_data.channels() >= 3);
  if (rgba_data.channels() == 3) {
    cvtColor(rgba_data, rgba_data, CV_BGR2BGRA);
  }
  vector<Mat> channels;
  split(rgba_data, channels);
  alpha_mask = channels[3];
}

/** Mesh2D **/

void ImageData::get_size() {
  nw = data.cols / GRID_SIZE + (data.cols % GRID_SIZE != 0);
  nh = data.rows / GRID_SIZE + (data.rows % GRID_SIZE != 0);
  lw = data.cols / (double)nw;
  lh = data.rows / (double)nh;
}

int ImageData::getGridIndexOfPoint(const Point2f & _p) {
  Point2i grid_p(_p.x / lw, _p.y / lh);
  grid_p.x = grid_p.x - (grid_p.x == nw);
  grid_p.y = grid_p.y - (grid_p.y == nh);
  return grid_p.x + grid_p.y * nw;
}

/** MeshGrid **/

vector<int> ImageData::getBoundaryVertexIndices() {
  if (boundary_edge_indices.empty()) {
      const int memory = DIMENSION_2D * (nh + nw);
      boundary_edge_indices.reserve(memory);
      const int bottom_shift = DIMENSION_2D * nh * nw + nh;
      for (int w = 0; w < nw; w ++) {
          boundary_edge_indices.emplace_back(2 * w);
          boundary_edge_indices.emplace_back(bottom_shift + w);
      }
      const int dh = 2 * nw + 1;
      for (int h = 0; h < nh; h ++) {
          int tmp = h * dh;
          boundary_edge_indices.emplace_back(tmp + 1);
          boundary_edge_indices.emplace_back(tmp + dh - 1);
      }
      assert(memory == boundary_edge_indices.size());
  }
  return boundary_edge_indices;
}

vector<Point2f> ImageData::getVertices() {
  if (mesh_points.empty()) {
    const int memory = (nh + 1) * (nw + 1);
    mesh_points.reserve(memory);
    for (int h = 0; h <= nh; h ++) {
      for (int w = 0; w <= nw; w ++) {
        mesh_points.emplace_back(w * lw, h * lh);
      }
    }
    assert(memory == mesh_points.size());
  }
  return mesh_points;
}

vector<vector<int> > ImageData::getTriangulationIndices() {
  if (triangulation_indices.empty()) {
    // 三角填充区域, 原值[[0, 1, 2], [0, 2, 3]], 即用两个三角形填满[0, 1, 2, 3](顺时针)的矩形区域
    triangulation_indices.resize(2);
    triangulation_indices[0].emplace_back(0);
    triangulation_indices[0].emplace_back(1);
    triangulation_indices[0].emplace_back(2);
    triangulation_indices[1].emplace_back(0);
    triangulation_indices[1].emplace_back(2);
    triangulation_indices[1].emplace_back(3);
  }
  return triangulation_indices;
}

vector<vector<int> > ImageData::getPolygonsIndices() {
  // 获取每个mesh格子的4个顶点
  if (polygons_indices.empty()) {
    const Point2i nexts[GRID_VERTEX_SIZE] = {// 左上, 右上, 右下, 左下
      Point2i(0, 0), Point2i(1, 0), Point2i(1, 1), Point2i(0, 1)
    };
    const int memory = nh * nw;// mesh点数目
    polygons_indices.resize(memory);
    int index = 0;
    for (int h = 0; h < nh; h ++) {// 从上到下
      for (int w = 0; w < nw; w ++) {// 从左到右
        const Point2i p1(w, h);
        polygons_indices[index].reserve(GRID_VERTEX_SIZE);
        for (int n = 0; n < GRID_VERTEX_SIZE; n ++) {
          const Point2i p2 = p1 + nexts[n];
          polygons_indices[index].emplace_back(p2.x + p2.y * (nw + 1));// 横向索引 + 纵向索引 * (横向mesh数目)
        }
        index ++;
      }
    }
    assert(memory == polygons_indices.size());
  }
  return polygons_indices;
}

vector<Edge> ImageData::getEdges() {
  // 每个vertex的right, down邻接的vertex
  if (edges.empty()) {
    const vector<Point2i> nexts = { Point2i(1, 0), Point2i(0, 1) };
    const int memory = DIMENSION_2D * nh * nw + nh + nw;
    edges.reserve(memory);
    for (int h = 0; h <= nh; h ++) {
      for (int w = 0; w <= nw; w ++) {
        const Point2i p1(w, h);
        for (int n = 0; n < nexts.size(); n ++) {
          const Point2i p2 = p1 + nexts[n];
          if (p2.x >= 0 && p2.y >= 0 && p2.x <= nw && p2.y <= nh) {
            edges.emplace_back(p1.x + p1.y * (nw + 1),
                p2.x + p2.y * (nw + 1));
          }
        }
      }
    }
    assert(memory == edges.size());
  }
  return edges;
}

vector<vector<int> > ImageData::getPolygonsNeighbors() {
  // 所有vertex邻接vertex的索引(不包含边界)
  if (polygons_neighbors.empty()) {
    const vector<Point2i> nexts = {
      Point2i(1, 0), Point2i(0, 1), Point2i(-1, 0), Point2i(0, -1)
    };
    const int memory = nh * nw;
    polygons_neighbors.resize(memory);
    int index = 0;
    for (int h = 0; h < nh; h ++) {
      for (int w = 0; w < nw; w ++) {
        const Point2i p1(w, h);
        for (int n = 0; n < nexts.size(); n ++) {
          const Point2i p2 = p1 + nexts[n];
          if (p2.x >= 0 && p2.y >= 0 && p2.x < nw && p2.y < nh) {
            polygons_neighbors[index].emplace_back(p2.x + p2.y * nw);
          }
        }
        index ++;
      }
    }
    assert(memory == polygons_neighbors.size());
  }
  return polygons_neighbors;
}

vector<vector<int> > ImageData::getVertexStructures() {
  // 所有vertex邻接vertex的索引(包含边界)
  if (vertex_structures.empty()) {
    const vector<Point2i> nexts = {
      Point2i(1, 0), Point2i(0, 1), Point2i(-1, 0), Point2i(0, -1)
    };
    const int memory = (nh + 1) * (nw + 1);
    vertex_structures.resize(memory);
    int index = 0;
    for (int h = 0; h <= nh; h ++) {
      for (int w = 0; w <= nw; w ++) {
        Point2i p1(w, h);
        for (int n = 0; n < nexts.size(); n ++) {
          Point2i p2 = p1 + nexts[n];
          if (p2.x >= 0 && p2.y >= 0 && p2.x <= nw && p2.y <= nh) {
            vertex_structures[index].emplace_back(p2.x + p2.y * (nw + 1));
          }
        }
        index ++;
      }
    }
    assert(memory == vertex_structures.size());
  }
  return vertex_structures;
}

vector<Point2f> ImageData::getPolygonsCenter() {
  // 所有vertex的中心
  if (polygons_center.empty()) {
    const vector<Point2f> mesh_points = getVertices();
    const vector<vector<int> > polygons_indices = getPolygonsIndices();
    polygons_center.reserve(polygons_indices.size());
    for (int i = 0; i < polygons_indices.size(); i ++) {
      Point2f center(0, 0);
      for (int j = 0; j < polygons_indices[i].size(); j ++) {
        center += mesh_points[polygons_indices[i][j]];
      }
      polygons_center.emplace_back(center / (float)polygons_indices[i].size());
    }
  }
  return polygons_center;
}

vector<vector<int> > ImageData::getEdgeStructures() {
  // TODO
  if (edge_structures.empty()) {
    const vector<Point2i> nexts = { Point2i(1, 0), Point2i(0, 1) };
    const vector<Point2i> grid_neighbor = { Point2i(0, -1), Point2i(-1, 0) };
    const int memory = DIMENSION_2D * nh * nw + nh + nw;
    edge_structures.resize(memory);
    int index = 0;
    for (int h = 0; h <= nh; h ++) {
      for (int w = 0; w <= nw; w ++) {
        Point2i p1(w, h);
        for (int n = 0; n < nexts.size(); n ++) {
          Point2i p2 = p1 + nexts[n];
          if (p2.x >= 0 && p2.y >= 0 && p2.x <= nw && p2.y <= nh) {
            for (int j = 0; j < grid_neighbor.size(); j ++) {
              Point2i p3 = p1 + grid_neighbor[n] * j;
              if (p3.x >= 0 && p3.y >= 0 && p3.x < nw && p3.y < nh) {
                edge_structures[index].emplace_back(p3.x + p3.y * nw);
              }
            }
            index ++;
          }
        }
      }
    }
    assert(memory == edge_structures.size());
  }
  return edge_structures;
}

InterpolateVertex ImageData::getInterpolateVertex(const Point2f & _p) {
  const vector<Point2f> vertices = getVertices();
  const vector<vector<int> > & grids = getPolygonsIndices();

  const int grid_index = getGridIndexOfPoint(_p);

  const vector<int> & g = grids[grid_index];// TODO Indices

  const vector<int> diagonal_indices = {2, 3, 0, 1};/* 0 1    2 3
                                                       ->
                                                       3 2    1 0 */
  assert(g.size() == GRID_VERTEX_SIZE);

  vector<double> weights(GRID_VERTEX_SIZE);
  double sum_inv = 0;
  for (int i = 0; i < diagonal_indices.size(); i ++) {
    Point2f tmp(_p.x - vertices[g[diagonal_indices[i]]].x,
        _p.y - vertices[g[diagonal_indices[i]]].y);
    weights[i] = fabs(tmp.x * tmp.y);
    sum_inv += weights[i];
  }
  sum_inv = 1. / sum_inv;
  for (int i = 0; i < GRID_VERTEX_SIZE; i ++) {
    weights[i] = weights[i] * sum_inv;
  }
  return InterpolateVertex(grid_index, weights);// 构造函数
}