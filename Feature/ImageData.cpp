#include "ImageData.h"

void ImageData::init_data(const char *img_path) {
  get_img(img_path);
  get_size();

  assert(data.empty() == false);
  assert(alpha_mask.empty() == false);
  assert(mesh_points.empty() == false);
  assert(polygons_indices.empty() == false);
  assert(triangulation_indices.empty() == false);
  assert(edges.empty() == false);
  assert(vertex_structures.empty() == false);
}

void ImageData::get_img(const char *img_path) {
  data = imread(img_path);
  rgba_data = imread(img_path, IMREAD_UNCHANGED);
  grey_data = Mat();// 灰色图
  cvtColor(data, grey_data, CV_BGR2GRAY);
    
  float original_img_size = data.rows * data.cols;
  
  if(original_img_size > DOWN_SAMPLE_IMAGE_SIZE) {
      float scale = sqrt(DOWN_SAMPLE_IMAGE_SIZE / original_img_size);
      resize(data, data, Size(), scale, scale);
      resize(rgba_data, rgba_data, Size(), scale, scale);
  }
  
  assert(rgba_data.channels() >= 3);
  if(rgba_data.channels() == 3) {
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

vector<Point2f> ImageData::getMesh2dPoints() {
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
    triangulation_indices[0].emplace_back(1);
    triangulation_indices[0].emplace_back(3);
    triangulation_indices[0].emplace_back(2);
    triangulation_indices[1].emplace_back(3);
    triangulation_indices[1].emplace_back(0);
    triangulation_indices[1].emplace_back(1);
  }
  return triangulation_indices;
}

vector<vector<int> > ImageData::getPolygonsIndices() {
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

vector<vector<int> > ImageData::getPolygonsCenter() {
  ;
}

vector<Point2f> ImageData::getVertexStructures() {
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

InterpolateVertex ImageData::getInterpolateVertex(const Point2f & _p) {
  const vector<Point2f> vertices = mesh_points;
  const vector<vector<int> > & grids = polygons_indices;

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