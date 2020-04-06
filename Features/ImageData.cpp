#include "ImageData.h"

void ImageData::init_data(const char *img_path) {
  get_img(img_path);
  get_size();
  get_mesh2d_points();
  get_triangulation_indices();
  get_polygons_indices();
  assert(data.empty() == false);
  assert(alpha_mask.empty() == false);
  assert(mesh_points.empty() == false);
  assert(polygons_indices.empty() == false);
  assert(triangulation_indices.empty() == false);
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

/****************************************/

void ImageData::get_size() {
  nw = data.cols / GRID_SIZE + (data.cols % GRID_SIZE != 0);
  nh = data.rows / GRID_SIZE + (data.rows % GRID_SIZE != 0);
  lw = data.cols / (double)nw;
  lh = data.rows / (double)nh;
}

/****************************************/

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

void ImageData::get_triangulation_indices() {
  triangulation_indices.resize(2);
  triangulation_indices[0].emplace_back(0);
  triangulation_indices[0].emplace_back(1);
  triangulation_indices[0].emplace_back(2);
  triangulation_indices[1].emplace_back(0);
  triangulation_indices[1].emplace_back(2);
  triangulation_indices[1].emplace_back(3);
}

void ImageData::get_polygons_indices() {
  const Point2i nexts[GRID_VERTEX_SIZE] = {
      Point2i(0, 0), Point2i(1, 0), Point2i(1, 1), Point2i(0, 1)
  };
  const int memory = nh * nw;// TODO
  polygons_indices.resize(memory);
  int index = 0;
  for (int h = 0; h < nh; h ++) {// TODO
      for (int w = 0; w < nw; w ++) {// TODO
          const Point2i p1(w, h);
          polygons_indices[index].reserve(GRID_VERTEX_SIZE);
          for (int n = 0; n < GRID_VERTEX_SIZE; n ++) {
              const Point2i p2 = p1 + nexts[n];
              polygons_indices[index].emplace_back(p2.x + p2.y * (nw + 1));
          }
          index ++;
      }
  }
  assert(memory == polygons_indices.size());
}