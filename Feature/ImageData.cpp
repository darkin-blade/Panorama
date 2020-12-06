#include "ImageData.h"

void ImageData::readImg(const char *img_path) {
  data = imread(img_path);
  rgba_data = imread(img_path, IMREAD_UNCHANGED);
  grey_data = Mat();// 灰色图
  cvtColor(data, grey_data, CV_BGR2GRAY);

  LOG("origin channels %d", data.channels());
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

void ImageData::initVertices(vector<double> _col, vector<double> _row) {
  // 将原始图像按照_col, _row中的比例划分网格
  cols = _col.size();
  rows = _row.size();
  assert(cols >= 2);
  assert(rows >= 2);

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
  for (int i = 0; i < cols - 1; i ++) {
    for (int j = 0; j < rows - 1; j ++) {
      // 将一个mesh分为两个三角形记录
      vector<int> indice_1;
      vector<int> indice_2;
      // 左上, 右上, 右下
      indice_1.emplace_back(j * rows + i);
      indice_1.emplace_back(j * rows + (i + 1));
      indice_1.emplace_back((j + 1) * rows + (i + 1));
      // 左上, 左下, 右下
      indice_2.emplace_back(j * rows + i);
      indice_2.emplace_back((j + 1) * rows + i);
      indice_2.emplace_back((j + 1) * rows + (i + 1));
      triangle_indices.emplace_back(indice_1);
      triangle_indices.emplace_back(indice_2);
      // 将一个mesh分为一个矩形记录
      vector<int> indice_3;
      // 左上, 右上, 右下, 左下
      indice_3.emplace_back(j * rows + i);
      indice_3.emplace_back(j * rows + (i + 1));
      indice_3.emplace_back((j + 1) * rows + (i + 1));
      indice_3.emplace_back((j + 1) * rows + i);
      rectangle_indices.emplace_back(indice_3);
    }
  }
}