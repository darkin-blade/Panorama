#include "ImageData.h"

void ImageData::initData() {
  descriptors.clear();
  feature_points.clear();
  rows = cols = 0;// 网格顶点的行列数目 
  vertices.clear();
  triangle_indices.clear();
  rectangle_indices.clear();
  homographies.clear();
}

void ImageData::readImg(const Mat & _img, int mode) {
  LOG("origin channels %d", _img.channels());
  _img.copyTo(data);
  float original_img_size = data.rows * data.cols;

  // 缩放
  if (mode == 1) {
    if (original_img_size > DOWN_SAMPLE_IMAGE_SIZE) {
      float scale = sqrt(DOWN_SAMPLE_IMAGE_SIZE / original_img_size);
      resize(data, data, Size(), scale, scale);
    }
  }

  if (mode == 1) {
    // 原图
    mask = Mat(data.size(), CV_8UC1, Scalar(255));
  } else if (mode == 2) {
    // 需手动提取mask
    assert(data.channels() == 4);
    vector<Mat> channels;
    split(data, channels);
    channels[3].copyTo(mask);

    // TODO 检查黑边
    Mat tmp_result = Mat(data.size(), CV_8UC4, Scalar::all(255));
    show_img("mask", mask);
    Mat tmp_img;
    cvtColor(data, tmp_img, COLOR_RGB2RGBA);
    tmp_img.copyTo(tmp_result, mask);
    show_img("last", tmp_result);
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