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