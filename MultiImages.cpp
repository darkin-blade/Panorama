#include "MultiImages.h"

MultiImages::MultiImages() {
  img_num = 0;
}

void MultiImages::read_img(const char *img_path) {
  // 读取图片
  ImageData *imageData = new ImageData();
  imageData->data = imread(img_path);

  // 检查图片数量
  imgs.push_back(imageData);
  img_num ++;
  assert(img_num == imgs.size());
}